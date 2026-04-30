using System.Collections.Concurrent;
using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Numerics;
using System.Text;
using static SlimeImuProtocol.SlimeVR.FirmwareConstants;

namespace SlimeImuProtocol.SlimeVR
{
    public class UDPHandler : IDisposable
    {
        // Configured discovery target — broadcast at boot, narrowed to the resolved server
        // IP after handshake. Static because the user-visible "server endpoint" setting is
        // global, but each handler also tracks its own resolved IP in _endpoint below so
        // multiple trackers can route to different addresses without trampling each other.
        private static string _configuredEndpoint = "255.255.255.255";
        // Resolved IP for THIS handler's UdpClient. Was previously a static field shared
        // across every UDPHandler instance, which meant the first handshake reply for any
        // tracker clobbered the discovery target for all others — multi-tracker setups
        // routinely lost packets after the second device connected.
        private string _endpoint = _configuredEndpoint;
        // 0 = idle, 1 = handshake in progress. Serialized via Interlocked.CompareExchange to
        // prevent two handlers from racing into simultaneous discovery.
        private static int _handshakeOngoingFlag = 0;
        // Tracks whether THIS instance is currently the holder of the static handshake
        // flag. Without this, Dispose() unconditionally cleared the flag — so a Dispose on
        // any handler released the lock and let two handlers race into discovery again.
        private bool _iOwnHandshakeFlag;
        public static event EventHandler OnForceHandshake;
        public static event EventHandler OnForceDestroy;
        public static event EventHandler<string> OnServerDiscovered;
        private Stopwatch _timeSinceLastQuaternionDataPacket = new Stopwatch();
        private Stopwatch _timeSinceLastAccelerationDataPacket = new Stopwatch();
        private string _id;
        private byte[] _hardwareAddress;
        private int _supportedSensorCount;
        private PacketBuilder packetBuilder;
        private int slimevrPort = 6969;
        UdpClient udpClient;
        // Protects the udpClient reference during Configure/Dispose swaps. Reads take the lock
        // briefly to snapshot the reference, then operate on the snapshot — prevents
        // ObjectDisposedException races when Configure runs while a Send is in flight.
        private readonly object _udpClientLock = new object();
        int handshakeCount = 1000;
        bool _active = true;
        private bool disposed;
        private EventHandler forceHandShakeDelegate;
        private Vector3 _lastAccelerationPacket;
        private Quaternion _lastQuaternion;

        private long _lastPacketReceivedTime = DateTimeOffset.UtcNow.ToUniversalTime().ToUnixTimeMilliseconds();
        private bool _isInitialized = false;
        // Two-step discovery: first reply lands on the broadcast UdpClient (source port X);
        // we then swap to a unicast UdpClient (source port Y) targeted at the resolved server
        // IP. Server tracks tracker identity by (clientIp, sourcePort), so SENSOR_INFO/
        // heartbeat/rotation from port Y without a prior HANDSHAKE on port Y get silently
        // dropped — the tracker never appears in the dashboard. After the swap we resend
        // HANDSHAKE on port Y; the server's reply re-enters this branch with _endpointPinned
        // already set, and only then do we mark the handler initialized. Reset on every
        // explicit re-handshake (Rehandshake / watchdog timeout already drop _isInitialized).
        private bool _endpointPinned = false;
        private CancellationTokenSource _cts = new CancellationTokenSource();
        public bool IsDiscoveryOnly { get; set; } = false;

        // Packet telemetry counters surfaced to callers (UI / diagnostics). Incremented in
        // every SendAsync path via SendInternal; ServerReachable reflects handshake success +
        // absence of recent send failures.
        private long _packetsSent;
        private long _sendFailures;
        // Set from the server's FEATURE_FLAGS reply. Gates use of BUNDLE (type 100). Default
        // false until the reply lands — otherwise first packets would silently drop on a
        // server that doesn't advertise PROTOCOL_BUNDLE_SUPPORT. Volatile so the send hot
        // path reads the latest value without a lock.
        private volatile bool _serverSupportsBundle;
        // bit 1 of server flags. Gates BUNDLE_COMPACT (pkt 101 + Q15/Q7 frames). Half-bandwidth
        // path; only enabled once both this flag is true and the user opts in via configuration.
        private volatile bool _serverSupportsBundleCompact;
        public long PacketsSent => System.Threading.Interlocked.Read(ref _packetsSent);
        public long SendFailures => System.Threading.Interlocked.Read(ref _sendFailures);
        public bool ServerReachable => _isInitialized;
        public bool ServerSupportsBundle => _serverSupportsBundle;
        public bool ServerSupportsBundleCompact => _serverSupportsBundleCompact;

        /// <summary>
        /// Opt-in flag for using BUNDLE_COMPACT (pkt 101) when the server advertises support.
        /// Off by default because it's a new wire path — flipping to true on a misconfigured
        /// server would silently drop trackers. Recommended to enable globally only after a
        /// full test cycle. When false, falls back to BUNDLE (pkt 100) or two-packet sends.
        /// </summary>
        public static bool BundleCompactEnabled { get; set; } = false;

        /// <summary>
        /// Server-issued SET_CONFIG_FLAG (pkt 25) request. Tracker classes can subscribe to
        /// honour the request — for example, toggle the magnetometer enable on a JC2. The
        /// handler is invoked AFTER UDPHandler has already ACKed the request to the server,
        /// so listeners only need to apply the local-side change.
        /// </summary>
        public event EventHandler<ConfigFlagRequest> OnConfigFlagRequested;

        /// <summary>
        /// Unified send path. Counts success/failure so UI diagnostics don't lie. Callers use
        /// this instead of udpClient.SendAsync directly.
        /// </summary>
        private async Task SendInternal(ReadOnlyMemory<byte> payload)
        {
            if (disposed) return;
            UdpClient client;
            lock (_udpClientLock) { client = udpClient; }
            if (client == null) return;
            try
            {
                await client.SendAsync(payload);
                System.Threading.Interlocked.Increment(ref _packetsSent);
            }
            catch
            {
                System.Threading.Interlocked.Increment(ref _sendFailures);
            }
        }

        private async Task SendInternal(byte[] payload)
        {
            if (disposed) return;
            UdpClient client;
            lock (_udpClientLock) { client = udpClient; }
            if (client == null) return;
            try
            {
                await client.SendAsync(payload, payload.Length);
                System.Threading.Interlocked.Increment(ref _packetsSent);
            }
            catch
            {
                System.Threading.Interlocked.Increment(ref _sendFailures);
            }
        }

        public bool Active { get => _active; set => _active = value; }
        public static string Endpoint { get => _configuredEndpoint; set => _configuredEndpoint = value; }
        public static bool HandshakeOngoing => System.Threading.Volatile.Read(ref _handshakeOngoingFlag) != 0;

        /// <summary>
        /// Force the handshake state machine to re-run. Safe no-op if already initializing.
        /// </summary>
        public void Rehandshake() => TriggerHandshake();

        public UDPHandler(string firmware, byte[] hardwareAddress, BoardType boardType, ImuType imuType, McuType mcuType, MagnetometerStatus magnetometerStatus, int supportedSensorCount)
        {
            _id = Guid.NewGuid().ToString();
            _hardwareAddress = hardwareAddress;
            _supportedSensorCount = supportedSensorCount;
            packetBuilder = new PacketBuilder(firmware);
            ConfigureUdp();
            
            // Start the unified background loops
            Task.Run(() => ReceiveLoop(boardType, imuType, mcuType, magnetometerStatus, macAddress: hardwareAddress));
            Task.Run(() => WatchdogLoop(hardwareAddress, boardType, imuType, mcuType, magnetometerStatus, supportedSensorCount));
            Task.Run(() => HeartbeatLoop());

            forceHandShakeDelegate = delegate (object o, EventArgs e)
            {
                TriggerHandshake();
            };
            OnForceHandshake += forceHandShakeDelegate;
            OnForceDestroy += UDPHandler_OnForceDestroy;
        }

        private void TriggerHandshake()
        {
            _isInitialized = false;
            // Drop the pin so an explicit re-handshake re-runs the swap-and-resend dance
            // against whatever server replies first — the old server may have moved or
            // the user may have changed Endpoint while we were idle.
            _endpointPinned = false;
        }

        private void UDPHandler_OnForceDestroy(object? sender, EventArgs e)
        {
            OnForceHandshake -= forceHandShakeDelegate;
            OnForceDestroy -= UDPHandler_OnForceDestroy;
            this?.Dispose();
        }

        public static void ForceUDPClientsToDoHandshake()
        {
            OnForceHandshake?.Invoke(new object(), EventArgs.Empty);
        }

        public static void ForceDestroy()
        {
            OnForceDestroy?.Invoke(new object(), EventArgs.Empty);
        }

        private async Task WatchdogLoop(byte[] hardwareAddress, BoardType boardType, ImuType imuType, McuType mcuType, MagnetometerStatus magnetometerStatus, int supportedSensorCount)
        {
            var token = _cts.Token;
            while (!disposed && !token.IsCancellationRequested)
            {
                if (!_active)
                {
                    try { await Task.Delay(1000, token); } catch (OperationCanceledException) { return; }
                    continue;
                }

                if (!_isInitialized)
                {
                    // Sequential handshake — one handler at a time. Atomic CAS prevents two
                    // handlers racing into discovery simultaneously (prior bool flag raced).
                    while (System.Threading.Interlocked.CompareExchange(ref _handshakeOngoingFlag, 1, 0) != 0 && !disposed)
                    {
                        await Task.Delay(1000);
                    }
                    _iOwnHandshakeFlag = true;

                    if (disposed) break;

                    try
                    {
                        Debug.WriteLine($"[UDPHandler] Starting Handshake for {_id}...");

                        // Exponential backoff for the handshake loop. Old code blasted the
                        // configured endpoint with a packet every 1s indefinitely — when the
                        // server is offline or the network is partitioned that floods syslog,
                        // wakes power-saving NICs, and (on broadcast targets) bothers every
                        // peer on the LAN. Start at 500ms, double up to 30s, and never give
                        // up — the server might come online later.
                        const int handshakeBackoffStartMs = 500;
                        const int handshakeBackoffCapMs = 30_000;
                        int handshakeBackoffMs = handshakeBackoffStartMs;
                        while (!_isInitialized && _active && !disposed)
                        {
                            // Check if endpoint changed during handshake attempt
                            if (udpClient == null || _endpoint != Endpoint)
                            {
                                ConfigureUdp();
                            }

                            await SendInternal(packetBuilder.BuildHandshakePacket(boardType, imuType, mcuType, magnetometerStatus, hardwareAddress));
                            await Task.Delay(handshakeBackoffMs);
                            handshakeBackoffMs = Math.Min(handshakeBackoffMs * 2, handshakeBackoffCapMs);
                        }
                    }
                    catch (Exception ex)
                    {
                        Debug.WriteLine($"[UDPHandler] Handshake error for {_id}: {ex.Message}");
                    }
                    finally
                    {
                        // Only release the static flag if this instance is the holder. The
                        // previous unconditional Exchange let any Dispose-during-handshake
                        // clear the lock for whichever instance happened to be running, and
                        // a second instance immediately raced in.
                        if (_iOwnHandshakeFlag)
                        {
                            _iOwnHandshakeFlag = false;
                            System.Threading.Interlocked.Exchange(ref _handshakeOngoingFlag, 0);
                        }
                    }

                    if (_isInitialized)
                    {
                        Debug.WriteLine($"[UDPHandler] Handshake Success for {_id}. Sending sensor info...");
                        for (int i = 0; i < _supportedSensorCount; i++)
                        {
                            await SendInternal(packetBuilder.BuildSensorInfoPacket(imuType, TrackerPosition.NONE, TrackerDataType.ROTATION, (byte)i, magnetometerStatus));
                        }

                        // Advertise supported protocol features so the server knows it can use
                        // bundled packets for us. Without this, the server falls back to
                        // single-packet-per-datagram mode even if we send bundles ourselves.
                        try
                        {
                            // Advertise tracker-side capabilities. Bit 2 = SENSOR_CONFIG so the
                            // server knows it can issue SET_CONFIG_FLAG (pkt 25) — for example
                            // to toggle the magnetometer remotely. We do NOT advertise
                            // REMOTE_COMMAND or B64_WIFI_SCANNING since the bridge can't honour
                            // those. Previous code mistakenly sent server-namespace bit 0; both
                            // were ignored by the server but the new value is correct.
                            await SendInternal(packetBuilder.BuildFeatureFlagsPacket(
                                UDPPackets.FirmwareFeatureFlagBits.SENSOR_CONFIG));
                        }
                        catch (Exception ex) { Debug.WriteLine($"[UDPHandler] FeatureFlags send failed: {ex.Message}"); }

                        if (IsDiscoveryOnly)
                        {
                            Debug.WriteLine($"[UDPHandler] Discovery Only mode active. Disposing {_id}...");
                            Dispose();
                            break;
                        }
                    }
                }
                else if (udpClient != null && _endpoint != Endpoint)
                {
                    // Endpoint changed after initialization, re-handshake to new target
                    Debug.WriteLine($"[UDPHandler] Endpoint changed for {_id}. Re-configuring...");
                    _isInitialized = false;
                    ConfigureUdp();
                }

                // Connection persistence watchdog: 4 seconds
                long now = DateTimeOffset.UtcNow.ToUniversalTime().ToUnixTimeMilliseconds();
                if (_isInitialized && (now - _lastPacketReceivedTime > 4000))
                {
                    Debug.WriteLine($"[UDPHandler] Connection TIMEOUT for {_id}. Re-handshaking...");
                    _isInitialized = false;
                    _lastPacketReceivedTime = now; // Prevent instant re-trigger
                }

                await Task.Delay(1000);
            }
        }

        private async Task ReceiveLoop(BoardType boardType, ImuType imuType, McuType mcuType, MagnetometerStatus magnetometerStatus, byte[] macAddress)
        {
            var token = _cts.Token;
            while (!disposed && !token.IsCancellationRequested)
            {
                UdpClient client;
                lock (_udpClientLock) { client = udpClient; }
                if (client == null)
                {
                    try { await Task.Delay(100, token); } catch (OperationCanceledException) { return; }
                    continue;
                }
                try
                {
                    var result = await client.ReceiveAsync(token);
                    _lastPacketReceivedTime = DateTimeOffset.UtcNow.ToUniversalTime().ToUnixTimeMilliseconds();

                    byte[] buffer = result.Buffer;
                    if (buffer.Length == 0) continue;

                    // Parse packet type from first 4 bytes (big-endian int32). Handshake reply
                    // uses the same type as the outgoing request (RECEIVE_HANDSHAKE=3) followed
                    // by a version string payload. Relying on the type first is robust against
                    // the server changing its version string ("Hey OVR =D 5" etc).
                    uint packetType = buffer.Length >= 4
                        ? (uint)((buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3])
                        : uint.MaxValue;

                    bool looksLikeHandshakeAscii =
                        packetType == UDPPacketsIn.RECEIVE_HANDSHAKE
                        || (buffer.Length >= 12 && Encoding.ASCII.GetString(buffer, 0, Math.Min(buffer.Length, 16)).Contains("Hey OVR"));

                    if (looksLikeHandshakeAscii && !_isInitialized)
                    {
                        string serverIp = result.RemoteEndPoint.Address.ToString();
                        if (!_endpointPinned)
                        {
                            // Swap from the broadcast UdpClient to a unicast one targeted at
                            // the responding server. Connect() in-place would have raced
                            // concurrent SendAsync (rebinds the underlying socket), so we use
                            // a full client swap via ConfigureUdp — old client is closed only
                            // after the new one is published. The new socket has a fresh
                            // ephemeral source port; the server keys tracker identity on
                            // (clientIp, sourcePort), so we MUST re-send HANDSHAKE on the new
                            // socket before sending SENSOR_INFO. Skipping that step (the
                            // regression introduced in 8a17d02) left the server with a stale
                            // registration on the old port and ignored everything that came
                            // from the new port — tracker never appeared in the dashboard.
                            _endpoint = serverIp;
                            ConfigureUdp();
                            _endpointPinned = true;
                            await SendInternal(packetBuilder.BuildHandshakePacket(
                                boardType, imuType, mcuType, magnetometerStatus, macAddress));
                            Debug.WriteLine($"[UDPHandler] Pinned to server {_endpoint} for {_id}; re-handshaking on swapped socket");
                            continue;
                        }
                        // Second reply — this one arrived on the unicast socket, so the
                        // server has registered the (clientIp, sourcePort) for the new
                        // socket. Safe to mark initialized.
                        _endpoint = serverIp;
                        _isInitialized = true;
                        Debug.WriteLine($"[UDPHandler] Got Discovery Response for {_id}: {_endpoint}");
                        OnServerDiscovered?.Invoke(null, _endpoint);
                        continue;
                    }

                    if (packetType == UDPPackets.PING_PONG)
                    {
                        await SendInternal(buffer); // echo same buffer back
                    }
                    else if (packetType == UDPPacketsIn.RECEIVE_HEARTBEAT)
                    {
                        await SendInternal(packetBuilder.CreateHeartBeat());
                    }
                    else if (packetType == UDPPackets.FEATURE_FLAGS)
                    {
                        // Server reply layout: header(4) + packetId(8) + flagBytes(variable).
                        // Bits use the SERVER namespace (ServerFeatureFlagBits) — distinct
                        // from the FirmwareFeatureFlagBits we advertise outbound. Bit 0 =
                        // PROTOCOL_BUNDLE_SUPPORT, bit 1 = PROTOCOL_BUNDLE_COMPACT_SUPPORT.
                        // Without the reply, bundle is disabled by default to avoid silent drops.
                        if (buffer.Length >= 13)
                        {
                            byte flagByte0 = buffer[12];
                            bool bundleBit = (flagByte0 & (1 << UDPPackets.ServerFeatureFlagBits.PROTOCOL_BUNDLE_SUPPORT)) != 0;
                            bool bundleCompactBit = (flagByte0 & (1 << UDPPackets.ServerFeatureFlagBits.PROTOCOL_BUNDLE_COMPACT_SUPPORT)) != 0;
                            _serverSupportsBundle = bundleBit;
                            _serverSupportsBundleCompact = bundleCompactBit;
                            Debug.WriteLine($"[UDPHandler] Server FEATURE_FLAGS: bundle={bundleBit} bundleCompact={bundleCompactBit}");
                        }
                    }
                    else if (packetType == UDPPackets.SET_CONFIG_FLAG)
                    {
                        // Server→tracker remote config toggle. Layout after header(4)+packetId(8):
                        //   sensorId   (1 byte)
                        //   configType (2 bytes BE u16)
                        //   state      (1 byte 0/1)
                        // We accept the packet, log it, and ACK so the server's pending change
                        // doesn't time out. Actual application of the config (e.g. mag enable)
                        // is per-source business logic — surfaced via OnConfigFlagRequested.
                        if (buffer.Length >= 16)
                        {
                            byte sensorId = buffer[12];
                            ushort configType = (ushort)((buffer[13] << 8) | buffer[14]);
                            bool state = buffer[15] != 0;
                            Debug.WriteLine($"[UDPHandler] SET_CONFIG_FLAG sensor={sensorId} type={configType} state={state}");
                            try { OnConfigFlagRequested?.Invoke(this, new ConfigFlagRequest(sensorId, configType, state)); }
                            catch (Exception evEx) { Debug.WriteLine($"[UDPHandler] OnConfigFlagRequested handler threw: {evEx.Message}"); }
                            try { await SendInternal(packetBuilder.BuildAckConfigChangePacket(sensorId, configType)); }
                            catch (Exception ackEx) { Debug.WriteLine($"[UDPHandler] ACK_CONFIG_CHANGE send failed: {ackEx.Message}"); }
                        }
                    }
                }
                catch (OperationCanceledException) { return; }
                catch (ObjectDisposedException) { return; }
                catch (Exception ex) when (!disposed)
                {
                    Debug.WriteLine($"[UDPHandler] Receive Error: {ex.Message}");
                    try { await Task.Delay(100, token); } catch (OperationCanceledException) { return; }
                }
            }
        }

        private async Task HeartbeatLoop()
        {
            var token = _cts.Token;
            while (!disposed && !token.IsCancellationRequested)
            {
                if (_active && _isInitialized)
                {
                    await SendInternal(packetBuilder.CreateHeartBeat());
                }
                try { await Task.Delay(900, token); } catch (OperationCanceledException) { return; }
            }
        }

        // Re-entry guard. Concurrent ConfigureUdp calls (watchdog re-handshake + receive-loop
        // discovery landing simultaneously) used to create two UdpClients each lap; the loser
        // leaked because oldClient under each lock arm captured the winner's fresh client.
        private readonly object _configureLock = new object();

        public void ConfigureUdp()
        {
            lock (_configureLock)
            {
                UdpClient oldClient;
                UdpClient newClient = null;
                lock (_udpClientLock)
                {
                    oldClient = udpClient;
                    udpClient = null; // readers take this as "no client available" until new one in place
                }
                try
                {
                    // If discovery hasn't resolved yet, fall back to the configured (broadcast
                    // by default) endpoint. After handshake, _endpoint holds this handler's
                    // own resolved server IP — independent of every other handler's choice.
                    if (string.IsNullOrEmpty(_endpoint)) _endpoint = _configuredEndpoint;
                    newClient = new UdpClient();
                    newClient.Connect(_endpoint, 6969);
                    lock (_udpClientLock) { udpClient = newClient; }
                    Debug.WriteLine($"[UDPHandler] Configured UDP for {_id} -> {_endpoint}");
                }
                catch (Exception ex)
                {
                    try { newClient?.Dispose(); } catch { }
                    Debug.WriteLine($"[UDPHandler] ConfigureUdp error for {_id}: {ex.Message}");
                }
                finally
                {
                    try { oldClient?.Close(); } catch { }
                    try { oldClient?.Dispose(); } catch { }
                }
            }
        }

        public async Task<bool> SetSensorRotation(Quaternion rotation, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await SendInternal(packetBuilder.BuildRotationPacket(rotation, trackerId));
                _lastQuaternion = rotation;
            }
            return true;
        }

        public static bool QuatEqualsWithEpsilon(Quaternion a, Quaternion b)
        {
            const float epsilon = 0.0001f;
            return MathF.Abs(a.X - b.X) < epsilon
                && MathF.Abs(a.Y - b.Y) < epsilon
                && MathF.Abs(a.Z - b.Z) < epsilon
                && MathF.Abs(a.W - b.W) < epsilon;
        }

        public async Task<bool> SetSensorAcceleration(Vector3 acceleration, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await SendInternal(packetBuilder.BuildAccelerationPacket(acceleration, trackerId));
                _timeSinceLastAccelerationDataPacket.Restart();
                _lastAccelerationPacket = acceleration;
            }
            return true;
        }

        public async Task<bool> SetThumbstick(Vector2 analogueThumbstick, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await SendInternal(packetBuilder.BuildThumbstickPacket(analogueThumbstick, trackerId));
            }
            return true;
        }

        public async Task<bool> SetTrigger(float triggerAnalogue, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await SendInternal(packetBuilder.BuildTriggerAnaloguePacket(triggerAnalogue, trackerId));
            }
            return true;
        }

        public async Task<bool> SetGrip(float gripAnalogue, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await SendInternal(packetBuilder.BuildGripAnaloguePacket(gripAnalogue, trackerId));
            }
            return true;
        }

        public async Task<bool> SetSensorGyro(Vector3 gyro, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await SendInternal(packetBuilder.BuildGyroPacket(gyro, trackerId));
            }
            return true;
        }
        public async Task<bool> SetSensorFlexData(float flexResistance, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await SendInternal(packetBuilder.BuildFlexDataPacket(flexResistance, trackerId));
            }
            return true;
        }
        public async Task<bool> SendButton(UserActionType userActionType)
        {
            if (udpClient != null && _isInitialized)
            {
                await SendInternal(packetBuilder.BuildButtonPushedPacket(userActionType));
            }
            return true;
        }
        public async Task<bool> SendControllerButton(ControllerButton userActionType, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await SendInternal(packetBuilder.BuildControllerButtonPushedPacket(userActionType, trackerId));
            }
            return true;
        }

        public async Task<bool> SendPacket(byte[] packet)
        {
            if (udpClient != null && _isInitialized)
            {
                await SendInternal(packet);
            }
            return true;
        }

        public async Task<bool> SetSensorBattery(float battery, float voltage)
        {
            if (udpClient != null && _isInitialized)
            {
                await SendInternal(packetBuilder.BuildBatteryLevelPacket(battery, voltage));
            }
            return true;
        }

        public async Task<bool> SetSensorMagnetometer(Vector3 magnetometer, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                await SendInternal(packetBuilder.BuildMagnetometerPacket(magnetometer, trackerId));
            }
            return true;
        }

        /// <summary>
        /// Sends N packets as one BUNDLE datagram (type 100). Use when emitting multiple
        /// related updates per tick (e.g. rotation + accel + battery) — saves syscalls and UDP
        /// header overhead. Server must support PROTOCOL_BUNDLE_SUPPORT (advertised via
        /// FEATURE_FLAGS automatically after handshake).
        /// </summary>
        public async Task<bool> SendBundle(params ReadOnlyMemory<byte>[] innerPackets)
        {
            if (udpClient != null && _isInitialized && innerPackets != null && innerPackets.Length > 0)
            {
                await SendInternal(packetBuilder.BuildBundlePacket(innerPackets));
            }
            return true;
        }

        /// <summary>
        /// Hot-path convenience: builds ROTATION_DATA + ACCELERATION in one BUNDLE when the
        /// server has advertised PROTOCOL_BUNDLE_SUPPORT via FEATURE_FLAGS. Falls back to two
        /// separate sends when the flag isn't set — the server silently drops type-100
        /// packets it doesn't recognise, so we can't unconditionally bundle. ServerSupportsBundle
        /// flips true once the server's FEATURE_FLAGS reply reaches ReceiveLoop.
        /// </summary>
        public async Task<bool> SetSensorBundle(Quaternion rotation, Vector3 acceleration, byte trackerId)
        {
            if (udpClient == null || !_isInitialized) return true;

            // Tier 1: BUNDLE_COMPACT (pkt 101 + pkt 23 Q15/Q7). Halves bandwidth; gated on
            // both server advertisement AND opt-in static flag because it's a new wire path.
            if (BundleCompactEnabled && _serverSupportsBundleCompact)
            {
                var inner = packetBuilder.BuildRotationAccelCompactInner(rotation, acceleration, trackerId);
                await SendInternal(packetBuilder.BuildBundleCompactPacket(
                    ((byte)UDPPackets.ROTATION_AND_ACCELERATION_COMPACT, inner)));
                _lastQuaternion = rotation;
                _lastAccelerationPacket = acceleration;
                return true;
            }

            var rot = packetBuilder.BuildRotationPacket(rotation, trackerId);
            var acc = packetBuilder.BuildAccelerationPacket(acceleration, trackerId);
            if (_serverSupportsBundle)
            {
                await SendInternal(packetBuilder.BuildBundlePacket(rot, acc));
            }
            else
            {
                // Send rotation first so the server has a frame to apply the accel sample
                // against. Reversed order produced visible 1-frame jitter on legacy servers
                // because the accel packet was attached to the previous rotation.
                await SendInternal(rot);
                await SendInternal(acc);
            }
            _lastQuaternion = rotation;
            _lastAccelerationPacket = acceleration;
            return true;
        }

        public void Dispose()
        {
            try
            {
                if (!disposed)
                {
                    disposed = true;
                    _isInitialized = false;
                    // Cancel in-flight ReceiveAsync / SendAsync. Without _cts the loops kept
                    // running on a disposed UdpClient until it threw ObjectDisposedException
                    // and the catch swallowed it.
                    try { _cts.Cancel(); } catch { }
                    // Only release the global handshake flag if WE were holding it. The
                    // previous unconditional release let one handler's Dispose unlock
                    // discovery for everyone, racing two parallel handshakes.
                    if (_iOwnHandshakeFlag)
                    {
                        _iOwnHandshakeFlag = false;
                        System.Threading.Interlocked.Exchange(ref _handshakeOngoingFlag, 0);
                    }
                    // Unsubscribe from the static force-handshake/destroy events. Discovery-
                    // only handlers used to leak their delegate into OnForceHandshake for the
                    // lifetime of the process and continued receiving rehandshake calls.
                    try { OnForceHandshake -= forceHandShakeDelegate; } catch { }
                    try { OnForceDestroy -= UDPHandler_OnForceDestroy; } catch { }
                    UdpClient c;
                    lock (_udpClientLock) { c = udpClient; udpClient = null; }
                    try { c?.Close(); } catch { }
                    try { c?.Dispose(); } catch { }
                }
            }
            catch (Exception ex) { Debug.WriteLine($"[UDPHandler] Dispose: {ex.Message}"); }
        }

        public void SendTrigger(float trigger, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                _ = SendInternal(packetBuilder.BuildTriggerAnaloguePacket(trigger, trackerId));
            }
        }

        public void SendGrip(float grip, byte trackerId)
        {
            if (udpClient != null && _isInitialized)
            {
                _ = SendInternal(packetBuilder.BuildGripAnaloguePacket(grip, trackerId));
            }
        }
    }

    /// <summary>
    /// Carries SET_CONFIG_FLAG (pkt 25) request data from <see cref="UDPHandler.OnConfigFlagRequested"/>.
    /// <c>ConfigType</c> identifies which config flag the server wants to toggle (e.g. mag enable);
    /// the exact id↔meaning mapping is managed in SlimeVR-Server's <c>ConfigTypeId</c> registry.
    /// </summary>
    public sealed class ConfigFlagRequest : EventArgs
    {
        public byte SensorId { get; }
        public ushort ConfigType { get; }
        public bool State { get; }
        public ConfigFlagRequest(byte sensorId, ushort configType, bool state)
        {
            SensorId = sensorId;
            ConfigType = configType;
            State = state;
        }
    }
}
