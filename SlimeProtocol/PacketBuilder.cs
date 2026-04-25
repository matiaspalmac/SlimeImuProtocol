using SlimeImuProtocol.SlimeVR;
using System;
using System.Buffers.Binary;
using System.Numerics;
using static SlimeImuProtocol.SlimeVR.FirmwareConstants;

namespace SlimeImuProtocol.SlimeVR
{
    public class PacketBuilder
    {
        private string _identifierString = "Bootleg Tracker";
        private int _protocolVersion = 19;
        private long _packetId;

        // Per-packet sizes — allocated fresh on every Build* call. Previous version reused
        // instance-field buffers, which corrupted silently when two Tasks (e.g. the JSL
        // rotation callback and a concurrent accel send) built packets in parallel before
        // the first one had been sent. Fresh allocations trade ~90 B/packet garbage for
        // correctness. ArrayPool<byte>.Shared.Rent/Return is the optimization path if the
        // allocation rate ever shows up in profiling.
        private const int RotationBufferSize       = 4 + 8 + 1 + 1 + 16 + 1;
        private const int AccelerationBufferSize   = 4 + 8 + 12 + 1;
        private const int StickBufferSize          = 4 + 8 + 12 + 1;
        private const int TouchpadBufferSize       = 4 + 8 + 12 + 1;
        private const int GyroBufferSize           = 4 + 8 + 1 + 1 + 12 + 1;
        private const int MagnetometerBufferSize   = 4 + 8 + 1 + 1 + 12 + 1;
        private const int FlexDataBufferSize       = 4 + 8 + 1 + 4;
        private const int ButtonBufferSize         = 4 + 8 + 1;
        private const int BatteryBufferSize        = 4 + 8 + 4 + 4;
        private const int HapticBufferSize         = 4 + 8 + 4 + 4 + 1;
        private const int ControllerButtonSize     = 4 + 8 + 1 + 1;
        private const int TriggerAnalogueSize      = 4 + 8 + 4 + 1;
        private const int GripAnalogueSize         = 4 + 8 + 4 + 1;
        private const int HeartbeatBufferSize      = 4 + 8 + 1;

        public PacketBuilder(string fwString)
        {
            _identifierString = fwString;
        }

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        private long NextPacketId()
        {
            // Atomic increment: rotation/accel/heartbeat/haptic packets are sent from concurrent Tasks.
            return System.Threading.Interlocked.Increment(ref _packetId) - 1;
        }

        public ReadOnlyMemory<byte> CreateHeartBeat()
        {
            var buf = new byte[HeartbeatBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.HEARTBEAT); // Header
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteByte(0); // Tracker Id
            return buf.AsMemory(0, w.Position);
        }

        /// <summary>
        /// Build pkt 17 ROTATION_DATA. Wire layout matches SlimeVR-Server's
        /// <c>UDPProtocolParser.parseRotationData</c>: quaternion is sent in
        /// <b>(X, Y, Z, W)</b> order. <see cref="System.Numerics.Quaternion"/> stores W last
        /// internally, so the four <c>WriteSingle</c> calls below preserve that ordering on
        /// the wire.
        /// </summary>
        public ReadOnlyMemory<byte> BuildRotationPacket(Quaternion rotation, byte trackerId)
        {
            var buf = new byte[RotationBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.ROTATION_DATA); // Header
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteByte(trackerId); // Tracker id
            w.WriteByte(1); // Data type
            w.WriteSingle(rotation.X); // Quaternion X
            w.WriteSingle(rotation.Y); // Quaternion Y
            w.WriteSingle(rotation.Z); // Quaternion Z
            w.WriteSingle(rotation.W); // Quaternion W
            w.WriteByte(0); // Calibration Info
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildAccelerationPacket(Vector3 acceleration, byte trackerId)
        {
            var buf = new byte[AccelerationBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.ACCELERATION); // Header
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteSingle(acceleration.X); // Accel X (m/s²)
            w.WriteSingle(acceleration.Y); // Accel Y
            w.WriteSingle(acceleration.Z); // Accel Z
            w.WriteByte(trackerId); // Tracker id
            return buf.AsMemory(0, w.Position);
        }
        public ReadOnlyMemory<byte> BuildThumbstickPacket(Vector2 _analogueStick, byte trackerId)
        {
            var buf = new byte[StickBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.THUMBSTICK); // Header
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteSingle(_analogueStick.X); // Analogue X
            w.WriteSingle(_analogueStick.Y); // Analogue Y
            w.WriteSingle(0); // Analogue Z (Unused)
            w.WriteByte(trackerId); // Tracker id
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildTouchpadPacket(Vector2 _analogueTouchpad, byte trackerId)
        {
            var buf = new byte[TouchpadBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.THUMBSTICK); // Header
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteSingle(_analogueTouchpad.X); // Analogue X
            w.WriteSingle(_analogueTouchpad.Y); // Analogue Y
            w.WriteSingle(0); // Analogue Z (Unused)
            w.WriteByte(trackerId); // Tracker id
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildGyroPacket(Vector3 gyro, byte trackerId)
        {
            var buf = new byte[GyroBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.GYRO); // Header
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteByte(trackerId); // Tracker id
            w.WriteByte(1); // Data type
            w.WriteSingle(gyro.X); // Gyro X (rad/s)
            w.WriteSingle(gyro.Y); // Gyro Y
            w.WriteSingle(gyro.Z); // Gyro Z
            w.WriteByte(0); // Calibration Info
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildMagnetometerPacket(Vector3 m, byte trackerId)
        {
            var buf = new byte[MagnetometerBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.MAG); // Header
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteByte(trackerId); // Tracker id
            w.WriteByte(1); // Data type
            w.WriteSingle(m.X); // Mag X (µT)
            w.WriteSingle(m.Y); // Mag Y
            w.WriteSingle(m.Z); // Mag Z
            w.WriteByte(0); // Calibration Info
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildFlexDataPacket(float flex, byte trackerId)
        {
            var buf = new byte[FlexDataBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.FLEX_DATA_PACKET); // Header
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteByte(trackerId); // Tracker id
            w.WriteSingle(flex); // Flex data
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildButtonPushedPacket(UserActionType action)
        {
            var buf = new byte[ButtonBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.CALIBRATION_ACTION); // Header
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteByte((byte)action); // Action type
            return buf.AsMemory(0, w.Position);
        }

        /// <summary>
        /// Builds a BATTERY_LEVEL packet for the SlimeVR server.
        /// Wire layout: header(4) + packetId(8) + voltage(float,volts) + level(float, 0..1).
        /// </summary>
        /// <param name="batteryPercent">Battery percentage in 0..100 range.</param>
        /// <param name="voltageVolts">Battery voltage in volts (e.g. 3.7). Pass a sane default if unknown — zero hides the indicator in SlimeVR UI.</param>
        public ReadOnlyMemory<byte> BuildBatteryLevelPacket(float batteryPercent, float voltageVolts)
        {
            var buf = new byte[BatteryBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.BATTERY_LEVEL);
            w.WriteInt64(NextPacketId());
            w.WriteSingle(voltageVolts <= 0.1f ? 3.7f : voltageVolts);
            w.WriteSingle(Math.Clamp(batteryPercent / 100f, 0f, 1f));
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildHapticPacket(float intensity, int duration)
        {
            var buf = new byte[HapticBufferSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.HAPTICS); // full-width header like every other packet
            w.WriteInt64(NextPacketId());
            w.WriteSingle(intensity);
            w.WriteInt32(duration);
            w.WriteByte(1); // active
            return buf.AsMemory(0, w.Position);
        }

        public byte[] BuildHandshakePacket(BoardType boardType, ImuType imuType, McuType mcuType, MagnetometerStatus magStatus, byte[] mac)
        {
            var idBytes = System.Text.Encoding.UTF8.GetBytes(_identifierString);
            int totalSize = 4 + 8 + 4 * 7 + 1 + idBytes.Length + mac.Length;
            var span = new byte[totalSize];

            var w = new BigEndianWriter(span);
            w.WriteInt32((int)UDPPackets.HANDSHAKE); // Header 
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteInt32((int)boardType); // Board type
            w.WriteInt32((int)imuType); // IMU type
            w.WriteInt32((int)mcuType); // MCU Type
            // SlimeVR firmware layout here is 3 × int32 "IMU Info" slots. For a single-sensor
            // tracker the first slot holds magnetometer status and the rest are reserved/zero.
            w.WriteInt32((int)magStatus); // IMU Info slot 1 (magnetometer status)
            w.WriteInt32(0);              // IMU Info slot 2 (reserved)
            w.WriteInt32(0);              // IMU Info slot 3 (reserved)
            w.WriteInt32(_protocolVersion); // Protocol Version

            // Identifier string
            w.WriteByte((byte)idBytes.Length);  // Identifier Length
            idBytes.CopyTo(span.AsSpan(w.Position)); // Identifier String
            w.Skip(idBytes.Length);

            // MAC address
            mac.CopyTo(span.AsSpan(w.Position)); // MAC Address
            w.Skip(mac.Length);

            return span;
        }

        /// <summary>
        /// SENSOR_INFO (type 15). Layout per SlimeVR-Server UDPPacket15SensorInfo.readData:
        ///   [header(4)][packetId(8)][sensorId byte][sensorStatus byte][sensorType byte]
        ///   [sensorConfig uint16 BE][hasCompletedRestCalibration byte][trackerPosition byte]
        ///   [trackerDataType byte]
        /// SensorConfig bit packing:
        ///   bit 0 = magnetometer enable (1=on), bit 1 = magnetometer supported (1=present).
        ///   ENABLED=0x03, DISABLED=0x02, NOT_SUPPORTED=0x00.
        /// Previous version omitted sensorConfig + hasCompletedRestCalibration and wrote a
        /// hardcoded "calibration state" int16 in their place, which the server parsed as
        /// sensorConfig=1 → bit 1 unset → NOT_SUPPORTED regardless of what we passed in the
        /// handshake's magStatus field (the server doesn't read mag from the handshake, only
        /// from this packet).
        /// </summary>
        public byte[] BuildSensorInfoPacket(ImuType imuType, TrackerPosition pos, TrackerDataType dataType, byte trackerId, MagnetometerStatus magStatus = MagnetometerStatus.NOT_SUPPORTED)
        {
            ushort sensorConfig = magStatus switch
            {
                MagnetometerStatus.ENABLED => 0x0003,        // bit 1 + bit 0
                MagnetometerStatus.DISABLED => 0x0002,       // bit 1 only
                _ => 0x0000,                                 // NOT_SUPPORTED
            };
            var span = new byte[4 + 8 + 1 + 1 + 1 + 2 + 1 + 1 + 1];
            var w = new BigEndianWriter(span);
            w.SetPosition(0);
            w.WriteInt32((int)UDPPackets.SENSOR_INFO);
            w.WriteInt64(NextPacketId());
            w.WriteByte(trackerId);
            w.WriteByte(0);                      // sensor status (0 = OK)
            w.WriteByte((byte)imuType);
            w.WriteInt16((short)sensorConfig);   // sensorConfig bitmask — carries magStatus
            w.WriteByte(0);                      // hasCompletedRestCalibration (0 = not yet)
            w.WriteByte((byte)pos);
            w.WriteByte((byte)dataType);
            return span;
        }

        public ReadOnlyMemory<byte> BuildControllerButtonPushedPacket(ControllerButton action, byte trackerId)
        {
            var buf = new byte[ControllerButtonSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.CONTROLLER_BUTTON); // Header
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteByte((byte)action); // Action type
            w.WriteByte(trackerId); // Tracker id
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildTriggerAnaloguePacket(float triggerAnalogue, byte trackerId)
        {
            var buf = new byte[TriggerAnalogueSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.TRIGGER); // Header
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteSingle(triggerAnalogue); // Trigger value 0..1
            w.WriteByte(trackerId); // Tracker id
            return buf.AsMemory(0, w.Position);
        }

        public ReadOnlyMemory<byte> BuildGripAnaloguePacket(float gripAnalogue, byte trackerId)
        {
            var buf = new byte[GripAnalogueSize];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.GRIP); // Header
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteSingle(gripAnalogue); // Grip value 0..1
            w.WriteByte(trackerId); // Tracker id
            return buf.AsMemory(0, w.Position);
        }

        /// <summary>
        /// FEATURE_FLAGS packet (type 22). Advertises tracker-side capabilities to the
        /// server. Layout: header(4) + packetId(8) + flagBytes(variable, LSB0 bit order).
        /// Tracker-side bits live in <see cref="FirmwareConstants.UDPPackets.FirmwareFeatureFlagBits"/>
        /// (REMOTE_COMMAND=0, B64_WIFI_SCANNING=1, SENSOR_CONFIG=2). Server-side bits read
        /// from inbound pkt 22 use <see cref="FirmwareConstants.UDPPackets.ServerFeatureFlagBits"/>.
        /// </summary>
        public byte[] BuildFeatureFlagsPacket(params int[] enabledFlagBits)
        {
            int maxBit = 0;
            foreach (var b in enabledFlagBits) if (b > maxBit) maxBit = b;
            int flagByteCount = Math.Max(1, (maxBit / 8) + 1);

            var buf = new byte[4 + 8 + flagByteCount];
            var w = new BigEndianWriter(buf);
            w.SetPosition(0);
            w.WriteInt32((int)UDPPackets.FEATURE_FLAGS);
            w.WriteInt64(NextPacketId());
            foreach (var bit in enabledFlagBits)
            {
                int byteIdx = bit / 8;
                int bitIdx = bit % 8;
                buf[12 + byteIdx] |= (byte)(1 << bitIdx);
            }
            return buf;
        }

        /// <summary>
        /// ACK_CONFIG_CHANGE packet (type 24). Tracker→server response to a SET_CONFIG_FLAG
        /// request. Echoes the sensorId + configType so the server can correlate the ack
        /// with its own pending change. Server's <c>UDPPacket24AckConfigChange.readData</c>
        /// reads sensorId (1B) + configType (2B BE u16).
        /// </summary>
        public byte[] BuildAckConfigChangePacket(byte sensorId, ushort configType)
        {
            var buf = new byte[4 + 8 + 1 + 2];
            var w = new BigEndianWriter(buf);
            w.SetPosition(0);
            w.WriteInt32((int)UDPPackets.ACK_CONFIG_CHANGE);
            w.WriteInt64(NextPacketId());
            w.WriteByte(sensorId);
            w.WriteInt16((short)configType);
            return buf;
        }

        /// <summary>
        /// PING_PONG packet (type 10) echo. Server sends ping with a challenge ID; tracker
        /// echoes same ID back verbatim. Fixes latency display in SlimeVR dashboard.
        /// Layout: header(4) + packetId(8) + pingId(4).
        /// </summary>
        public byte[] BuildPingPongPacket(int pingId)
        {
            var buf = new byte[4 + 8 + 4];
            var w = new BigEndianWriter(buf);
            w.SetPosition(0);
            w.WriteInt32((int)UDPPackets.PING_PONG);
            w.WriteInt64(NextPacketId());
            w.WriteInt32(pingId);
            return buf;
        }

        /// <summary>
        /// BUNDLE packet (type 100). Layout per SlimeVR-Server UDPProtocolParser.kt:
        ///   outer: [int32 BE type=100][int64 BE packetNumber]
        ///   repeated inner: [uint16 BE length][int32 BE innerType][innerPayload]
        /// The outer has a packet number (consumed by parse() before the BUNDLE branch), but
        /// each inner packet is header(type)+payload ONLY — no inner packet number. Our
        /// standalone Build*Packet helpers include an 8-byte packet id right after the type,
        /// which is correct for single-datagram sends but must be stripped when embedded in
        /// a bundle. Strip bytes [4..12] of each inner before copying.
        /// </summary>
        public byte[] BuildBundlePacket(params ReadOnlyMemory<byte>[] innerPackets)
        {
            int total = 4 + 8; // bundle type + outer packet number
            foreach (var p in innerPackets) total += 2 + (p.Length - 8); // length + (packet minus inner packet-id)

            var buf = new byte[total];
            var w = new BigEndianWriter(buf);
            w.SetPosition(0);
            w.WriteInt32((int)UDPPackets.BUNDLE);
            w.WriteInt64(NextPacketId());
            foreach (var p in innerPackets)
            {
                int innerLen = p.Length - 8; // drop the 8-byte packet id from the inner
                w.WriteInt16((short)innerLen);
                // type (first 4 bytes)
                p.Span.Slice(0, 4).CopyTo(buf.AsSpan(w.Position));
                w.Skip(4);
                // payload (everything after the 12-byte type+id prefix)
                p.Span.Slice(12).CopyTo(buf.AsSpan(w.Position));
                w.Skip(p.Length - 12);
            }
            return buf;
        }

        // Q15 quat scale: 1.0 unit  ↔ 0x7FFF; -1.0 ↔ 0x8000. Quaternions are unit so each
        // component is bounded in [-1, 1] — perfect fit.
        private const float Q15Scale = 32767f;
        // Q7 accel scale: 1 unit ↔ 128 ticks. ±256 m/s² range covers any human movement.
        private const float Q7Scale = 128f;

        /// <summary>
        /// Builds the 15-byte inner payload for pkt 23 (ROTATION_AND_ACCELERATION compact).
        /// Layout (all big-endian): sensorId(1) + qX(2) + qY(2) + qZ(2) + qW(2) + aX(2) +
        /// aY(2) + aZ(2). Quaternion components are encoded Q15, acceleration is Q7. Caller
        /// composes one or more of these into a BUNDLE_COMPACT envelope.
        /// </summary>
        public ReadOnlyMemory<byte> BuildRotationAccelCompactInner(Quaternion rotation, Vector3 acceleration, byte trackerId)
        {
            var buf = new byte[15];
            var w = new BigEndianWriter(buf);
            w.WriteByte(trackerId);
            w.WriteInt16(SaturatingShort(rotation.X * Q15Scale));
            w.WriteInt16(SaturatingShort(rotation.Y * Q15Scale));
            w.WriteInt16(SaturatingShort(rotation.Z * Q15Scale));
            w.WriteInt16(SaturatingShort(rotation.W * Q15Scale));
            w.WriteInt16(SaturatingShort(acceleration.X * Q7Scale));
            w.WriteInt16(SaturatingShort(acceleration.Y * Q7Scale));
            w.WriteInt16(SaturatingShort(acceleration.Z * Q7Scale));
            return buf.AsMemory(0, w.Position);
        }

        private static short SaturatingShort(float v)
        {
            if (v >= short.MaxValue) return short.MaxValue;
            if (v <= short.MinValue) return short.MinValue;
            return (short)v;
        }

        /// <summary>
        /// Builds a BUNDLE_COMPACT envelope (pkt 101) containing one or more compact frames.
        /// Each inner is wrapped <c>length(1B) + packetId(1B) + payload</c>; the outer has
        /// the standard <c>type(4) + packetId(8)</c> header. Length byte covers the inner
        /// packetId + payload (i.e. payload.Length + 1).
        /// </summary>
        public byte[] BuildBundleCompactPacket(params (byte InnerPacketId, ReadOnlyMemory<byte> Payload)[] inners)
        {
            int total = 4 + 8;
            foreach (var p in inners) total += 1 + 1 + p.Payload.Length;

            var buf = new byte[total];
            var w = new BigEndianWriter(buf);
            w.WriteInt32((int)UDPPackets.BUNDLE_COMPACT);
            w.WriteInt64(NextPacketId());
            foreach (var p in inners)
            {
                int innerLen = 1 + p.Payload.Length;
                if (innerLen > 255) throw new InvalidOperationException("BUNDLE_COMPACT inner exceeds 255 bytes (length is 1B).");
                w.WriteByte((byte)innerLen);
                w.WriteByte(p.InnerPacketId);
                p.Payload.Span.CopyTo(buf.AsSpan(w.Position));
                w.Skip(p.Payload.Length);
            }
            return buf;
        }
    }
}
