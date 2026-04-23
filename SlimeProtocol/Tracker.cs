using SlimeImuProtocol.SlimeVR;
using SlimeImuProtocol.Utility;
using System.Numerics;
using System.Text;
using static SlimeImuProtocol.SlimeVR.FirmwareConstants;

namespace SlimeImuProtocol.SlimeProtocol {
    public class Tracker : IDisposable {
        public int TrackerNum { get; set; }
        public string Name { get; set; }
        public string DisplayName { get; set; }
        public bool HasRotation { get; set; }
        public bool HasAcceleration { get; set; }
        public bool UserEditable { get; set; }
        public ImuType ImuType { get; set; }
        public bool AllowFiltering { get; set; }
        public bool NeedsReset { get; set; }
        public bool NeedsMounting { get; set; }
        public bool UsesTimeout { get; set; }
        public MagnetometerStatus MagStatus { get; set; }

        public float BatteryLevel {
            get => _batteryLevel;
            set
            {
                // Debounce: only send when level changes more than 1% (0.01 absolute) or
                // voltage changes more than 0.05V. Avoids packet spam from callers that
                // update battery every frame.
                if (MathF.Abs(value - _batteryLevel) < 0.01f) return;
                _batteryLevel = value;
                if (_ready)
                {
                    _ = _udpHandler.SetSensorBattery(_batteryLevel, BatteryVoltage);
                }
            }
        }
        public float BatteryVoltage { get; set; } = 3.7f;
        public float? Temperature { get; set; }
        public int SignalStrength { get; set; }

        public TrackerStatus Status = TrackerStatus.Disconnected;
        private UDPHandler _udpHandler;
        private bool _ready;
        private float _batteryLevel;
        private Quaternion _currentRotation;
        private Vector3 _currentAcceleration;
        private CancellationTokenSource _cts = new CancellationTokenSource();

        // These can be set after construction when device data is parsed
        public string FirmwareVersion { get; set; }
        public string HardwareIdentifier { get; set; }
        public BoardType BoardType { get; set; }
        public McuType McuType { get; set; }
        public Quaternion CurrentRotation { get => _currentRotation; set => _currentRotation = value; }
        public Vector3 CurrentAcceleration { get => _currentAcceleration; set => _currentAcceleration = value; }

        public Tracker(TrackerDevice device, int trackerNum, string name, string displayName, bool hasRotation, bool hasAcceleration,
            bool userEditable, ImuType imuType, bool allowFiltering, bool needsReset,
            bool needsMounting, bool usesTimeout, MagnetometerStatus magStatus) {
            TrackerNum = trackerNum;
            Name = name;
            DisplayName = displayName;
            HasRotation = hasRotation;
            HasAcceleration = hasAcceleration;
            UserEditable = userEditable;
            ImuType = imuType;
            AllowFiltering = allowFiltering;
            NeedsReset = needsReset;
            NeedsMounting = needsMounting;
            UsesTimeout = usesTimeout;
            MagStatus = magStatus;

            var token = _cts.Token;
            // Await firmware availability with a 60s ceiling. Still polls since the source
            // (external TrackerDevice) doesn't expose a FirmwareReady event — but uses
            // Task.Delay (non-blocking) and respects cancellation so Dispose can unstick it.
            Task.Run(async () => {
                const int maxWaitMs = 60000;
                const int pollMs = 250;
                int elapsed = 0;
                while (device.FirmwareVersion == null) {
                    if (token.IsCancellationRequested) return;
                    try { await Task.Delay(pollMs, token); } catch (OperationCanceledException) { return; }
                    elapsed += pollMs;
                    if (elapsed >= maxWaitMs) return;
                }
                if (token.IsCancellationRequested) return;
                _udpHandler = new UDPHandler(device.FirmwareVersion + "_EsbToLan",
                 Encoding.UTF8.GetBytes(device.HardwareIdentifier), device.BoardType,
                 ImuType, device.McuType, MagStatus, 1);
                _ready = true;
            }, token);
        }

        public void TryInitialize() {
            if (FirmwareVersion != null && HardwareIdentifier != null) {
                _udpHandler = new UDPHandler(
                    FirmwareVersion + "_EsbToLan",
                    Encoding.UTF8.GetBytes(HardwareIdentifier),
                    BoardType,
                    ImuType,
                    McuType,
                    MagStatus,
                    1
                );
                _ready = true;
            }
        }

        public void SetRotation(Quaternion q) {
            if (_ready)
            {
                _currentRotation = q;
                _udpHandler?.SetSensorRotation(q, 0);
            }
        }

        public void SetAcceleration(Vector3 a) {
            if (_ready)
            {
                _currentAcceleration = a;
                _udpHandler?.SetSensorAcceleration(a, 0);
            }
        }

        public void SetMagVector(Vector3 m) {
            if (_ready)
            {
                _udpHandler?.SetSensorMagnetometer(m, 0);
            }
        }

        public void Dispose() {
             _ready = false;
             try { _cts.Cancel(); } catch { }
             _udpHandler?.Dispose();
             _cts.Dispose();
        }
    }

    public enum TrackerStatus {
        OK,
        Disconnected
    }
}
