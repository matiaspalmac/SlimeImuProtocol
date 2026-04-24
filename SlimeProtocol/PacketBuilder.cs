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

        public byte[] BuildSensorInfoPacket(ImuType imuType, TrackerPosition pos, TrackerDataType dataType, byte trackerId)
        {
            var span = new byte[4 + 8 + 1 + 1 + 1 + 2 + 1 + 1];
            var w = new BigEndianWriter(span);
            w.SetPosition(0);
            w.WriteInt32((int)UDPPackets.SENSOR_INFO); // Packet header
            w.WriteInt64(NextPacketId()); // Packet counter
            w.WriteByte(trackerId); // Tracker Id
            w.WriteByte(0); // Sensor status
            w.WriteByte((byte)imuType);  // IMU type
            w.WriteInt16(1); // Calibration state
            w.WriteByte((byte)pos);  // Tracker Position
            w.WriteByte((byte)dataType);  // Tracker Data Type
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
        /// FEATURE_FLAGS packet (type 22). Advertises tracker-side capabilities to the server.
        /// Layout: header(4) + packetId(8) + flagBytes(variable, LSB0 bit order).
        /// Bit 0 = PROTOCOL_BUNDLE_SUPPORT. Other bits reserved.
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
        /// BUNDLE packet (type 100). Wraps N inner packets in one datagram, each prefixed with
        /// uint16 length. Requires server to advertise PROTOCOL_BUNDLE_SUPPORT via FEATURE_FLAGS.
        /// Saves syscall + UDP header overhead at high send rates (e.g. rotation+accel+battery).
        /// </summary>
        public byte[] BuildBundlePacket(params ReadOnlyMemory<byte>[] innerPackets)
        {
            int total = 4 + 8; // bundle header + packet id
            foreach (var p in innerPackets) total += 2 + p.Length; // uint16 length + payload

            var buf = new byte[total];
            var w = new BigEndianWriter(buf);
            w.SetPosition(0);
            w.WriteInt32((int)UDPPackets.BUNDLE);
            w.WriteInt64(NextPacketId());
            foreach (var p in innerPackets)
            {
                w.WriteInt16((short)p.Length);
                p.Span.CopyTo(buf.AsSpan(w.Position));
                w.Skip(p.Length);
            }
            return buf;
        }
    }
}
