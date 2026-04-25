using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SlimeImuProtocol.SlimeVR
{
    public static class FirmwareConstants
    {

        public enum BoardType
        {
            UNKNOWN = 0,
            SLIMEVR_LEGACY = 1,
            SLIMEVR_DEV = 2,
            NODEMCU = 3,
            CUSTOM = 4,
            WROOM32 = 5,
            WEMOSD1MINI = 6,
            TTGO_TBASE = 7,
            ESP01 = 8,
            SLIMEVR = 9,
            LOLIN_C3_MINI = 10,
            BEETLE32C32 = 11,
            ES32C3DEVKITM1 = 12,
            OWOTRACK = 13,
            WRANGLER = 14,
            MOCOPI = 15,
            WEMOSWROOM02 = 16,
            XIAO_ESP32C3 = 17,
            HARITORA = 18,
            DEV_RESERVED = 250
        }

        public enum ImuType
        {
            UNKNOWN = 0,
            MPU9250 = 1,
            MPU6500 = 2,
            BNO080 = 3,
            BNO085 = 4,
            BNO055 = 5,
            MPU6050 = 6,
            BNO086 = 7,
            BMI160 = 8,
            ICM20948 = 9,
            ICM42688 = 10,
            BMI270 = 11,
            LSM6DS3TRC = 12,
            LSM6DSV = 13,
            LSM6DSO = 14,
            LSM6DSR = 15,
            DEV_RESERVED = 250
        }
        public enum McuType
        {
            UNKNOWN = 0,
            ESP8266 = 1,
            ESP32 = 2,
            OWOTRACK_ANDROID = 3,
            WRANGLER = 4,
            OWOTRACK_IOS = 5,
            ESP32_C3 = 6,
            MOCOPI = 7,
            HARITORA = 8,
            DEV_RESERVED = 250
        }
        public enum MagnetometerStatus
        {
            NOT_SUPPORTED,
            DISABLED,
            ENABLED
        }

        public enum TrackerDataType
        {
            ROTATION = 0,
            FLEX_RESISTANCE = 1,
            FLEX_ANGLE = 2
        }

        public enum UserActionType : byte
        {
            RESET_FULL = 2,
            RESET_YAW = 3,
            RESET_MOUNTING = 4,
            PAUSE_TRACKING = 5,
        }

        public enum ControllerButton : byte
        {
            BUTTON_1_HELD = 1,
            BUTTON_1_UNHELD = 2,
            BUTTON_2_HELD = 3,
            BUTTON_2_UNHELD = 4,
            MENU_RECENTER_HELD = 5,
            MENU_RECENTER_UNHELD = 6,
            STICK_CLICK_HELD = 7,
            STICK_CLICK_UNHELD = 8,
            TRACKPAD_CLICK_HELD = 9,
		    TRACKPAD_CLICK_UNHELD = 10
        }

        public enum TrackerPosition
        {
            NONE = 0,
            HEAD = 1,
            NECK = 2,
            UPPER_CHEST = 3,
            CHEST = 4,
            WAIST = 5,
            HIP = 6,
            LEFT_UPPER_LEG = 7,
            RIGHT_UPPER_LEG = 8,
            LEFT_LOWER_LEG = 9,
            RIGHT_LOWER_LEG = 10,
            LEFT_FOOT = 11,
            RIGHT_FOOT = 12,
            LEFT_LOWER_ARM = 13,
            RIGHT_LOWER_ARM = 14,
            [Obsolete("Typo — use LEFT_LOWER_ARM")] LEFT_LOWER_AR = 13,
            [Obsolete("Typo — use RIGHT_LOWER_ARM")] RIGHT_LOWER_AR = 14,
            LEFT_UPPER_ARM = 15,
            RIGHT_UPPER_ARM = 16,
            LEFT_HAND = 17,
            RIGHT_HAND = 18,
            LEFT_SHOULDER = 19,
            RIGHT_SHOULDER = 20,
            LEFT_THUMB_METACARPAL = 21,
            LEFT_THUMB_PROXIMAL = 22,
            LEFT_THUMB_DISTAL = 23,
            LEFT_INDEX_PROXIMAL = 24,
            LEFT_INDEX_INTERMEDIATE = 25,
            LEFT_INDEX_DISTAL = 26,
            LEFT_MIDDLE_PROXIMAL = 27,
            LEFT_MIDDLE_INTERMEDIATE = 28,
            LEFT_MIDDLE_DISTAL = 29,
            LEFT_RING_PROXIMAL = 30,
            LEFT_RING_INTERMEDIATE = 31,
            LEFT_RING_DISTAL = 32,
            LEFT_LITTLE_PROXIMAL = 33,
            LEFT_LITTLE_INTERMEDIATE = 34,
            LEFT_LITTLE_DISTAL = 35,
            RIGHT_THUMB_METACARPAL = 36,
            RIGHT_THUMB_PROXIMAL = 37,
            RIGHT_THUMB_DISTAL = 38,
            RIGHT_INDEX_PROXIMAL = 39,
            RIGHT_INDEX_INTERMEDIATE = 40,
            RIGHT_INDEX_DISTAL = 41,
            RIGHT_MIDDLE_PROXIMAL = 42,
            RIGHT_MIDDLE_INTERMEDIATE = 43,
            RIGHT_MIDDLE_DISTAL = 44,
            RIGHT_RING_PROXIMAL = 45,
            RIGHT_RING_INTERMEDIATE = 46,
            RIGHT_RING_DISTAL = 47,
            RIGHT_LITTLE_PROXIMAL = 48,
            RIGHT_LITTLE_INTERMEDIATE = 49,
            RIGHT_LITTLE_DISTAL = 50
        }
        /// <summary>
        /// Packet IDs sent tracker → SlimeVR server (outgoing).
        /// </summary>
        public static class UDPPackets
        {
            public const int HEARTBEAT = 0;
            public const int ROTATION = 1;
            public const int GYRO = 2;
            public const int HANDSHAKE = 3;
            public const int ACCELERATION = 4;
            public const int MAG = 5;
            public const int RAW_CALIBRATION_DATA = 6;
            public const int CALIBRATION_FINISHED = 7;
            public const int CONFIG = 8;
            public const int RAW_MAGNETOMETER = 9;
            public const int PING_PONG = 10;
            public const int SERIAL = 11;
            public const int BATTERY_LEVEL = 12;
            public const int TAP = 13;
            public const int RESET_REASON = 14;
            public const int SENSOR_INFO = 15;
            public const int ROTATION_2 = 16;
            public const int ROTATION_DATA = 17;
            public const int MAGNETOMETER_ACCURACY = 18;
            public const int CALIBRATION_ACTION = 21;
            public const int FLEX_DATA_PACKET = 26;
            public const int HAPTICS = 30;
            public const int BUTTON_PUSHED = 60;
            public const int SEND_MAG_STATUS = 61;
            public const int CHANGE_MAG_STATUS = 62;
            public const int FEATURE_FLAGS = 22;
            public const int ROTATION_AND_ACCELERATION_COMPACT = 23;
            public const int ACK_CONFIG_CHANGE = 24;
            public const int SET_CONFIG_FLAG = 25;
            public const int CONTROLLER_BUTTON = 66;
            public const int THUMBSTICK = 67;
            public const int TRIGGER = 68;
            public const int GRIP = 69;
            public const int BUNDLE = 100;
            public const int BUNDLE_COMPACT = 101;

            /// <summary>
            /// Server→tracker capability bits, advertised when the server sends pkt 22 to us.
            /// Read these from the inbound packet to gate which compact / bundle features we
            /// can use back. Indices match SlimeVR-Server's <c>ServerFeatureFlags</c> enum.
            /// </summary>
            public static class ServerFeatureFlagBits
            {
                public const int PROTOCOL_BUNDLE_SUPPORT = 0;
                public const int PROTOCOL_BUNDLE_COMPACT_SUPPORT = 1;
            }

            /// <summary>
            /// Tracker→server capability bits, advertised when WE send pkt 22 to the server.
            /// Different namespace from <see cref="ServerFeatureFlagBits"/>: pkt 22 is
            /// bidirectional, but each side packs its own enum into the flag bytes.
            /// Indices match SlimeVR-Server's <c>FirmwareFeatures.FirmwareFeatureFlags</c>.
            /// </summary>
            public static class FirmwareFeatureFlagBits
            {
                public const int REMOTE_COMMAND = 0;
                public const int B64_WIFI_SCANNING = 1;
                public const int SENSOR_CONFIG = 2;
            }

            /// <summary>
            /// Deprecated: pre-existing alias that incorrectly mixed server / firmware
            /// namespaces. Bit 0 here is <c>REMOTE_COMMAND</c> on the firmware side, NOT
            /// <c>PROTOCOL_BUNDLE_SUPPORT</c> as the name suggested. Kept for source
            /// compat; the only previous caller has been migrated to the correct flag set.
            /// </summary>
            [Obsolete("Use ServerFeatureFlagBits or FirmwareFeatureFlagBits explicitly. The constant here was misnamed — bit 0 is REMOTE_COMMAND in firmware namespace.")]
            public static class FeatureFlagBits
            {
                public const int PROTOCOL_BUNDLE_SUPPORT = 0;
            }

            // Deprecated typo aliases — will be removed in a future version.
            [Obsolete("Use RAW_MAGNETOMETER")] public const int RAW_MAGENTOMETER = RAW_MAGNETOMETER;
            [Obsolete("Use MAGNETOMETER_ACCURACY")] public const int MAGENTOMETER_ACCURACY = MAGNETOMETER_ACCURACY;
        }

        /// <summary>
        /// Packet IDs received from SlimeVR server (incoming). Values can collide with
        /// outgoing packet IDs — the direction disambiguates them.
        /// </summary>
        public static class UDPPacketsIn
        {
            public const int RECEIVE_HEARTBEAT = 1;
            public const int RECEIVE_VIBRATE = 2;
            public const int RECEIVE_HANDSHAKE = 3;
            public const int RECEIVE_COMMAND = 4;
        }

    }
}
