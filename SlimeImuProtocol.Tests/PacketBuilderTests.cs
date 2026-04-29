using System.Buffers.Binary;
using System.Numerics;
using SlimeImuProtocol.SlimeVR;
using Xunit;

namespace SlimeImuProtocol.Tests;

/// <summary>
/// Round-trip tests for the SlimeVR UDP wire format. The server's parser is the source of
/// truth; these tests pin the byte layout that the parser expects so a refactor of
/// PacketBuilder can't silently change what we put on the wire.
/// </summary>
public class PacketBuilderTests
{
    private const int PACKET_ROTATION_DATA = 17;
    private const int PACKET_ACCELERATION = 4;
    private const int PACKET_GYRO = 2;

    [Fact]
    public void BuildRotationPacket_writes_quaternion_in_XYZW_order()
    {
        var pb = new PacketBuilder("test");
        // Use values whose IEEE-754 representation has no rounding so the round-trip is exact.
        var q = new Quaternion(0.25f, -0.5f, 0.75f, -0.125f);
        const byte trackerId = 7;

        var bytes = pb.BuildRotationPacket(q, trackerId).ToArray();

        Assert.Equal(31, bytes.Length); // 4 + 8 + 1 + 1 + 16 + 1
        Assert.Equal(PACKET_ROTATION_DATA, BinaryPrimitives.ReadInt32BigEndian(bytes.AsSpan(0, 4)));
        // Skip 8-byte packet id.
        Assert.Equal(trackerId, bytes[12]);
        Assert.Equal(1, bytes[13]); // Data type
        Assert.Equal(q.X, ReadFloat(bytes, 14));
        Assert.Equal(q.Y, ReadFloat(bytes, 18));
        Assert.Equal(q.Z, ReadFloat(bytes, 22));
        Assert.Equal(q.W, ReadFloat(bytes, 26));
        Assert.Equal(0, bytes[30]); // Calibration info
    }

    [Fact]
    public void BuildAccelerationPacket_writes_vector_in_XYZ_order_with_trailing_trackerId()
    {
        var pb = new PacketBuilder("test");
        var a = new Vector3(1.5f, -9.81f, 0.125f);
        const byte trackerId = 3;

        var bytes = pb.BuildAccelerationPacket(a, trackerId).ToArray();

        Assert.Equal(25, bytes.Length); // 4 + 8 + 12 + 1
        Assert.Equal(PACKET_ACCELERATION, BinaryPrimitives.ReadInt32BigEndian(bytes.AsSpan(0, 4)));
        Assert.Equal(a.X, ReadFloat(bytes, 12));
        Assert.Equal(a.Y, ReadFloat(bytes, 16));
        Assert.Equal(a.Z, ReadFloat(bytes, 20));
        Assert.Equal(trackerId, bytes[24]);
    }

    [Fact]
    public void BuildGyroPacket_layout_matches_rotation_pattern_minus_W()
    {
        var pb = new PacketBuilder("test");
        var g = new Vector3(0.1f, -0.2f, 0.3f);
        const byte trackerId = 1;

        var bytes = pb.BuildGyroPacket(g, trackerId).ToArray();

        Assert.Equal(27, bytes.Length); // 4 + 8 + 1 + 1 + 12 + 1
        Assert.Equal(PACKET_GYRO, BinaryPrimitives.ReadInt32BigEndian(bytes.AsSpan(0, 4)));
        Assert.Equal(trackerId, bytes[12]);
        Assert.Equal(1, bytes[13]); // Data type
        Assert.Equal(g.X, ReadFloat(bytes, 14));
        Assert.Equal(g.Y, ReadFloat(bytes, 18));
        Assert.Equal(g.Z, ReadFloat(bytes, 22));
        Assert.Equal(0, bytes[26]);
    }

    [Fact]
    public void NextPacketId_is_monotonic_across_calls()
    {
        var pb = new PacketBuilder("test");
        long Id(byte[] b) => BinaryPrimitives.ReadInt64BigEndian(b.AsSpan(4, 8));

        var first = Id(pb.BuildAccelerationPacket(Vector3.Zero, 0).ToArray());
        var second = Id(pb.BuildAccelerationPacket(Vector3.Zero, 0).ToArray());
        var third = Id(pb.BuildAccelerationPacket(Vector3.Zero, 0).ToArray());

        Assert.True(second > first, $"expected monotonic, got {first}, {second}");
        Assert.True(third > second, $"expected monotonic, got {second}, {third}");
    }

    private static float ReadFloat(byte[] buf, int offset)
    {
        Span<byte> reversed = stackalloc byte[4];
        buf.AsSpan(offset, 4).CopyTo(reversed);
        if (BitConverter.IsLittleEndian) reversed.Reverse();
        return BitConverter.ToSingle(reversed);
    }
}
