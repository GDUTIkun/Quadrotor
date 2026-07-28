import struct

from stm_bridge.protocol import (
    MSG_CMD_VEL,
    MSG_IMU,
    MSG_STATUS,
    MSG_WHEEL_ODOM,
    FrameParser,
    build_frame,
    crc16_ibm,
    decode_imu,
    encode_cmd_vel,
)


def test_crc16_ibm_known_vector():
    assert crc16_ibm(b'123456789') == 0xBB3D


def test_cmd_vel_payload_units_and_clamping():
    payload = encode_cmd_vel(1.234, -0.567, True)
    assert struct.unpack('<hhB', payload) == (1234, -567, 1)

    payload = encode_cmd_vel(100.0, -100.0, False)
    assert struct.unpack('<hhB', payload) == (32767, -32768, 0)


def test_protocol_documented_example_frames():
    stop = build_frame(MSG_CMD_VEL, 0, struct.pack('<hhB', 0, 0, 0))
    forward = build_frame(MSG_CMD_VEL, 1, struct.pack('<hhB', 200, 0, 1))
    imu_flat = build_frame(
        MSG_IMU,
        0,
        struct.pack('<hhhhhhhhhI', 0, 0, 1000, 0, 0, 0, 0, 0, 0, 1000),
    )
    odom_zero = build_frame(
        MSG_WHEEL_ODOM,
        0,
        struct.pack('<iiihhhhI', 0, 0, 0, 0, 0, 0, 0, 1000),
    )
    status_ready = build_frame(
        MSG_STATUS,
        0,
        struct.pack('<HhBHI', 12000, 0, 1, 0, 1000),
    )

    assert stop.hex(' ').upper() == 'AA 55 01 00 05 00 00 00 00 00 C1 99'
    assert forward.hex(' ').upper() == 'AA 55 01 01 05 C8 00 00 00 01 F1 49'
    assert imu_flat.hex(' ').upper() == (
        'AA 55 81 00 16 00 00 00 00 E8 03 00 00 00 00 00 00 00 00 00 00 '
        '00 00 E8 03 00 00 1D 51'
    )
    assert odom_zero.hex(' ').upper() == (
        'AA 55 82 00 18 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 '
        '00 00 00 00 E8 03 00 00 38 45'
    )
    assert status_ready.hex(' ').upper() == (
        'AA 55 83 00 0B E0 2E 00 00 01 00 00 E8 03 00 00 85 A4'
    )


def test_parser_handles_noise_half_packets_and_sticky_packets():
    first = build_frame(MSG_CMD_VEL, 7, b'abc')
    second = build_frame(MSG_IMU, 8, struct.pack('<hhhhhhhhhI', 1, 2, 3, 4, 5, 6, 7, 8, 9, 10))
    parser = FrameParser()

    assert parser.feed(b'\x00bad' + first[:4]) == []
    frames = parser.feed(first[4:] + second)

    assert len(frames) == 2
    assert frames[0].msg_id == MSG_CMD_VEL
    assert frames[0].seq == 7
    assert frames[0].payload == b'abc'
    assert decode_imu(frames[1].payload).stamp_ms == 10
    assert parser.dropped_bytes == 4


def test_parser_drops_crc_errors_and_resyncs():
    bad = bytearray(build_frame(MSG_CMD_VEL, 1, b'bad'))
    bad[-1] ^= 0xFF
    good = build_frame(MSG_CMD_VEL, 2, b'ok')
    parser = FrameParser()

    frames = parser.feed(bad + good)

    assert len(frames) == 1
    assert frames[0].seq == 2
    assert parser.crc_errors == 1
