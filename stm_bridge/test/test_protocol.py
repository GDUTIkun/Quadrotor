import struct

from stm_bridge.protocol import (
    MSG_CMD_VEL,
    MSG_IMU,
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
