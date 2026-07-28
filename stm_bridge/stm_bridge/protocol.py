"""Binary protocol helpers for the ROS-to-STM serial link."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable, List


HEADER = b'\xAA\x55'
CRC_INIT = 0x0000
CRC_POLY_REVERSED = 0xA001

MSG_CMD_VEL = 0x01
MSG_IMU = 0x81
MSG_WHEEL_ODOM = 0x82
MSG_STATUS = 0x83

MAX_PAYLOAD_LEN = 64


@dataclass(frozen=True)
class Frame:
    msg_id: int
    seq: int
    payload: bytes


@dataclass(frozen=True)
class ImuPayload:
    ax_mg: int
    ay_mg: int
    az_mg: int
    gx_mdps: int
    gy_mdps: int
    gz_mdps: int
    yaw_cdeg: int
    pitch_cdeg: int
    roll_cdeg: int
    stamp_ms: int


@dataclass(frozen=True)
class WheelOdomPayload:
    x_mm: int
    y_mm: int
    yaw_mrad: int
    vx_mm_s: int
    wz_mrad_s: int
    left_mm_s: int
    right_mm_s: int
    stamp_ms: int


@dataclass(frozen=True)
class StatusPayload:
    voltage_mv: int
    current_ma: int
    state: int
    error_flags: int
    stamp_ms: int


def crc16_ibm(data: bytes) -> int:
    """CRC-16/IBM, also known as CRC-16/ARC: poly 0x8005, init 0x0000."""
    crc = CRC_INIT
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ CRC_POLY_REVERSED
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def build_frame(msg_id: int, seq: int, payload: bytes) -> bytes:
    if not 0 <= msg_id <= 0xFF:
        raise ValueError('msg_id must fit in uint8')
    if not 0 <= seq <= 0xFF:
        raise ValueError('seq must fit in uint8')
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError(f'payload too large: {len(payload)} > {MAX_PAYLOAD_LEN}')

    body = bytes([msg_id, seq, len(payload)]) + payload
    return HEADER + body + struct.pack('<H', crc16_ibm(body))


def clamp_int16(value: int) -> int:
    return max(-32768, min(32767, value))


def encode_cmd_vel(v_m_s: float, w_rad_s: float, enable: bool) -> bytes:
    v_mm_s = clamp_int16(round(v_m_s * 1000.0))
    w_mrad_s = clamp_int16(round(w_rad_s * 1000.0))
    return struct.pack('<hhB', v_mm_s, w_mrad_s, 1 if enable else 0)


def decode_imu(payload: bytes) -> ImuPayload:
    expected = struct.calcsize('<hhhhhhhhhI')
    if len(payload) != expected:
        raise ValueError(f'bad IMU payload length: {len(payload)} != {expected}')
    return ImuPayload(*struct.unpack('<hhhhhhhhhI', payload))


def decode_wheel_odom(payload: bytes) -> WheelOdomPayload:
    expected = struct.calcsize('<iiihhhhI')
    if len(payload) != expected:
        raise ValueError(f'bad wheel odom payload length: {len(payload)} != {expected}')
    return WheelOdomPayload(*struct.unpack('<iiihhhhI', payload))


def decode_status(payload: bytes) -> StatusPayload:
    expected = struct.calcsize('<HhBHI')
    if len(payload) != expected:
        raise ValueError(f'bad status payload length: {len(payload)} != {expected}')
    return StatusPayload(*struct.unpack('<HhBHI', payload))


class FrameParser:
    """Incremental parser that tolerates noise, half packets, and sticky packets."""

    def __init__(self, max_payload_len: int = MAX_PAYLOAD_LEN) -> None:
        self._buffer = bytearray()
        self.max_payload_len = max_payload_len
        self.crc_errors = 0
        self.dropped_bytes = 0
        self.frames_received = 0

    def feed(self, data: bytes | bytearray | Iterable[int]) -> List[Frame]:
        self._buffer.extend(data)
        frames: List[Frame] = []

        while True:
            header_index = self._buffer.find(HEADER)
            if header_index < 0:
                self.dropped_bytes += len(self._buffer)
                self._buffer.clear()
                break
            if header_index > 0:
                self.dropped_bytes += header_index
                del self._buffer[:header_index]

            if len(self._buffer) < 5:
                break

            payload_len = self._buffer[4]
            if payload_len > self.max_payload_len:
                self.dropped_bytes += 1
                del self._buffer[0]
                continue

            frame_len = 2 + 3 + payload_len + 2
            if len(self._buffer) < frame_len:
                break

            raw = bytes(self._buffer[:frame_len])
            body = raw[2:-2]
            expected_crc = struct.unpack('<H', raw[-2:])[0]
            actual_crc = crc16_ibm(body)
            if actual_crc != expected_crc:
                self.crc_errors += 1
                del self._buffer[0]
                continue

            frames.append(Frame(msg_id=raw[2], seq=raw[3], payload=raw[5:-2]))
            self.frames_received += 1
            del self._buffer[:frame_len]

        return frames
