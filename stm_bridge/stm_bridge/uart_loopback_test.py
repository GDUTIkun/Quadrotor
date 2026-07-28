#!/usr/bin/env python3
"""UART loopback test for the STM bridge serial port.

Connect TX and RX together before running this script. For UART0_M2 on this
board, that means Pin 8 TX(GPIO4_A3) to Pin 10 RX(GPIO4_A4).
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import time

try:
    import serial
    from serial import SerialException
except ImportError:
    serial = None
    SerialException = Exception

from stm_bridge.protocol import FrameParser, MSG_CMD_VEL, build_frame, encode_cmd_vel


DEFAULT_PORT = '/dev/ttyS0'
DEFAULT_BAUDRATE = 576000


def short_hex(data: bytes, limit: int = 32) -> str:
    shown = data[:limit].hex(' ')
    if len(data) > limit:
        return f'{shown} ... ({len(data)} bytes)'
    return shown


def read_exact(ser: serial.Serial, size: int, timeout_s: float) -> bytes:
    deadline = time.monotonic() + timeout_s
    data = bytearray()
    while len(data) < size and time.monotonic() < deadline:
        chunk = ser.read(size - len(data))
        if chunk:
            data.extend(chunk)
            continue
        time.sleep(0.001)
    return bytes(data)


def check_echo(ser: serial.Serial, payload: bytes, timeout_s: float) -> tuple[bool, bytes]:
    ser.reset_input_buffer()
    ser.write(payload)
    ser.flush()
    received = read_exact(ser, len(payload), timeout_s)
    return received == payload, received


def make_raw_payload(seq: int, payload_size: int) -> bytes:
    prefix = b'UART0_M2_LOOP'
    stamp_ns = time.monotonic_ns() & 0xFFFFFFFF
    header = prefix + struct.pack('<HI', seq & 0xFFFF, stamp_ns)
    if payload_size <= len(header):
        return header[:payload_size]
    return header + os.urandom(payload_size - len(header))


def run_raw_tests(
    ser: serial.Serial,
    count: int,
    payload_size: int,
    timeout_s: float,
    interval_s: float,
) -> bool:
    print(f'Raw byte loopback: {count} packets, payload={payload_size} bytes')
    passed = 0
    for seq in range(count):
        payload = make_raw_payload(seq, payload_size)
        ok, received = check_echo(ser, payload, timeout_s)
        if not ok:
            print(f'[FAIL] raw seq={seq}: expected {len(payload)} bytes, got {len(received)} bytes')
            print(f'       tx: {short_hex(payload)}')
            print(f'       rx: {short_hex(received)}')
            return False
        passed += 1
        print(f'[ OK ] raw seq={seq} echoed {len(received)} bytes')
        time.sleep(interval_s)
    print(f'Raw byte loopback passed: {passed}/{count}')
    return True


def run_protocol_tests(
    ser: serial.Serial,
    count: int,
    timeout_s: float,
    interval_s: float,
) -> bool:
    print(f'Protocol frame loopback: {count} CMD_VEL frames')
    parser = FrameParser()
    passed = 0
    for seq in range(count):
        v = 0.10 + 0.01 * (seq % 5)
        w = -0.20 + 0.02 * (seq % 7)
        payload = encode_cmd_vel(v, w, True)
        packet = build_frame(MSG_CMD_VEL, seq & 0xFF, payload)
        ok, received = check_echo(ser, packet, timeout_s)
        if not ok:
            print(f'[FAIL] frame seq={seq}: expected {len(packet)} bytes, got {len(received)} bytes')
            print(f'       tx: {short_hex(packet)}')
            print(f'       rx: {short_hex(received)}')
            return False

        frames = parser.feed(received)
        if len(frames) != 1:
            print(f'[FAIL] frame seq={seq}: parser returned {len(frames)} frame(s)')
            print(f'       rx: {short_hex(received)}')
            return False
        frame = frames[0]
        if frame.msg_id != MSG_CMD_VEL or frame.seq != (seq & 0xFF) or frame.payload != payload:
            print(f'[FAIL] frame seq={seq}: parsed content mismatch')
            print(f'       msg_id=0x{frame.msg_id:02X}, parsed_seq={frame.seq}, payload={short_hex(frame.payload)}')
            return False

        passed += 1
        print(f'[ OK ] frame seq={seq} echoed and parsed')
        time.sleep(interval_s)
    print(f'Protocol frame loopback passed: {passed}/{count}')
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Self-send/self-receive UART loopback test for UART0_M2.',
    )
    parser.add_argument('--port', default=DEFAULT_PORT, help=f'serial device, default: {DEFAULT_PORT}')
    parser.add_argument(
        '--baudrate',
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f'baudrate, default: {DEFAULT_BAUDRATE}',
    )
    parser.add_argument('--count', type=int, default=20, help='packets per test stage')
    parser.add_argument('--payload-size', type=int, default=32, help='raw payload bytes per packet')
    parser.add_argument('--timeout', type=float, default=0.2, help='read/write timeout in seconds')
    parser.add_argument('--interval', type=float, default=0.02, help='delay between packets in seconds')
    parser.add_argument('--raw-only', action='store_true', help='only run raw byte loopback')
    parser.add_argument('--protocol-only', action='store_true', help='only run STM protocol frame loopback')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.raw_only and args.protocol_only:
        print('Choose at most one of --raw-only and --protocol-only.', file=sys.stderr)
        return 2
    if args.count <= 0:
        print('--count must be > 0', file=sys.stderr)
        return 2
    if args.payload_size <= 0:
        print('--payload-size must be > 0', file=sys.stderr)
        return 2

    if serial is None:
        print('pyserial is not installed. Install python3-serial first.', file=sys.stderr)
        return 1

    print(f'Opening {args.port} at {args.baudrate} 8N1, no flow control')
    try:
        with serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=args.timeout,
            write_timeout=args.timeout,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        ) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            ok = True
            if not args.protocol_only:
                ok = run_raw_tests(ser, args.count, args.payload_size, args.timeout, args.interval)
            if ok and not args.raw_only:
                ok = run_protocol_tests(ser, args.count, args.timeout, args.interval)
    except SerialException as exc:
        print(f'Cannot open/use serial port {args.port}: {exc}', file=sys.stderr)
        print('If this is a permission error, try: sudo usermod -aG dialout $USER, then re-login.', file=sys.stderr)
        return 1

    if ok:
        print(f'PASS: {args.port} TX/RX loopback is working at {args.baudrate}.')
        return 0

    print(f'FAIL: {args.port} did not echo correctly at {args.baudrate}.', file=sys.stderr)
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
