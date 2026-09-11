#!/usr/bin/env python3
"""Raw probe: which ttyACM port talks to the Unitree servo bus?

For each port:
  1. open at 6 Mbps 8N1
  2. listen 0.5 s for spontaneous bytes (a debug console would emit ASCII)
  3. send a 20-byte "stop" control packet (mode=0, all zeros) to id 0
  4. listen for a 26-byte 0xFC 0xEE feedback frame
"""
import sys, time, struct

import serial

from unitree_servo import build_control_packet, parse_feedback_packet, RATIO

PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]
BAUD = 6_000_000


def probe(port):
    print(f"\n=== {port} ===")
    try:
        ser = serial.Serial(port, BAUD, timeout=0.2)
    except Exception as e:
        print(f"  open failed: {e}")
        return
    try:
        ser.reset_input_buffer()

        # 1. spontaneous data?
        idle = ser.read(256)
        print(f"  idle read 0.2s -> {len(idle)} bytes: {idle[:64]!r}")

        # 2. send stop command (mode=0 lock/stop, everything zero)
        cmd = build_control_packet(0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0)
        ser.reset_input_buffer()
        ser.write(cmd)
        print(f"  TX stop pkt: {cmd.hex(' ')}")

        # 3. read up to 26 bytes
        buf = b""
        t0 = time.time()
        while len(buf) < 26 and time.time() - t0 < 0.5:
            chunk = ser.read(26 - len(buf))
            buf += chunk
        print(f"  RX {len(buf)} bytes: {buf.hex(' ')}")

        if len(buf) == 26:
            p = parse_feedback_packet(buf)
            if p:
                print("  PARSED OK ->", {k: (round(v, 3) if isinstance(v, float) else v)
                                         for k, v in p.items()})
            else:
                print("  parse failed (bad head or CRC)")
        else:
            print("  no full feedback frame")
    finally:
        ser.close()


if __name__ == "__main__":
    for port in (sys.argv[1:] or PORTS):
        probe(port)
