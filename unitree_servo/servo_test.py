#!/usr/bin/env python3
"""Bench test tool for Unitree J288/S288 digital servo behind the 1-Wire->USB board.

Usage:
    python3 servo_test.py status                  # one status frame, no motion
    python3 servo_test.py scan                    # which IDs answer on the bus
    python3 servo_test.py hold   --kp 20 --kd 1.0 # hold current position
    python3 servo_test.py step   --kp 20 --kd 1.0 --delta 0.2
    python3 servo_test.py sine   --kp 40 --kd 1.5 --amp 0.3 --freq 0.2 --secs 6
    python3 servo_test.py damp   --kd 1.0         # Kp=0: pure damping, no motion
    python3 servo_test.py stop                    # mode 0, lock

All positions/speeds/torques are OUTPUT-side (after the 288.35:1 gearbox),
which is what the control packet wants.
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

from unitree_servo import (
    RATIO, MotorProtocolSync, build_control_packet, parse_feedback_packet,
)

DEFAULT_PORT = "/dev/unitree_servo"
BAUD = 6_000_000


def set_latency_timer(port: str, value: int = 1) -> str:
    """CDC-ACM buffers reads for latency_timer ms (default 16). 16 ms caps the
    loop at ~60 Hz no matter how fast the wire is. 1 ms -> ~1 kHz."""
    name = os.path.basename(os.path.realpath(port))   # /dev/unitree_servo -> ttyACM0
    path = f"/sys/bus/usb-serial/devices/{name}/latency_timer"
    try:
        old = open(path).read().strip()
    except OSError:
        return None   # cdc_acm on kernel 6.8 exposes no latency_timer; nothing to tune
    try:
        with open(path, "w") as f:
            f.write(str(value))
        return f"latency_timer: {old} -> {open(path).read().strip()}"
    except OSError as e:
        return f"latency_timer: cannot write {path}: {e} (keep {old} ms)"


def show(tag, f):
    if f is None:
        print(f"{tag}: NO FEEDBACK")
        return
    print(f"{tag}: pos={f['outputPos']:+.4f} rad  spd={f['outputSpd']:+.4f} rad/s  "
          f"tor={f['outputTor']:+.5f} Nm  vol={f['vol']:.1f} V  "
          f"T={f['Temp']}/{f['sensor']}C  err=0x{f['MError']:05x} warn={f['MWarn']}")


def do_status(m):
    show("status", m.send_and_receive(0, 0, 0, 0, 0, 0, 0, 0))


def do_scan(m):
    print("scanning ids 0-14 (mode=0, all gains 0)")
    found = []
    for i in range(15):
        f = m.send_and_receive(i, 0, 0, 0, 0, 0, 0, 0)
        if f is not None:
            found.append(i)
            show(f"  id {i:2d}", f)
    print("found:", found or "none")


def do_damp(m, a):
    print(f"mode=1, Kp=0, Kd={a.kd}  -> damping only, no position target")
    for _ in range(5):
        show("damp", m.send_and_receive(a.id, 1, 0, 0, 0, 0, 0, a.kd))
        time.sleep(0.05)


def do_hold(m, a):
    f = m.send_and_receive(a.id, 0, 0, 0, 0, 0, 0, 0)
    p0 = f["outputPos"]
    print(f"current outputPos = {p0:+.4f} rad -> commanding hold with Kp={a.kp} Kd={a.kd}")
    t0, n = time.time(), 0
    while time.time() - t0 < a.secs:
        fb = m.send_and_receive(a.id, 1, 0, 0.0, 0.0, p0, a.kp, a.kd)
        n += 1
    print(f"loop: {n} frames in {time.time()-t0:.2f} s = {n/(time.time()-t0):.1f} Hz")
    show("hold", fb)


def do_step(m, a):
    f = m.send_and_receive(a.id, 0, 0, 0, 0, 0, 0, 0)
    p0 = f["outputPos"]
    targets = [p0, p0 + a.delta, p0, p0 - a.delta, p0]
    for tgt in targets:
        t0, fb, n = time.time(), None, 0
        while time.time() - t0 < a.secs:
            fb = m.send_and_receive(a.id, 1, 0, 0.0, 0.0, tgt, a.kp, a.kd)
            n += 1
            if abs(fb["outputPos"] - tgt) < 0.01:
                break
        dt = time.time() - t0
        print(f"target {tgt:+.4f}: settled in {dt*1000:.0f} ms ({n} frames, "
              f"{n/dt:.0f} Hz) ->")
        show("   ", fb)


def do_sine(m, a):
    f = m.send_and_receive(a.id, 0, 0, 0, 0, 0, 0, 0)
    p0 = f["outputPos"]
    print(f"centre {p0:+.4f} rad, amp {a.amp} rad, f={a.freq} Hz, Kp={a.kp} Kd={a.kd}")
    t0 = time.time()
    n = 0
    fb = None
    next_print = 0.0
    while True:
        t = time.time() - t0
        if t >= a.secs:
            break
        tgt = p0 + a.amp * math.sin(2 * math.pi * a.freq * t)
        fb = m.send_and_receive(a.id, 1, 0, 0.0, 0.0, tgt, a.kp, a.kd)
        n += 1
        if t >= next_print and fb:
            print(f"  t={t:5.2f}s tgt={tgt:+.4f} pos={fb['outputPos']:+.4f} "
                  f"spd={fb['outputSpd']:+.3f} tor={fb['outputTor']:+.4f} vol={fb['vol']:.1f}")
            next_print = t + 0.5
    dr = time.time() - t0
    print(f"loop: {n} frames / {dr:.2f} s = {n/dr:.1f} Hz")
    show("end", fb)


def do_timeout(m, a):
    """timeout bit = 1 -> the servo only accepts one frame and expects the next
    within its protection window (default 1 s). After the gap the feedback's
    timeout bit should read back 1 (and is cleared by sending control bit 0)."""
    f = m.send_and_receive(a.id, 0, 0, 0, 0, 0, 0, 0)
    p0 = f["outputPos"]
    print(f"phase 1: streaming hold with timeout=1 for 0.5 s (Kp={a.kp} Kd={a.kd})")
    t0, n = time.time(), 0
    while time.time() - t0 < 0.5:
        fb = m.send_and_receive(a.id, 1, 1, 0.0, 0.0, p0, a.kp, a.kd)
        n += 1
    print(f"  {n/(time.time()-t0):.0f} Hz, timeout flag in feedback = {fb['timeout']}")
    print("phase 2: silent for 2.0 s (no frames sent)")
    time.sleep(2.0)
    print("phase 3: one frame with timeout bit 0 -> read back the protection flag")
    fb = m.send_and_receive(a.id, 1, 0, 0.0, 0.0, p0, a.kp, a.kd)
    print(f"  timeout flag = {fb['timeout']}  (1 = protection had triggered)")
    show("  ", fb)


def do_stop(m, a):
    for _ in range(3):
        show("stop", m.send_and_receive(a.id, 0, 0, 0, 0, 0, 0, 0))
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["status", "scan", "hold", "step", "sine", "damp",
                                    "timeout", "stop"])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--kp", type=float, default=20.0)
    ap.add_argument("--kd", type=float, default=1.0)
    ap.add_argument("--delta", type=float, default=0.2)
    ap.add_argument("--amp", type=float, default=0.3)
    ap.add_argument("--freq", type=float, default=0.2)
    ap.add_argument("--secs", type=float, default=3.0)
    ap.add_argument("--no-latency-fix", action="store_true")
    a = ap.parse_args()

    if not a.no_latency_fix:
        msg = set_latency_timer(a.port)
        if msg:
            print(msg)

    m = MotorProtocolSync(a.port, BAUD, timeout=0.2)
    print(f"{a.port} @ {BAUD} baud, id {a.id}")
    try:
        if a.cmd == "stop":
            do_stop(m, a)
        elif a.cmd in ("status",):
            do_status(m)
        elif a.cmd in ("scan",):
            do_scan(m)
        else:
            {"damp": do_damp, "hold": do_hold, "step": do_step,
             "sine": do_sine, "timeout": do_timeout}[a.cmd](m, a)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        # leave the servo in a known, quiet state
        for _ in range(3):
            try:
                m.send_and_receive(a.id, 0, 0, 0, 0, 0, 0, 0)
            except Exception:
                pass
            time.sleep(0.02)
        m.close()
        print("closed (servo left in mode 0 / locked)")


if __name__ == "__main__":
    main()
