#!/usr/bin/env python3
"""S288 BAM friction identification with ZERO extra equipment.

Everything comes from the servo's own feedback frame:
  torque  (i16, 256000 counts = 1 N.m, ROTOR side)
  speed   (i16, 2*pi/2.56 rotor rad/s per count  <-- coarse, do NOT trust it)
  pos     (i32, 32768/2pi counts per rotor rad   <-- fine, differentiate this instead)

Sub-commands
  sweep      constant-velocity friction curve, both directions, several speeds
  breakaway  pure torque ramp (kp=kd=0) until the shaft starts moving -> stiction
  report     fit Coulomb + viscous + Stribeck from the sweep CSV
  encoder    M1/M7: command angles, compare rotor pos vs OutPos across a reversal

Sweep method:  mode=1, kp=0, kd=KD, spd_des=+/-v.  At steady state the torque the
servo must apply equals the resisting torque (friction + any gravity load), so
averaging tau over the dwell gives friction.  Running the SAME speed in both
directions and taking (tau_+ - tau_-)/2 cancels gravity and any torque offset.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
import time
from datetime import datetime

from unitree_servo import RATIO, MotorProtocolSync, emit_result

BAUD = 6_000_000
DEFAULT_PORT = "/dev/unitree_servo"
OUTDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bam_data")

TOR_COUNTS_PER_NM = 256000.0        # rotor N.m
SPD_COUNTS = 2.56 / (2 * math.pi)   # rotor rad/s -> counts
POS_COUNTS = 32768.0 / (2 * math.pi)  # rotor rad -> counts


def tor_out(counts: float) -> float:
    """rotor torque counts -> output-side N.m"""
    return counts / TOR_COUNTS_PER_NM * RATIO


def rot_of_out(out_rad_s: float) -> float:
    return out_rad_s * RATIO


class Logger:
    def __init__(self, name: str):
        os.makedirs(OUTDIR, exist_ok=True)
        self.path = os.path.join(OUTDIR, f"{name}_{datetime.now():%Y%m%d_%H%M%S}.csv")
        self.fh = open(self.path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(["# Unitree S288 friction log", f"# port {DEFAULT_PORT}",
                         f"# date {datetime.now():%Y-%m-%d %H:%M:%S}",
                         f"# supply 12.0V", f"# RATIO {RATIO:.4f}"])
        self.w.writerow(["t_ms", "phase", "spd_cmd_out", "pos_rot_counts", "tor_rot_counts",
                         "spd_rot_counts", "vol_V", "temp_C", "err"])

    def row(self, t0, phase, spd_cmd, fb):
        self.w.writerow([f"{(time.perf_counter()-t0)*1000:.3f}", phase, spd_cmd,
                         fb["pos_raw"], fb["torque_raw"], fb["speed_raw"],
                         fb["vol"], fb["Temp"], f"0x{fb['MError']:x}"])

    def close(self):
        self.fh.close()
        return self.path


def cycle(m, motor_id, kp, kd, tor, spd, pos, timeout=0):
    """one blocking request/response (mode=1 FOC closed loop)"""
    return m.send_and_receive(motor_id, 1, timeout, tor, spd, pos, kp, kd)


# ---------------------------------------------------------------- sweep

def cmd_sweep(m, a):
    log = Logger(f"M6a_friction_sweep_id{a.id}")
    speeds = [float(x) for x in a.speeds.split(",")]
    print(f"vel sweep: kp=0 kd={a.kd}, speeds {speeds} output rad/s, "
          f"{a.rep} rep, dwell {a.dwell}s")
    p_start = m.send_and_receive(a.id, 0, 0, 0, 0, 0, 0, 0)["outputPos"]
    print(f"start pos {p_start:+.4f} rad, abort limit |dp| > {a.limit} rad\n")
    results = []
    t0 = time.perf_counter()
    for v in speeds:
        for sign in (+1, -1):
            for rep in range(a.rep):
                vt = sign * v
                # settle
                t_end = time.perf_counter() + a.dwell
                while time.perf_counter() < t_end:
                    fb = cycle(m, a.id, 0.0, a.kd, 0.0, vt, 0.0)
                # record
                rec = []
                t_end = time.perf_counter() + a.rec
                while time.perf_counter() < t_end:
                    fb = cycle(m, a.id, 0.0, a.kd, 0.0, vt, 0.0)
                    log.row(t0, f"v{vt:+.3f}_r{rep}", vt, fb)
                    rec.append((time.perf_counter(), fb["pos_raw"], fb["torque_raw"],
                                fb["speed_raw"], fb["vol"], fb["Temp"], fb["MError"]))
                if fb["outputPos"] - p_start > a.limit or p_start - fb["outputPos"] > a.limit:
                    print(f"!! position limit {a.limit} rad reached at v={vt}, aborting")
                    log.close()
                    return
                span = (rec[-1][1] - rec[0][1]) / POS_COUNTS / RATIO
                dt = rec[-1][0] - rec[0][0]
                v_meas = span / dt
                tor_mean = statistics.fmean(r[2] for r in rec)
                tor_sd = statistics.pstdev(r[2] for r in rec)
                results.append(dict(v_cmd=vt, v_meas=v_meas, tor_rot=tor_mean, tor_sd=tor_sd,
                                    tor_out=tor_out(tor_mean), n=len(rec), rep=rep,
                                    vol=rec[-1][4], temp=rec[-1][5], err=rec[-1][6]))
                print(f"  v_cmd={vt:+.3f}  v_meas={v_meas:+.4f} out-rad/s  "
                      f"tau_rot={tor_mean:+8.2f} counts (+-{tor_sd:.1f})  "
                      f"tau_out={tor_out(tor_mean):+.5f} Nm  "
                      f"{rec[-1][4]:.1f}V {rec[-1][5]}C")
    log.close()

    # paired friction estimate
    print("\npaired friction  F(v) = (tau(+v) - tau(-v)) / 2   [cancels gravity/offset]")
    print("  v_out[rad/s]   F_rot[counts]   F_out[Nm]   gravity_bias_out[Nm]")
    pairs = {}
    for r in results:
        pairs.setdefault(abs(r["v_cmd"]), {"+": [], "-": []})
        pairs[abs(r["v_cmd"])]["+" if r["v_cmd"] > 0 else "-"].append(r)
    fit_rows = []
    for v in sorted(pairs):
        pos = [r["tor_rot"] for r in pairs[v]["+"]]
        neg = [r["tor_rot"] for r in pairs[v]["-"]]
        if not pos or not neg:
            continue
        f_counts = (statistics.fmean(pos) - statistics.fmean(neg)) / 2
        bias = (statistics.fmean(pos) + statistics.fmean(neg)) / 2
        v_meas = statistics.fmean(abs(r["v_meas"]) for r in pairs[v]["+"] + pairs[v]["-"])
        print(f"  {v_meas:10.4f}   {f_counts:+12.2f}   {tor_out(f_counts):+10.5f}   "
              f"{tor_out(bias):+10.5f}")
        fit_rows.append((v_meas, f_counts, tor_out(f_counts), tor_out(bias)))
    # Write a timestamped copy as well as the stable name: the stable name is scratch and gets
    # overwritten by the next sweep, and with it the only intermediate artifact behind a
    # published fit. (The raw per-frame log is timestamped already; this keeps the curve too.)
    for path in (os.path.join(OUTDIR, f"M6a_friction_curve_id{a.id}.csv"),
                 os.path.join(OUTDIR, f"M6a_friction_curve_id{a.id}_"
                                      f"{datetime.now():%Y%m%d_%H%M%S}.csv")):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([f"# friction curve, paired both directions, {datetime.now():%Y-%m-%d %H:%M:%S}"])
            w.writerow(["v_out_rad_s", "F_rot_counts", "F_out_Nm", "gravity_bias_out_Nm"])
            w.writerows(fit_rows)
    print(f"\nraw log: {log.path}")
    print(f"curve  : {OUTDIR}/M6a_friction_curve_id{a.id}.csv")
    emit_result("friction_sweep", id=a.id, kd=a.kd,
                v_out=[r[0] for r in fit_rows],
                f_out_nm=[r[2] for r in fit_rows],
                gravity_bias_out_nm=[r[3] for r in fit_rows],
                v_cmd=sorted(pairs), n_records=len(results),
                case_c=statistics.fmean(r["temp"] for r in results),
                supply_v=statistics.fmean(r["vol"] for r in results),
                curve_csv=os.path.join(OUTDIR, f"M6a_friction_curve_id{a.id}.csv"),
                raw_log=log.path)


# ---------------------------------------------------------------- breakaway

def _settle(m, motor_id, p_ref, kp=5.0, kd=1.0, secs=1.5, quiet=0.8):
    """Drive slowly back to p_ref, then release to zero torque so any elastic
    wind-up in the geartrain unwinds BEFORE the ramp starts.  Without this the
    unwind is mistaken for breakaway (seen: -0.005 N.m 'motion' that was really
    the +0.021 rad of stored deflection coming back)."""
    t_end = time.perf_counter() + secs
    while time.perf_counter() < t_end:
        cycle(m, motor_id, kp, kd, 0.0, 0.0, p_ref)
    t_end = time.perf_counter() + quiet
    fb = None
    while time.perf_counter() < t_end:
        fb = cycle(m, motor_id, 0.0, kd, 0.0, 0.0, 0.0)
    return fb


def cmd_breakaway(m, a):
    """kp=kd=0, ramp pure feedforward torque.  The shaft stays put until the
    commanded torque exceeds static friction, then it moves -- that torque IS
    the breakaway/stiction value.  Aborts the instant motion is detected."""
    log = Logger(f"M4_breakaway_id{a.id}")
    print(f"torque ramp 0 -> {a.tmax} N.m (output side) in steps of {a.step} N.m, "
          f"kp=kd=0, mode=1")
    print("!! the shaft WILL start turning -- nothing may be attached to the horn\n")
    t0 = time.perf_counter()
    taus = []
    t = a.step
    while t <= a.tmax + 1e-9:
        taus.append(round(t, 5))
        t += a.step
    signs = {"both": (+1, -1), "+": (1,), "-": (-1,)}[a.dir]
    onsets = {}
    for sign in signs:
        print(f"--- ramp {sign:+d} direction ---")
        fb = _settle(m, a.id, a.p_ref)
        p0 = fb["pos_raw"]
        print(f"  settled at {fb['outputPos']:+.4f} rad, residual torque "
              f"{fb['torque_raw']:+d} counts")
        broke = None
        for tau in taus:
            tau *= sign
            t_end = time.perf_counter() + a.hold
            while time.perf_counter() < t_end:
                fb = cycle(m, a.id, 0.0, 0.0, tau, 0.0, 0.0)
                log.row(t0, f"ramp{tau:+.4f}", 0.0, fb)
                dp_out = (fb["pos_raw"] - p0) / POS_COUNTS / RATIO
                if abs(dp_out) > a.detect:
                    print(f"  tau_out={tau:+.4f} Nm -> MOTION after {dp_out:+.5f} rad "
                          f"({fb['torque_raw']:+d} counts)")
                    broke = tau
                    break
            if broke is not None:
                break
            print(f"  tau_out={tau:+.4f} Nm held {a.hold}s: still "
                  f"({fb['torque_raw']:+d} counts)")
        if broke is None:
            print(f"  no motion up to {a.tmax} Nm in this direction")
        onsets["+" if sign > 0 else "-"] = abs(broke) if broke is not None else None
    log.close()
    _park(m, a.id)
    print(f"\nraw log: {log.path}")
    fb = m.send_and_receive(a.id, 0, 0, 0, 0, 0, 0, 0)
    emit_result("breakaway", id=a.id, step=a.step, tmax=a.tmax, detect_rad=a.detect,
                onset_out_nm=onsets,
                case_c=fb["Temp"], supply_v=fb["vol"], raw_log=log.path)


# ---------------------------------------------------------------- encoder (M1/M7)

def cmd_encoder(m, a):
    """M1 zero/scale check + M7 backlash: compare rotor pos (converted to output)
    with the independent output-side encoder, across a direction reversal."""
    log = Logger(f"M1M7_encoder_id{a.id}")
    print("M1/M7: step 0 -> +10deg -> 0 -> -10deg -> 0, compare转子 vs 输出端编码器")
    t0 = time.perf_counter()
    seq = [0.0, 0.1745, 0.0, -0.1745, 0.0]
    pts = []
    for tgt in seq:
        t_end = time.perf_counter() + a.hold
        while time.perf_counter() < t_end:
            fb = cycle(m, a.id, a.kp, a.kd, 0.0, 0.0, tgt)
            log.row(t0, f"tgt{tgt:+.4f}", 0.0, fb)
        rot_out = fb["outputPos"]
        out_enc = 2 * math.pi * (fb["ExPos"] % (2 * math.pi)) / (2 * math.pi)
        print(f"  target {math.degrees(tgt):+7.2f}deg -> 转子(输出端等效) {rot_out:+.5f} rad / "
              f"{math.degrees(rot_out):+7.3f}deg | 输出端编码器 {fb['ExPos']:.5f} rad / "
              f"{math.degrees(fb['ExPos']):+7.3f}deg | 差 {math.degrees(rot_out - fb['ExPos']):+.4f}deg")
        pts.append(dict(target_rad=tgt, rotor_out_rad=rot_out, out_enc_rad=fb["ExPos"]))
    log.close()
    _park(m, a.id)
    print(f"raw log: {log.path}")
    emit_result("encoder", id=a.id, kp=a.kp, points=pts,
                case_c=fb["Temp"], supply_v=fb["vol"], raw_log=log.path)


# ---------------------------------------------------------------- report / fit

def cmd_report(m, a):
    path = a.csv
    if not path:
        cands = sorted(f for f in os.listdir(OUTDIR) if f.startswith("M6a_friction_curve"))
        if not cands:
            print("no friction curve csv found; run `sweep` first")
            return
        path = os.path.join(OUTDIR, cands[-1])
    print(f"fitting Stribeck curve from {path}\n")
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or line.startswith("v_out"):
                continue
            p = line.strip().split(",")
            if len(p) == 4 and p[0]:
                rows.append((float(p[0]), float(p[1]), float(p[2])))
    if not rows:
        print("empty curve file")
        return
    v = [r[0] for r in rows]
    F = [r[2] for r in rows]
    print("  v[rad/s]   F[Nm]")
    for a1, b1 in zip(v, F):
        print(f"  {a1:8.4f}  {b1:8.5f}")

    # linear (viscous-dominant) tail: fit on the fastest half
    tail = sorted(zip(v, F))[len(v) // 2:]
    n = len(tail)
    sx = sum(t[0] for t in tail); sy = sum(t[1] for t in tail)
    sxx = sum(t[0] ** 2 for t in tail); sxy = sum(t[0] * t[1] for t in tail)
    den = n * sxx - sx * sx
    b = (n * sxy - sx * sy) / den if den else 0.0
    aa = (sy - b * sx) / n if n else 0.0
    print(f"\nviscous (from fastest half):  tau_viscous ~= {b:.5f} Nm*s/rad")
    print(f"coulomb intercept           :  {aa:+.5f} Nm")
    print(f"lowest measured point       :  F({v[0]:.3f}) = {F[0]:.5f} Nm  <- upper bound on stiction")
    print("\nStribeck shape needs the low-speed points; with 6+ speeds fit")
    print("  F(v) = Fc + Fs*exp(-(v/vs)^alpha) + Fv*v   yourself, or add a weights")
    print("  measurement later.  Drop these into microduck's m6-style json as")
    print("  friction_base / friction_stribeck / friction_viscous / dtheta_stribeck / alpha.")


def _park(m, motor_id):
    for _ in range(3):
        try:
            m.send_and_receive(motor_id, 0, 0, 0, 0, 0, 0, 0)
        except Exception:
            pass
        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["sweep", "breakaway", "encoder", "report"])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--kd", type=float, default=2.0, help="velocity-mode damping (output side)")
    ap.add_argument("--kp", type=float, default=2.0)
    ap.add_argument("--speeds", default="0.05,0.1,0.2,0.5,1.0")
    ap.add_argument("--rep", type=int, default=2)
    ap.add_argument("--dwell", type=float, default=0.4)
    ap.add_argument("--rec", type=float, default=0.4)
    ap.add_argument("--hold", type=float, default=1.5)
    ap.add_argument("--limit", type=float, default=3.0, help="abort if |dp| exceeds this (output rad)")
    ap.add_argument("--tmax", type=float, default=0.06)
    ap.add_argument("--step", type=float, default=0.005)
    ap.add_argument("--detect", type=float, default=0.02)
    ap.add_argument("--dir", default="both", choices=["both", "+", "-"])
    ap.add_argument("--p_ref", type=float, default=None,
                    help="position to return to between ramps (default: wherever we are)")
    ap.add_argument("--csv", default=None)
    a = ap.parse_args()

    m = MotorProtocolSync(a.port, BAUD, timeout=0.2)
    print(f"{a.port} @ {BAUD}, id {a.id}")
    if a.p_ref is None and a.cmd == "breakaway":
        a.p_ref = m.send_and_receive(a.id, 0, 0, 0, 0, 0, 0, 0)["outputPos"]
    try:
        {"sweep": cmd_sweep, "breakaway": cmd_breakaway,
         "encoder": cmd_encoder, "report": cmd_report}[a.cmd](m, a)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        _park(m, a.id)
        m.close()
        print("servo parked (mode 0)")


if __name__ == "__main__":
    main()
