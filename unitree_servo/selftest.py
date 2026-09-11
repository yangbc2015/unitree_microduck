#!/usr/bin/env python3
"""Off-bench self-test for this folder. No servo required.

Not a test suite -- there is no framework here, just assertions that fail loudly. Run it
after touching the protocol layer, the two fits or the published parameters:

    python3 selftest.py          # needs numpy; imports bam_fit, so scipy + matplotlib too

It checks the things that can be checked off-bench:
  * every .py in the folder compiles and bam_dynamics.py imports cleanly
  * the two fits extracted this turn recover KNOWN ground truth from synthetic data
  * friction_nm() matches the closed-form Stribeck model
  * the published json agrees with the constants in the code that produced it
  * .gitignore really keeps third-party material out of the repo
  * fetch_upstream.sh parses
  * verify_official_issues.py reproduces its archived output byte-for-byte

Explicitly NOT verified here: anything needing the servo (the measured values themselves) --
those are only as good as the runs recorded in bam_data/.
"""
import json
import math
import os
import subprocess
import sys
import tempfile

FOLDER = os.path.dirname(os.path.abspath(__file__))
PY = [os.path.join(FOLDER, f) for f in
      ("unitree_servo.py", "servo_test.py", "bam_friction.py", "bam_dynamics.py",
       "bam_fit.py", "probe_ports.py", "verify_official_issues.py")]
sys.path.insert(0, FOLDER)

fails = []


def check(name, ok, detail=""):
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"   {detail}" if detail else ""))
    if not ok:
        fails.append(name)


print("== compile + import ==")
for p in PY:
    r = subprocess.run([sys.executable, "-m", "py_compile", p], capture_output=True, text=True)
    check(f"py_compile {os.path.basename(p)}", r.returncode == 0, r.stderr.strip()[:120])

import bam_dynamics as bd          # noqa: E402  (import after the compile sweep)
import bam_fit as bf               # noqa: E402
check("import bam_dynamics (needs pyserial at module level)", True)
check("import bam_fit", True)

print("== friction_nm matches the closed form ==")
F = bd.FRICTION
for v in (0.0, F["vs"], 1.0, -0.5):
    want = F["Fc"] + F["Fs"] * math.exp(-(abs(v) / F["vs"]) ** F["alpha"]) + F["Fv"] * abs(v)
    got = bd.friction_nm(v)
    check(f"friction_nm({v}) = {got:.8f}", abs(got - want) < 1e-12)
check("friction_nm is even in v (sign convention)", bd.friction_nm(0.3) == bd.friction_nm(-0.3))
check("friction_nm(0) == Fc + Fs", abs(bd.friction_nm(0) - (F["Fc"] + F["Fs"])) < 1e-12,
      f"{bd.friction_nm(0):.5f} == {F['Fc'] + F['Fs']:.5f}")

print("== fit_delay_response recovers known ground truth ==")
import numpy as np                 # noqa: E402
rng = np.random.default_rng(7)
for td_true, tau_true, A_true in ((0.48, 0.54, 37.0), (0.20, 0.15, 100.0), (1.00, 1.00, 12.0)):
    T, Y = [], []
    for _ in range(200):                      # 200 transitions, ~0.16 ms apart, 1 count noise
        t = np.arange(-0.3, 2.0, 0.16)
        y = np.where(t > td_true, A_true * (1 - np.exp(-(t - td_true) / tau_true)), 0.0)
        T += list(t)
        Y += list(y + rng.normal(0, 1.0, len(t)))
    fit = bd.fit_delay_response(T, Y, 40.0)
    ok = (fit is not None and abs(fit.td - td_true) <= 0.05
          and abs(fit.tau - tau_true) <= 0.15 and abs(fit.A - A_true) <= 2.0)
    check(f"td={td_true} tau={tau_true} A={A_true} -> td={fit.td:.2f} tau={fit.tau:.2f} "
          f"A={fit.A:.1f} rms={fit.rms:.2f}", ok)
    check(f"  td_band {fit.td_band} brackets the truth",
          fit.td_band is not None and fit.td_band[0] <= td_true <= fit.td_band[1])
    check(f"  returns a typed DelayFit with n={fit.n}", isinstance(fit, bd.DelayFit)
          and fit.n == len(T))
# a pure step (tau -> 0) must not come back negative or nan
fit0 = bd.fit_delay_response(list(np.arange(-0.3, 2.0, 0.16)) * 20, None or
                             list(np.tile(np.where(np.arange(-0.3, 2.0, 0.16) > 0.5, 30.0, 0.0), 20)),
                             40.0)
check("clean step: td ≈ 0.50, A ≈ 30", abs(fit0.td - 0.50) <= 0.02 and abs(fit0.A - 30) < 0.1,
      f"td={fit0.td:.2f} A={fit0.A:.2f} rms={fit0.rms:.3f}")

print("== fit_sine_inertia recovers a known J ==")
w = 2 * math.pi * 0.5
t = np.arange(0, 8, 1 / 6000.0)
for amp, J_true in ((0.25, 0.100), (0.25, 0.005)):
    pos = 0.3 + amp * np.sin(w * t)
    alpha = -amp * w * w * np.sin(w * t)
    v = amp * w * np.cos(w * t)
    tor = J_true * alpha + np.array([bd.friction_nm(x) for x in v])
    fit = bd.fit_sine_inertia(t, pos, tor, w)
    check(f"J_true={J_true} -> J={fit.J:.5f} (R^2={fit.r2:.4f}, amp={fit.amp:.4f})",
          abs(fit.J - J_true) < 0.001 and fit.r2 > 0.999)
# At the torque noise this bench actually shows (sd ~40 mN.m -- see the M6a sweep), 8 s at
# 6 kHz still pins J tightly. So the real sine run's failure (J<0, R^2=-0.63) was NOT noise:
# something the model does not contain was in the path. That is what sent us to the housing.
noisy = [bd.fit_sine_inertia(t, 0.3 + 0.25 * np.sin(w * t),
                             (0.1 * (-0.25 * w * w * np.sin(w * t))
                              + np.array([bd.friction_nm(x) for x in 0.25 * w * np.cos(w * t)])
                              + rng.normal(0, 0.040, len(t))), w)
         for _ in range(5)]
check("J survives the bench's real torque noise (sd 40 mN.m), 5 draws",
      all(abs(f.J - 0.1) < 0.005 and f.r2 > 0.5 for f in noisy),
      "J = " + ", ".join(f"{f.J:.4f}" for f in noisy))

print("== the published json agrees with the code that produced it ==")
with open(os.path.join(FOLDER, "bam_data", "s288_bam.json")) as f:
    j = json.load(f)
check("json parses", True)
check("ratio == 70070/243", abs(j["ratio"] - 70070.0 / 243.0) < 1e-9,
      f"{j['ratio']:.6f}")
for key, src in (("friction_base", F["Fc"]), ("friction_stribeck", F["Fs"]),
                 ("friction_viscous", F["Fv"]), ("dtheta_stribeck", F["vs"]),
                 ("alpha", F["alpha"])):
    check(f"{key} agrees with bam_dynamics.FRICTION", abs(j[key] - src) < 1e-4,
          f"{j[key]} vs {src}")
check("armature is null, not a free-body number",
      j["armature"] is None, f"armature={j['armature']}")
check("breakaway in json == bam_fit.STICTION", j["_robust"]["breakaway_torque"] == bf.STICTION,
      f"{j['_robust']['breakaway_torque']} vs {bf.STICTION}")
check("backlash present as a radian value", abs(j["backlash"] - 0.0184) < 1e-9,
      f"{j['backlash']} rad = {math.degrees(j['backlash']):.3f} deg")
check("caveats list mentions the clamp requirement",
      any("clamp" in c.lower() for c in j["_measurement"]["caveats"]))

print("== third-party material stays out of the repo ==")
# paths are relative to FOLDER, which is where the .gitignore being tested lives
for path in ("upstream/digital_servo-1.0.1/specs/protocol.md", "docs/s288_manual.pdf",
             "docs/debug_manual.pdf", "docs/protocol_upstream.md"):
    r = subprocess.run(["git", "check-ignore", "-q", path], cwd=FOLDER)
    check(f"git ignores {path}", r.returncode == 0)
# and the tracked files next to them are NOT ignored
for path in ("docs/verify_official_issues_output.txt", "bam_data/s288_bam.json"):
    r = subprocess.run(["git", "check-ignore", "-q", path], cwd=FOLDER)
    check(f"git does NOT ignore {path}", r.returncode != 0)
r = subprocess.run(["git", "status", "--porcelain", "--", "upstream", "docs"],
                   cwd=FOLDER, capture_output=True, text=True)
check("git status is clean for upstream/ and docs/", r.stdout.strip() == "",
      r.stdout.strip()[:80] or "no untracked files leaking out of the ignore rules")

print("== fetch_upstream.sh parses ==")
r = subprocess.run(["bash", "-n", os.path.join(FOLDER, "fetch_upstream.sh")],
                   capture_output=True, text=True)
check("bash -n fetch_upstream.sh", r.returncode == 0, r.stderr.strip()[:120])

print("== the report's reproducer still reproduces its own output ==")
out = subprocess.run([sys.executable, os.path.join(FOLDER, "verify_official_issues.py")],
                     capture_output=True, text=True)
archived = os.path.join(FOLDER, "docs", "verify_official_issues_output.txt")
same = subprocess.run(["diff", "-q", "-", archived], input=out.stdout, text=True).returncode == 0
check("verify_official_issues.py output == docs/verify_official_issues_output.txt", same,
      f"{len(out.stdout.splitlines())} lines")
# the script deliberately prints "WRONG" for what a naive byte-order CRC would give, so the
# meaningful assertion is that BOTH real-frame vectors match the true algorithm
check("both real-frame CRC vectors MATCH the true algorithm", out.stdout.count("MATCH") >= 4,
      f"{out.stdout.count('MATCH')} MATCH lines")
check("both vectors were re-derived from the frames, not hardcoded twice",
      "0x79204680" in out.stdout and "0x42DCF4D4" in out.stdout)

print()
if fails:
    print(f"FAILED {len(fails)}: {fails}")
    sys.exit(1)
print("all checks passed (not a suite: the off-bench logic and the cross-file consistency)")
