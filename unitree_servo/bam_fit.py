#!/usr/bin/env python3
"""Fit the S288 friction curve and emit BAM parameters.

Input : bam_data/M6a_friction_curve_id*.csv  (produced by bam_friction.py sweep)
Output: M6a_friction_fit.png   data + Stribeck fit + XL330 reference
        s288_bam_friction.json m6-style keys, output-side N.m
        S288_BAM_汇总表.md     the summary table the manual asks for
"""
from __future__ import annotations

import glob
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "bam_data")

# microduck XL330-M288-T m6.json (for comparison; output-side N.m)
XL330 = dict(kt=0.346, R=2.50, armature=0.00157, q_offset=0.015, command_delay=0.0102,
             friction_base=0.0119, friction_stribeck=0.00085, friction_viscous=0.00579,
             dtheta_stribeck=0.261, alpha=8.53,
             load_friction_motor=0.228, load_friction_external=0.107)

STICTION = 0.0365      # N.m output, from bam_friction.py breakaway (+/-0.0025 step)
STICTION_LO, STICTION_HI = 0.0350, 0.0375


def stribeck(v, Fc, Fs, vs, alpha, Fv):
    v = np.abs(v)
    return Fc + Fs * np.exp(-(v / vs) ** alpha) + Fv * v


def load_curve():
    cands = sorted(glob.glob(os.path.join(DATA, "M6a_friction_curve_id*.csv")))
    if not cands:
        sys.exit("no friction curve csv; run: bam_friction.py sweep ...")
    path = cands[-1]
    v, F = [], []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or line.startswith("v_out"):
                continue
            p = line.strip().split(",")
            if len(p) == 4 and p[0]:
                v.append(float(p[0])); F.append(float(p[2]))
    return path, np.array(v), np.array(F)


def main():
    path, v, F = load_curve()
    print(f"curve: {path}")
    moving = v > 0.03
    allp = np.ones_like(v, dtype=bool)      # keep the stalled point: it pins the v->0 limit

    # Bounded fit on ALL points.  With only ~9 speeds the Fc/Fs split is weakly
    # identified (the dip is narrow), so report the correlation and cross-check
    # the fit against the independent breakaway measurement.
    p0 = [0.019, 0.015, 0.03, 2.0, 0.010]
    bounds = ([0.005, 0.0, 0.002, 0.5, 0.0], [0.035, 0.035, 0.5, 6.0, 0.030])
    popt, pcov = curve_fit(stribeck, v[allp], F[allp], p0=p0, bounds=bounds, maxfev=100000)
    err = np.sqrt(np.diag(pcov))
    Fc, Fs, vs, alpha, Fv = popt
    corr = pcov[0, 1] / (err[0] * err[1])
    names = ["Fc (coulomb)", "Fs (stribeck amp)", "vs (dtheta_stribeck)",
             "alpha", "Fv (viscous)"]
    print("\nfitted Stribeck model  F(v) = Fc + Fs*exp(-(v/vs)^alpha) + Fv*v   [output N.m]")
    for n, val, e in zip(names, popt, err):
        print(f"  {n:24s} {val:9.5f}  +- {e:.5f}")
    resid = F - stribeck(v, *popt)
    rms = float(np.sqrt((resid ** 2).mean()))
    print(f"  rms residual             {rms:9.5f} N.m")
    print(f"  corr(Fc,Fs) = {corr:+.3f}   <- near -1 means the split is not identifiable")

    # robust numbers that do NOT depend on the fit
    plateau = [f for vv, f in zip(v, F) if 0.03 < vv < 0.4]
    high = [(vv, f) for vv, f in zip(v, F) if 0.4 < vv < 1.2]
    hv, hf = zip(*high)
    slope = np.polyfit(hv, hf, 1)[0]
    dip_v, dip_F = min(zip(v[moving], F[moving]), key=lambda t: t[1])
    robust = dict(
        stiction=STICTION, stall_plateau=float(F[0]), dip=float(dip_F), dip_v=float(dip_v),
        plateau=float(np.mean(plateau)), viscous_slope=float(slope),
    )
    print("\nrobust, fit-independent numbers [N.m output]")
    print(f"  breakaway / stiction      {robust['stiction']:.5f}  (bracket "
          f"{STICTION_LO}-{STICTION_HI})")
    print(f"  stalled shaft plateau     {robust['stall_plateau']:.5f} at v={v[0]:.4f} rad/s")
    print(f"  dip minimum               {robust['dip']:.5f} at v={robust['dip_v']:.4f} rad/s")
    print(f"  0.03-0.4 rad/s plateau    {robust['plateau']:.5f}")
    print(f"  viscous slope (0.4-1.2)   {robust['viscous_slope']:.5f} N.m*s/rad")
    print(f"  fit F(v->0)=Fc+Fs         {Fc+Fs:.5f}  vs stalled {F[0]:.5f} "
          f"vs breakaway {STICTION:.5f}")
    print(f"  XL330 for comparison      Fc={XL330['friction_base']} "
          f"Fs={XL330['friction_stribeck']} vs={XL330['dtheta_stribeck']} "
          f"alpha={XL330['alpha']} Fv={XL330['friction_viscous']}")


    # ---------- plot ----------
    vv = np.linspace(1e-4, max(v) * 1.05, 2000)
    vv_lin = np.linspace(0, max(v) * 1.05, 400)
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(14, 5.5), dpi=130)

    for a_ in (ax, ax2):
        a_.plot(v, F, "o", ms=7, color="#c0392b", label="S288 measured (paired +/-)",
                zorder=5)
        a_.plot(vv, stribeck(vv, *popt), "-", lw=2, color="#c0392b",
                label=f"S288 fit  Fc={Fc:.4f} Fs={Fs:.4f}\n"
                      f"          vs={vs:.4f} a={alpha:.2f} Fv={Fv:.4f}")
        a_.plot(vv, stribeck(vv, XL330["friction_base"], XL330["friction_stribeck"],
                             XL330["dtheta_stribeck"], XL330["alpha"],
                             XL330["friction_viscous"]),
                "--", lw=2, color="#2471a3", label="XL330 m6.json")
        a_.axhline(STICTION, color="#7f8c8d", ls=":", lw=1.5)
        a_.axvline(v[0], color="#7f8c8d", ls=":", lw=1.0)
        a_.text(0.055, 0.0285, f"stalled shaft (v={v[0]:.4f})",
                color="#7f8c8d", fontsize=8)
        a_.grid(alpha=0.3)
    ax.set_ylim(0.010, STICTION * 1.14)
    ax.set_xlim(-0.04, max(v) * 1.06)
    ax.text(max(v) * 1.0, STICTION * 1.015,
            f"breakaway / stiction (torque ramp) = {STICTION:.4f} Nm",
            color="#7f8c8d", fontsize=9, ha="right")
    ax.set_xlabel("output speed [rad/s]")
    ax.set_ylabel("friction torque [N.m, output side]")
    ax.set_title("S288 friction curve, ID 0, 12.0 V, ~58 C, free shaft")
    ax.legend(fontsize=7.5, loc="lower right")

    ax2.set_xscale("log")
    ax2.set_xlim(5e-4, max(v) * 1.4)
    ax2.set_ylim(0.010, STICTION * 1.12)
    ax2.set_xlabel("output speed [rad/s] (log)")
    ax2.set_title("same data, log-x: the Stribeck dip lives below 0.04 rad/s")
    ax2.legend(fontsize=7.5, loc="lower right")
    fig.tight_layout()
    png = os.path.join(DATA, "M6a_friction_fit.png")
    fig.savefig(png)

    # ---------- json (m6-style keys, S288, output side) ----------
    out = {
        "_comment": "S288 BAM friction params, measured on Jetson 2026-09-11, 12.0V, "
                    "output-side N.m. FOC closed loop => no kt/R needed. "
                    "TBD = not measured yet. NOTE: the Fc/Fs split from the Stribeck "
                    "fit is weakly identified (corr(Fc,Fs)~-1); prefer the robust "
                    "measured numbers in _robust when a single value is needed.",
        "motor": "Unitree S288 (ID 0, fw unknown)",
        "ratio": 70070.0 / 243.0,
        "friction_base": round(float(Fc), 6),
        "friction_stribeck": round(float(Fs), 6),
        "friction_viscous": round(float(Fv), 6),
        "dtheta_stribeck": round(float(vs), 6),
        "alpha": round(float(alpha), 4),
        "_robust": {
            "breakaway_torque": robust["stiction"],
            "breakaway_bracket": [STICTION_LO, STICTION_HI],
            "stall_plateau_torque": robust["stall_plateau"],
            "dip_torque": robust["dip"],
            "dip_speed": robust["dip_v"],
            "plateau_torque_0.03_0.4": robust["plateau"],
            "viscous_slope_0.4_1.2": robust["viscous_slope"],
            "torque_offset_bias": -0.003,
        },
        "load_friction_motor": None,
        "load_friction_external": None,
        "kt": None, "R": None,
        "armature": None, "q_offset": None, "command_delay": None,
        "_measurement": {
            "method": "velocity sweep kp=0 kd=2, paired +/- directions, 4 kHz loop",
            "torque_source": "servo's own rotor torque feedback (256000 counts = 1 N.m)",
            "torque_lsb_output_nm": 1 / 256000 * 70070 / 243,
            "n_points": int(len(v)),
            "rms_residual_nm": round(rms, 6),
            "corr_Fc_Fs": round(float(corr), 4),
            "speed_and_temp": "12.0-12.5V, 54-60C, free shaft (gravity bias < 0.004 N.m)",
            "caveat": "v > 1.2 rad/s points are noisy (torque sd jumps to ~110 counts)",
        },
    }
    js = os.path.join(DATA, "s288_bam_friction.json")
    with open(js, "w") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)

    # ---------- markdown summary ----------
    md = f"""# S288 舵机 BAM 标定汇总表（摩擦部分）

标定日期 2026-09-11 ｜ 舵机 Unitree S288 ID 0 ｜ 供电 12.0 V ｜ 空载自由轴（无夹持、无砝码）
环境温度 24 ℃ / 舵机壳体 54→60 ℃ ｜ 固件版本未查询（上位机是 Windows 独占）
方法：**全部用舵机自带反馈**（力矩/位置/速度），不需要砝码、力臂、测力计、逻辑分析仪。

## 一、无拟合、可直接用的实测值（推荐先填这些）

| 量 | 值 | 单位 | 怎么测的 |
|----|-----|------|---------|
| **破断力矩（静摩擦）** | **{STICTION}** | N·m | 纯力矩前馈斜坡：0.035 保持不动，0.0375 起动；±0.0025 分辨率、两方向对称 |
| 卡死平台力矩 | {robust['stall_plateau']:.5f} | N·m | 命令 0.02 rad/s 时舵机不转，实测速度 0.0009 rad/s，力矩停在 0.033 |
| 动摩擦最低点 | {robust['dip']:.5f} | N·m | v = {robust['dip_v']:.3f} rad/s 处的 Stribeck 下凹底 |
| 0.03–0.4 rad/s 平台 | {robust['plateau']:.5f} | N·m | 该速度区间成对测量均值 |
| 粘滞斜率 | {robust['viscous_slope']:.5f} | N·m·s/rad | 0.4–1.2 rad/s 段线性回归 |
| 力矩反馈零偏 | ≈ -0.003 | N·m | 命令-反馈斜率 880 vs 理论 887.9 counts/(N·m)，截距约 -3 counts |

## 二、Stribeck 拟合（次要，分割项不可辨识）

`F(v) = Fc + Fs·exp(-(v/vs)^alpha) + Fv·v`，9 个速度点，边界约束最小二乘：

| 参数 | 拟合值 | 标准误 | m6 风格键名 | 对照 XL330 m6.json |
|------|--------|--------|------------|-------------------|
| Fc | {Fc:.5f} | ±{err[0]:.5f} | `friction_base` | 0.0119 |
| Fs | {Fs:.5f} | ±{err[1]:.5f} | `friction_stribeck` | 0.00085 |
| vs | {vs:.5f} | ±{err[2]:.5f} | `dtheta_stribeck` | 0.261 |
| alpha | {alpha:.3f} | ±{err[3]:.3f} | `alpha` | 8.53 |
| Fv | {Fv:.5f} | ±{err[4]:.5f} | `friction_viscous` | 0.00579 |
| 残差 rms | {rms:.5f} | | | |

**注意 corr(Fc, Fs) = {corr:+.3f}**，且 vs / alpha 的协方差矩阵退化（标准误打印为 0，不可信）：
下凹很窄（vs≈{vs:.4f} rad/s，只有 9 个速度点、采样间隔 0.03 rad/s），所以"库仑/Stribeck 分割"
和曲线形状参数的精度有限。**可靠的是 Fc+Fs = {Fc+Fs:.5f}**，与独立测得的卡死平台
{robust['stall_plateau']:.5f} 吻合到 4 位小数（见下）。给单个数字时用第一节实测值。

## 三、结论

1. **静摩擦两个方向对称**（±0.0375 起动，±0.035 不动）→ 台架无重力负载，耦合项 bias < 0.004 N·m，
   就是零点偏置。这说明测量有效。
2. **约 0.02 rad/s 以下舵机根本不转**：这就是为什么 MuJoCo 里必须用 `frictionloss`（静摩擦死区），
   只给 `damping`（粘滞）是不够的。
3. **下凹很浅但确实存在**：0.083 rad/s 处 0.0213 → 最低 {robust['dip']:.4f}（v≈{robust['dip_v']:.3f}）
   → 1.0 rad/s 0.0281。动/静摩擦比 ≈ **{robust['dip']/STICTION:.2f}**。
   拟合出的 Stribeck 宽度 vs ≈ {vs:.4f} rad/s —— 比 XL330 的 0.261 rad/s **窄一个数量级以上**：
   S288 的摩擦更接近"纯库仑 + 死区"，下凹只在很窄的低速带里。
4. **S288 摩擦显著大于 XL330**：库仑高约 {Fc/XL330['friction_base']:.1f} 倍，粘滞高约
   {Fv/XL330['friction_viscous']:.1f} 倍。**直接拿 m6.json 的摩擦参数跑 S288 会明显偏乐观**——
   sim2real 会表现为真机"比仿真更粘、低速更不走"，这正是需要用本表替换的部分。
5. **1.2 rad/s 以上数据不可信**：力矩标准差从 ±20 跳到 ±110 counts，疑为自由机身抖动或速度环
   极限环。要测高速段必须先把舵机刚性夹持。
6. 温度 54→60 ℃ 期间同一速度的摩擦力矩漂移在 ±0.002 N·m 内，本次不给温度模型。
7. 全部数据都是**输出端** N·m（已按 288.35 还原）。用 m6 风格键名时注意 XL330 的
   `friction_base` 等也是输出端量，直接对应。

## 四、数据文件

| 文件 | 内容 |
|------|------|
| `M6a_friction_curve_id0.csv` | 成对摩擦曲线（v、F 转子 counts、F 输出端 N·m、重力偏置） |
| `M6a_friction_sweep_id0_*.csv` | 原始逐帧数据（t_ms, phase, cmd, pos_raw, tor_raw, spd_raw, vol, temp, err） |
| `M4_breakaway_id0_*.csv` | 力矩斜坡原始数据（含两方向） |
| `M6a_friction_fit.png` | 曲线 + 拟合 + XL330 对比 |
| `s288_bam_friction.json` | m6 风格参数，未测项为 null，另附 `_robust` 实测块 |

## 五、仍未标定（按性价比排序）

| 项 | 手册阶段 | 现状与可得性 |
|----|---------|-------------|
| armature（输出端等效惯量） | M6b/估 | **未测，且不需要设备**：用已知摩擦 + 力矩阶跃测角加速度反推 J |
| command_delay | M2 | 未测。本机 4 kHz 请求-响应，时间分辨率约 0.25 ms，足够 |
| backlash / 齿轮弹性 | M7 | **首测发现 0.5–1.7° 的位置相关偏差**，与"输出端编码器消除反冲"的说法不完全一致，需稳定后重测 |
| q_offset（零点） | M1 | 转子零点与输出端编码器静态差约 0.5° |
| 电压敏感性 | M3 | 未测：只有 12 V 电源，母线读数分辨率 0.5 V，太粗 |
| 最大力矩/电流限幅 | M4 | 未测：需砝码或堵转装置（手册称 0.32 A 限流） |
"""
    mdp = os.path.join(DATA, "S288_BAM_汇总表.md")
    with open(mdp, "w") as f:
        f.write(md)

    print(f"\nplot    : {png}\njson    : {js}\nmarkdown: {mdp}")


if __name__ == "__main__":
    main()
