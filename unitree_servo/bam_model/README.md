# S288 BAM 仿真侧执行器模型（收录）

本目录收录 [zj281136/microduck-s288](https://github.com/zj281136/microduck-s288) 的
S288 BAM（Rhoban BAM）仿真执行器建模，与上级目录 `unitree_servo/` 的台架侧标定
（`bam_friction.py` / `bam_dynamics.py` / `bam_fit.py`）对应：那边从实测拟合参数，
这边把拟合结果带进 mjlab 训练环境。对应配置即
[`print/docs/BUILD-LOG.md`](../../print/docs/BUILD-LOG.md) 里"S288 V0 BacklashDR
rotor-only 61D"的 **M1 BAM**。

## 来源与许可

- **来源仓库**：zj281136/microduck-s288，`software/training/src/mjlab_microduck/actuator/`
  与 `docs/07_engineering_notes/pd_vs_bam.md`，2026-11-02 收录。
- **许可**：Apache-2.0。来源仓库根 LICENSE 保留 Pollen Robotics 署名文本（其训练代码基于
  [pollen-robotics/microduck_rl](https://github.com/pollen-robotics/microduck_rl) 的 S288
  适配）；按 Apache-2.0 §4(b)，本目录的改动只有：`s288_bam_m1.py` 的同级 import 由
  `mjlab_microduck.actuator.friction_dr_bam` 改为相对导入，头部加注来源说明；
  `pd_vs_bam.md` 文末两处交叉引用改为未入库说明。其余文件原样。
- **未随附的依赖**：`bam`（[Rhoban BAM](https://github.com/Rhoban/bam)）、`mjlab`、
  `mujoco_warp`、`torch`、`numpy` 均不在本仓库，代码不可直接运行；收录目的是训练侧
  对接时有一份可对照的实现与参数，不是即插即用的组件。

## 文件

| 文件 | 源路径 | 说明 |
| --- | --- | --- |
| `s288_bam_m1.py` | `.../mjlab_microduck/actuator/s288_bam_m1.py` | M1 adapter：固件 PD 律 + 齿比映射 + 电压限幅 + M1 摩擦，mjlab 张量实现 |
| `friction_dr_bam.py` | `.../mjlab_microduck/actuator/friction_dr_bam.py` | `FrictionDRBamActuator`（摩擦域随机化）与 `BacklashEncoderBamActuator`（编码器在间隙输出侧、PD 读到 main+backlash） |
| `params/s288_m1.json` | `.../mjlab_microduck/actuator/params/s288_m1.json` | 单轴台架拟合的 M1 参数 |
| `pd_vs_bam.md` | `docs/07_engineering_notes/pd_vs_bam.md` | PD vs BAM 工程笔记（背景、边界、对比记录方法） |

## M1 模型结构

```text
q_target
  → 固件 PD（kp_sdk/kd_sdk，rotor q/qdot，可选 kp/kd/强度域随机化）
  → gear_ratio² × M1 ratio 映射
  → 当前电压下的输出力矩限幅（0.55 N·m @ 12 V 为 task 配置，非厂家额定）
  → 可选一阶力矩响应（torque_response_tau，默认关闭，待实测频响标定）
  → M1 库仑 + 粘滞摩擦（可 per-env 缩放）
```

回差不在 json 里：源项目用 `passive_<joint>_backlash` 无驱动铰串联在伺服关节与连杆
之间（`robot_allcollisions_backlash.xml`），由 `BacklashEncoderBamActuator` 让 PD
读到 main+backlash 之和，模拟输出侧编码器闭合位置环的真实路径。

## 与台架实测的对照

同为输出端单位。两边是不同台架、不同个体、不同拟合方法，数值应读作**范围**而非真值。

| 参数 | `s288_m1.json` | 本仓库台架实测（见上级 README 附录 A/B、`bam_data/`） |
| --- | --- | --- |
| 库仑摩擦 `friction_base` | 0.0227 N·m | 破断 0.0365；卡死平台 0.0329；库仑 Fc 0.0187（Fc+Fs=0.0329） |
| 粘滞摩擦 `friction_viscous` | 0.00874 N·m/(rad/s) | Fv 0.0109 ± 0.0011 |
| 关节惯量 `armature` | 9.95e-4 kg·m² | 5e-4（区间 4–6e-4） |
| 指令延迟 `command_delay` | ≈0（5.8e-18，拟合中不可辨识） | 0.48 ms 传输 + 0.54 ms 一阶力矩建立 |
| 回差 | json 外：2–4° recipe（BUILD-LOG） | 实测 1.056° = 0.0184 rad |
| 零位 `q_offset` | 0.0175 rad（含摆臂/夹具零位） | 编码器间偏置 −1.63°（两者都不可直接当整机关节零点） |

要点：

- json 里的 `max_torque≈14.91` 是拟合上限（辨识日志从未激励到），**不是**可用输出力矩；
  仿真限幅由 adapter/task 单独配置。
- `q_offset` 是单轴台架零位，不能替代整机 14 个 RL 关节的 rotor 标定。
- 两边延迟口径不同：他们拟合出的 command_delay 约等于零，我们的实测是传输+建立两段；
  对接训练时不要把同一段延迟重复叠加。
- 本仓库的 `bam_fit.py` 产物为同格式 json，后续可用自有台架数据重拟合 M1 参数再替换
  `params/s288_m1.json`。
