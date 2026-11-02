# zj281136 工程笔记与策略契约（收录）

本目录收录 [zj281136/microduck-s288](https://github.com/zj281136/microduck-s288) 的 6 篇
文档，与 `unitree_servo/bam_model/`（仿真执行器模型）、
[`宇树官方_多电机强化学习部署例程.md`](../宇树官方_多电机强化学习部署例程.md)（官方存档）
同属 S288 技术链。原仓库路径镜像保留，便于与来源逐字对照。

## 来源与许可

- **来源**：zj281136/microduck-s288 `docs/`，2026-11-02 收录。
- **许可**：Apache-2.0（来源仓库 LICENSE 署名文本为 Copyright 2026 Pollen Robotics，
  其训练代码基于 pollen-robotics/microduck_rl 的 S288 适配）。
- **改动**：仅 `07_engineering_notes/sim2real.md` 文末一处——原链接
  `../08-troubleshooting.md`（未收录）改为文字说明；其余逐字收录。

## 收录清单与对本仓库的意义

| 文件 | 内容 | 对本仓库的意义 |
| --- | --- | --- |
| [`05-control-contract.md`](05-control-contract.md) | 61 维 actor 观测布局、14 关节顺序、action/HOME/last_action 语义、M1 配置数值（SDK Kp/Kd、0.55 N·m 限幅、command lag 0–25 ms、电压 DR 11–12 V）、ONNX 契约 `s288_v0_backlash_dr_rotor_pd_m1_61d_v1` | 我们观测构建器与 ONNX 校验的对齐目标；roadmap 里"XL330 动力学训的策略需重训"，这就是 S288 侧的接口定义 |
| [`05_reinforcement_learning/recipe.md`](05_reinforcement_learning/recipe.md) | reward 权重表、termination 条件、command 范围与 DR、配方文件定位 | 重训配方的静态快照；权重是初始值（课程会改），不能当性能保证 |
| [`07_engineering_notes/sim2real.md`](07_engineering_notes/sim2real.md) | 接口核对优先于调参、"谁先分叉"判读表、real target replay 的价值与限制 | sim2real 调试方法论：先核对接口语义，再动参数 |
| [`07_engineering_notes/xl330_vs_s288.md`](07_engineering_notes/xl330_vs_s288.md) | 迁移不是改名：SDK 增益单位、齿比、双编码器、firmware PD、限幅、摩擦、command lag 逐项核对 | 对应 `docs/project/s288-servo-port.md` phase 4/5 的核对清单 |
| [`07_engineering_notes/communication_analysis.md`](07_engineering_notes/communication_analysis.md) | 通信故障排查假设表、同条件单因素对照法、故障后处理 | 4 Mbps / 3并5串 整机线束 bring-up（s288-servo-port.md Risks）的直接参照 |
| [`07_engineering_notes/lessons_learned.md`](07_engineering_notes/lessons_learned.md) | 现象 → 早期假设 → 可支持的结论 → 仍需验证 的证据等级表 | 防止把相关当因果；发布实验报告的格式模板 |

## 使用注意

- 这些文档描述的是 **Pi Zero 2 W + Python + rotor-only 61D** 栈；本仓库真机栈是
  Jetson + Rust（`duck-control`/`robotd`）。借**语义、数值与流程**，代码整体搬运意义不大。
- 数值冲突时的优先级：本仓库实测（`unitree_servo/README.md`、`bam_data/`）>
  宇树官方存档 > 本文档。例：总线波特率按官方文档取 4 Mbps（3并5串），
  不以本文档栈的接法为准。
- 未收录但值得后续补充的：来源仓库 `docs/08-troubleshooting.md`、
  `04-motor-calibration.md`、`TRAIN_GUIDE.md`、`10-local-validation.md`
  （对应 `print/docs/` 里的 TODO 项）。
