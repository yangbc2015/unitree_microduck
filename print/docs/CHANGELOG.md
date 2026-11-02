# Changelog

## Unreleased — 2026-11-02

- 3D 建模完成并入库：`print/sldprt/` 含 45 个 SolidWorks 零件（SLDPRT）、整机装配 `microduck.SLDASM` 与轴承子装配 `w_61702-2rs1.SLDASM`；新增整机 STEP 导出 `print/step/microduck-v0.2.1-assembly.STEP`。
- 打印件归档重组为 `print/stl/`、`print/sldprt/`、`print/step/`、`print/docs/`；已验证 STL 与 manifest 移入 `print/stl/`，内容不变。
- 新增源模型：左壳、躯干左侧壳、铝壳装配无刷版本（合并修改版）、M2/M2.5 螺钉与 6700 轴承源文件；补齐此前只有镜像 STL 的零件正本。
- 机械文档补齐：README、选型思路（DESIGN_GUIDE）、安装教程（BUILD_GUIDE）、打印分册（01-mechanics-printing、02-assembly）与 BOM 放入 `print/docs/`。
- 仍待完善：装配图与紧固件说明、机械制作过程照片、舵盘角度与关节零位参考照片。

## Unreleased — 2026-10-16

- 增加现状盘点、路径映射、缺口/发布审核与命令证据分级。
- 保留唯一原鸭头S288 V0 BacklashDR rotor-only61D，未改控制或动力学。
- 增加完整字段BOM CSV/XLSX，未知价格/接口留空。
- 增加策略YAML目录、空值配置/标定模板、模型静态检查工具与导航。
- 区分新增离线比较待验证入口与尚缺的ONNX/MuJoCo闭环流程。
- 增加设计/工程经验、双用户路线与英文README。
- 不执行训练、推理、仿真、硬件动作或刷写；公开发布仍受硬件资料和权属缺口阻塞。

历史版本见[版本记录](BUILD-LOG.md)。
