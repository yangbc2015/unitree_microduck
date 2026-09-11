# Unitree J288/S288 数字舵机 — Jetson 测试环境

硬件：Artery AT32 USB-CDC 单总线转 USB 板（`2e3c:7640`，XT30 供电 + TYPE-C 到 Jetson + PH2.0 接舵机）
实测日期：2026-09-11 ｜ 板子序列号 `3744CBB71974`

## 硬件与端口（实测结论）

| 项 | 值 |
|----|----|
| 舵机总线 | `/dev/ttyACM0`（CDC 接口 0）→ 稳定软链 `/dev/unitree_servo` |
| 第二通道 | `/dev/ttyACM1`（CDC 接口 2）→ `/dev/unitree_servo_ch2`，对舵机命令**无任何回应** |
| 波特率 | 6000000（8N1，固定；CDC 虚拟波特率，随便填也能通，但要填对以免以后换真 UART 出问题） |
| 总线上的 ID | 只有 **0**（scan 0–14 只有 0 应答；15 是广播，不回包） |
| 供电电压 | 12.0 V（S288，手册要求 12 V；J288 是 25.2 V） |
| 空闲温度 | 壳体 54–56 ℃ / 绕组 46–47 ℃ |
| 错误码 | `MError=0x00000`，`ExFlag=0`，全程无告警 |
| 请求-响应速率 | **约 4000 Hz**（阻塞式一收一发，3 s 内 12143 帧） |
| 掉/漏包 | 除扫描不存在的 ID 外，无丢包；每帧 26 B CRC 全部通过 |

udev 规则（已装，免 sudo 访问，权限组用 `robot`）：

```
/etc/udev/rules.d/99-unitree-digital-servo.rules
SUBSYSTEM=="tty", ENV{ID_VENDOR_ID}=="2e3c", ENV{ID_MODEL_ID}=="7640", \
  ENV{ID_USB_INTERFACE_NUM}=="00", GROUP="robot", MODE="0660", SYMLINK+="unitree_servo"
```

注意：用 `ATTRS{bInterfaceNumber}` 匹配在这台机器上**不生效**（规则被读取但不匹配），必须用
`ENV{ID_USB_INTERFACE_NUM}`（由 60-serial.rules 的 usb_id 内置程序填入）。

另外：本机 cdc_acm 走的是 `/sys/bus/usb/cdc_acm`，**没有** `/sys/bus/usb-serial/devices/*/latency_timer`
这个节点，所以 16 ms 延迟这个经典坑不存在（实测 4 kHz 也印证了）。

## 文件

| 文件 | 用途 |
|------|------|
| `unitree_servo.py` | 协议层 + 阻塞式收发客户端（`build_control_packet` / `parse_feedback_packet` / `MotorProtocolSync`） |
| `servo_test.py` | 台架测试 CLI：`status` / `scan` / `damp` / `hold` / `step` / `sine` / `timeout` / `stop` |
| `bam_friction.py` | BAM 摩擦标定 CLI：`sweep` / `breakaway` / `encoder` / `report`（**零外部设备**） |
| `bam_fit.py` | Stribeck 拟合 + 出图 + 生成 json/汇总表 |
| `probe_ports.py` | 不依赖任何假设，直接扫 ttyACMx 找哪一个是舵机总线 |
| `verify_official_issues.py` | 用**官方原代码 + 真机帧**验证官方文档/例程的每一个问题（可交给官方复现） |
| `docs/宇树数字舵机_官方文档与例程问题报告.md` | **给官方的独立问题报告**（A 类 4 条会导致实现错误 + B 类 9 条文档缺陷，逐条带证据与建议） |
| `docs/verify_official_issues_output.txt` | 上面那份验证脚本的运行输出留档 |
| `servo_demo_orig.py` | 官方例程原样备份（有 bug，见下） |
| `docs/s288_manual.pdf` | 官方《无刷数字舵机 J288/S288 使用手册》 |
| `docs/debug_manual.pdf` | 官方《电机调试助手（上位机）使用手册》 |
| `docs/protocol_upstream.md` | 官方仓库 specs/protocol.md |
| `upstream/digital_servo-1.0.1/` | 官方仓库 v1.0.1（去掉 Keil 构建产物和 HAL 源码，保留 App/Core/python/specs） |

## 用法

```bash
cd ~/Desktop/microduck/unitree_servo
bash fetch_upstream.sh                         # 抓官方 v1.0.1 + 两份手册 PDF（不入库）
python3 servo_test.py status                    # 读一帧状态，不动
python3 servo_test.py scan                      # 扫 0-14 哪些 ID 在线
python3 servo_test.py damp   --kd 1.0           # Kp=0 纯阻尼，最保守的一步
python3 servo_test.py hold   --kp 20 --kd 1.0 --secs 3
python3 servo_test.py step   --kp 20 --kd 1.0 --delta 0.2
python3 servo_test.py sine   --kp 40 --kd 1.5 --amp 0.3 --freq 0.2 --secs 6
python3 servo_test.py timeout --kp 10 --kd 1.0
python3 servo_test.py stop
```

所有位置/速度/力矩参数都是**输出端**（过了 288.35:1 减速箱）的量，与官方例程一致。
脚本退出前一定会补发 mode=0（锁定/松劲），Ctrl-C 也一样。

## 协议要点（含官方手册对官方仓库 spec 的纠正）

物理层：单总线半双工、8N1、6 Mbps 固定；ID 0–14，15 广播（不回包）。

控制包 20 B：`FE EE | mode | 00 | tor(i16) spd(i16) pos(i32) k_pos(i16) k_spd(i16) | crc32`
CRC 覆盖字节 0–15（含帧头）。
mode 字节：`[3:0]=id, [6:4]=status(0=锁定 1=FOC 闭环), [7]=timeout(0=禁用超时保护, 1=开启，默认 1 s)`

反馈包 26 B：`FC EE | mode | 19 B | crc32`，CRC 覆盖字节 2–21（**不含帧头**）。
反馈里 mode 字节的 timeout 位是**舵机→主机**方向：1 = 已触发超时保护，需要控制位发 0 来清除。

19 字节 feedback 实测布局（`struct.unpack('<bBBhhiIHH')`）：

| 偏移 | 类型 | 字段 | 说明 |
|------|------|------|------|
| 0 | int8 | temp | 壳体温度 ℃（实测 54–56） |
| 1 | uint8 | sensor | 绕组温度 ℃（实测 46–47） |
| 2 | **uint8** | vol | 舵机端电压，**255 = 127.5 V**（即 raw/2） |
| 3 | int16 | torque | 转子力矩，256000 = 1 N·m |
| 5 | int16 | speed | 转子速度，2.56/2π = 1 rad/s |
| 7 | int32 | pos | 转子位置，32768/2π = 1 rad |
| 11 | uint32 | MError | 故障码位域（0x01 过流、0x08 欠压、0x200 绕组过热……） |
| 15 | uint16 | OutPos:13 / ExFlag:3 | 低 13 位输出端角度（8192 = 1 圈）/ 高 3 位警告码 |
| 17 | uint16 | res | 保留 |

**纠错 1**：官方仓库 `specs/protocol.md` 写 vol 是 uint16 —— 错。手册的 `MotorData_t` 和实测字节
都对不上 19 字节；vol 是 **uint8**。若按 uint16 解析，后面所有字段全部错位。
**纠错 2**：官方 Python 例程把 vol 解析成 **int8**（`'<bbb...'`）—— 量程 >127.5 V 时符号翻转，
而且在 `parse_feedback_packet` 里 `struct.unpack('<bbb hh i I H BB')` 的 padding 语义容易看错。
本目录的 `unitree_servo.py` 已改成 `'<bBBhhiIHH'`。

**纠错 3（最容易踩）**：CRC32 不是"按内存字节流顺序"算的。真实算法是
**把待校验区当作 4 字节小端字（uint32 LE）数组，每个字从 MSB 到 LSB（即地址高→低）依次送入
CRC-32/MPEG-2**（poly 0x04C11DB7、init 0xFFFFFFFF、不反射、xorout 0）。官方
`specs/protocol.md` 只写了多项式和初值，且"same as IEEE 802.3, but bit-reversed"这个说法
是错的（IEEE 802.3 是反射 + xorout 0xFFFFFFFF，两者算法不同）。用 `zlib.crc32` 一定算不对。
另外长度必须是 4 的倍数（控制包 16、反馈包 20），但官方函数 `len/4` 会**静默丢弃**尾字节。
两个真机测试向量（见 `verify_official_issues.py`）：

```
控制包 bytes 0..15  fe ee 00 00 ... 00        -> CRC = 0x79204680
反馈包 bytes 2..21  00 36 2d 18 f4 ff ... 09 00 00 -> CRC = 0x42DCF4D4
```

换算（官方公式，输出端 ↔ 转子端差一个 RATIO = 70070/243 ≈ 288.35）：

```
k_pos  = Kp / RATIO² × 1_280_000        Kp  : 0 .. 2128.523   (输出端 N·m/rad)
k_spd  = Kd / RATIO² × 128_000_000      Kd  : 0 .. 21.285     (输出端 N·m/(rad/s))
pos    = outPos × RATIO × 32768 / 2π    outPos: ±1428.019 rad
spd    = outSpd × RATIO × 2.56  / 2π    outSpd: ±278.902 rad/s
tor    = outTor / RATIO × 256_000       outTor: 转子端 |τ| < 2^15/256000 = 0.128 N·m
```

混合控制律：`τ = τ_ff + Kp·(p_des − p) + Kd·(ω_des − ω)`（手册第六节）。
Kp=0 + Kd>0 = 纯阻尼（舵机变成阻尼器）；mode=0 是锁定。

## 台架实测记录（2026-09-11）

* `hold` Kp=20 Kd=1，3 s：位置纹丝不动（1.8120 rad），力矩 ~0.002 N·m → 静止保持正常。
* `step` ±0.2 rad，Kp=20 Kd=1：每次 **约 180–190 ms 到位**（803 帧 @4.2 kHz），
  超调约 0.01 rad（5%），峰值速度 0.24 rad/s，峰值力矩 0.037 N·m。
* `sine` ±0.3 rad @0.2 Hz，Kp=40 Kd=1.5：跟随误差约 0.01–0.02 rad，相位滞后小，
  峰值力矩 0.077 N·m，母线电压在 12.0↔12.5 V 之间跳动（raw 分辨率 0.5 V）。
* `timeout`：timeout=1 时连续发帧 → 反馈 timeout 位=0；静默 2 s 后再查 → **=1**（保护已触发）。
* 4050 Hz 的请求-响应速率远超调试助手要求的「通信频率 >60 Hz」。

## 附录 A：BAM 摩擦标定（零外部设备，2026-09-11 实测）

舵机每帧自带的力矩反馈（`kt·iq` 估计，转子端 256000 counts = 1 N·m）足以做摩擦标定，
**不需要砝码、力臂、测力计、逻辑分析仪**。三个坑：

1. **力矩量化粗**：1 LSB = 3.9e-6 N·m 转子 = 1.13e-3 N·m 输出端，而摩擦力矩只有 0.02 N·m 量级
   → 单帧没用，必须几千帧平均，且 CSV 里存 raw counts。另外有约 -3 counts（-0.003 N·m）的零偏。
2. **速度反馈很粗**：LSB = 0.0085 输出端 rad/s（实测 0.1702/0.2128/0.332 都是它的整数倍）。
   要真实速度就**用位置差分**（位置 LSB = 6.6e-7 rad）。
3. **齿轮箱弹性会假装成破断**：力矩斜坡测破断时，若不在两个方向之间彻底释放弹性（回中 + 零力矩
   静置），换向瞬间的弹性回弹会被误判为"起动"（实测踩过：-0.005 N·m 就"动"了 0.021 rad）。

方法：
```bash
python3 bam_friction.py sweep  --kd 2.0 --speeds 0.02,...,1.5 --rep 2   # 成对 ± 速度扫描
python3 bam_friction.py breakaway --step 0.0025 --tmax 0.055 --dir both  # 力矩斜坡测破断
python3 bam_fit.py                                                       # 拟合+出图+出 json
```
成对 ± 同速取 (τ₊−τ₋)/2 可**消掉重力与零偏**；实测台架 bias < 0.004 N·m 说明自由轴无负载。

### S288 实测结果（输出端 N·m，12.0 V，壳体 54–60 ℃，自由轴）

| 量 | S288 | XL330 m6.json |
|----|------|---------------|
| 破断/静摩擦 | **0.0365**（±0.0025，两方向对称） | friction_base 0.0119 |
| 卡死平台（v≈0.001） | 0.0329 | – |
| 动摩擦最低点 | 0.0185 @ 0.038 rad/s | – |
| 库仑 Fc | 0.0187 ± 0.0007 | 0.0119 |
| Stribeck Fs | 0.0142 ± 0.0016 | 0.00085 |
| Stribeck 速度 vs | 0.0095 rad/s | 0.261 |
| alpha | 3.48 | 8.53 |
| 粘滞 Fv | 0.0109 ± 0.0011 | 0.00579 |

- 拟合自校验：外推到 v→0 得 Fc+Fs = 0.03289，独立测的卡死平台 0.03290 —— 吻合到 4 位小数。
- **低于约 0.02 rad/s 舵机完全不转**（命令 0.02，实测 0.0009，力矩停在 0.033 < 破断 0.0365）
  → MuJoCo 里必须有 `frictionloss`，只给 `damping` 不够。
- S288 摩擦约为 XL330 的 **1.6×（库仑）/ 1.9×（粘滞）**，且下凹窄一个数量级。
  直接沿用 m6.json 的摩擦参数会让 sim2real 明显偏乐观。
- 1.2 rad/s 以上力矩标准差从 ±20 飙到 ±110 counts（自由机身抖动/速度环极限环），
  要测高速段必须先刚性夹持。
- `vs`/`alpha` 的协方差退化、`corr(Fc,Fs)=-0.44`：分割项精度有限，**Fc+Fs 才是可靠的**。

产物：`bam_data/S288_BAM_汇总表.md`、`s288_bam_friction.json`、`M6a_friction_fit.png`（双面板，
线性 + log-x 放大下凹）、逐帧原始 CSV。仍未标定：armature、command_delay、backlash
（首测发现 0.5–1.7° 位置相关偏差，需重测）、电压敏感性、电流限幅。

## 上游例程的三个坑

1. `python/servo_demo.py` 的串口和波特率写死在 `SERIAL_PORT = 'COM29'`，Windows 专用；
   另外它用 `import serial` 但文件没有 `if __name__` 之外的可复用接口，直接拿来当库不方便。
2. `build_control_packet` 里 `struct.pack('<hhIhh', …)` 把 `pos_des` 当**无符号** int32 打包，
   任何负角度目标直接抛 `struct.error: 'I' format requires 0 <= number`。手册写明是 `int32_t`，
   本目录已改为 `<hhihh`。
3. 解析 19 字节反馈时用 `'<bbb hh i I H BB'`，把 `vol` 当有符号字节（>63.75 V 翻转），
   且 `no-padding` 语义容易看错偏移。本目录用 `'<bBBhhiIHH'`。
4. 官方《电机调试助手》**只支持 Windows 10/11**，本机跑不了；Linux 下就用本目录的脚本。
