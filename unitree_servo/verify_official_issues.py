#!/usr/bin/env python3
"""Reproduce every claim in 官方问题报告 using the ORIGINAL upstream v1.0.1 code
and real frames captured from an S288 on 2026-09-11.

Run: python3 verify_official_issues.py
"""
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
UP = os.path.join(HERE, "upstream", "digital_servo-1.0.1")
if not os.path.isdir(UP):
    sys.exit(f"upstream/ not fetched.\n  run: {os.path.join(HERE, 'fetch_upstream.sh')}")
sys.path.insert(0, os.path.join(UP, "python"))
import servo_demo  # untouched official example

FRAME = bytes.fromhex("fc ee 00 36 2d 18 f4 ff 00 00 1b 94 29 00 00 00 00 00 3f 09 00 00 d4 f4 dc 42")
STOP = bytes.fromhex("fe ee 00 00 00 00 00 00 00 00 00 00 00 00 00 00 80 46 20 79")


def crc_naive_bytes(data: bytes) -> int:
    """CRC-32/MPEG-2 applied to the bytes in memory order (what a naive reader
    of protocol.md would implement).  This is NOT what the servo uses."""
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if crc & 0x80000000 \
                else (crc << 1) & 0xFFFFFFFF
    return crc


def reorder_words(data: bytes) -> bytes:
    """Each 4-byte little-endian word reversed internally (= what you get by
    reading the buffer as uint32 LE words and processing each MSB-first)."""
    out = bytearray()
    for i in range(len(data) // 4):
        out += data[i * 4 + 3:i * 4 + 4] + data[i * 4 + 2:i * 4 + 3] \
            + data[i * 4 + 1:i * 4 + 2] + data[i * 4:i * 4 + 1]
    return bytes(out)


def crc_official(data: bytes) -> int:
    """The real algorithm, verified against two frames captured from hardware:
    CRC-32/MPEG-2 over the buffer processed as 4-byte little-endian words."""
    return crc_naive_bytes(reorder_words(data))


def official_crc(data: bytes) -> int:
    """Faithful port of crc_ccitt.h::crc32_lookup_byte_by_byte (table + len/4 loop)."""
    table = servo_demo.CRC32_TABLE
    crc = 0xFFFFFFFF
    for i in range(len(data) // 4):
        for b in (data[i * 4 + 3], data[i * 4 + 2], data[i * 4 + 1], data[i * 4]):
            crc = table[((crc >> 24) ^ b) & 0xFF] ^ ((crc << 8) & 0xFFFFFFFF)
    return crc


print("=" * 78)
print("B1  specs/protocol.md 的 feedback 字段表算出来是 20 字节, 帧里只有 19")
print("=" * 78)
spec = 1 + 1 + 2 + 2 + 2 + 4 + 4 + 2 + 1 + 1
manual = 1 + 1 + 1 + 2 + 2 + 4 + 4 + 2 + 2
stm32 = 1 + 1 + 1 + 2 + 2 + 4 + 4 + 2 + 1 + 1
print(f"  帧长等式      : 26 = head(2) + mode(1) + fbk(19) + crc(4)  ->  fbk = 19")
print(f"  protocol.md 表: temp1 sensor1 vol(uint16)2 torque2 speed2 pos4 MError4 "
      f"OutPos/ExFlag2 ExSensor2_1 ExCom_1 = {spec}  <-- 多 1 字节")
print(f"  使用手册      : 同上但 vol(uint8)1 + res(uint16)2              = {manual}")
print(f"  STM32 RIS_Fbk_t: temp1 sensor1 vol(uint8)1 ... ExSensor2_1 ExCom_1 = {stm32}")
print("  结论: 只有 protocol.md 把 vol 当成 uint16, 于是表长变成 20, 与它自己上面写的")
print("        '| 3 | 19 | fbk |' 自相矛盾。三份官方文档里它是唯一的例外。")

print()
print("=" * 78)
print("B2  vol 的类型在官方四处有四种写法")
print("=" * 78)
print("  specs/protocol.md        : uint16,  frames bytes 2-3,  value/2 = volts")
print("  使用手册 MotorData_t     : uint8,   255 表示 127.5V")
print("  STM32 protocol.h RIS_Fbk_t: uint8,   '电机端电压 0-127.5V 255表示127.5V'")
print("  python/servo_demo.py     : int8      '<bbb hh i I H BB'")
print(f"\n  实测帧: {FRAME.hex(' ')}")
fbk = FRAME[3:22]
temp, sensor, vol, torque, speed, pos, merr, ope, res = struct.unpack("<bBBhhiIHH", fbk)
print(f"\n  uint8(手册/STM32): vol_raw={vol} -> {vol/2:.1f} V")
print(f"     配套: temp={temp}C sensor={sensor}C torque={torque}->"
      f"{torque/256000*70070/243:+.5f} Nm(out) pos={pos}->"
      f"{2*3.141592653589793*pos/32768/(70070/243):.4f} rad(out) "
      f"OutPos={ope & 0x1FFF}->{2*3.141592653589793*(ope & 0x1FFF)/8192:.4f} rad "
      f"MError=0x{merr:x} ExFlag={ope >> 13}")
print("     -> 12.0 V 与实测供电一致, 其余字段全部物理自洽  ==>  uint8 才是对的")
print(f"\n  uint16(protocol.md): vol_raw={struct.unpack_from('<H', fbk, 2)[0]} -> "
      f"{struct.unpack_from('<H', fbk, 2)[0]/2:.1f} V   <-- 不可能; 且后续字段全体右移 1 字节")
v_i8 = struct.unpack_from("<b", fbk, 2)[0]
print(f"  int8(servo_demo.py): vol_raw={v_i8} -> {v_i8/2:.1f} V  (本帧巧合正确)")
for raw in (0x7F, 0x80, 0xFF):
    sv = struct.unpack_from("<b", bytes([raw]))[0]
    print(f"      raw=0x{raw:02X}: int8={sv:4d} -> {sv/2:6.1f} V   应为 {raw/2:5.1f} V"
          f"{'   <-- 符号翻转' if sv < 0 else ''}")

print()
print("=" * 78)
print("B3  servo_demo.build_control_packet(): 不能发负位置目标")
print("=" * 78)
try:
    pkt = servo_demo.build_control_packet(0, 1, 0, 0.0, 0.0, -0.1, 0.0, 0.0)
    print(f"  pos_des=-0.1 竟然成功: {pkt.hex(' ')}")
except struct.error as e:
    print(f"  pos_des=-0.1 rad -> struct.error: {e}")
try:
    pkt = servo_demo.build_control_packet(0, 1, 0, 0.0, 0.0, +0.1, 0.0, 0.0)
    print(f"  pos_des=+0.1 rad -> OK : {pkt.hex(' ')}")
except struct.error as e:
    print(f"  pos_des=+0.1 rad -> struct.error: {e}")
try:
    servo_demo.build_control_packet(0, 1, 0, -0.05, 0.0, 0.0, 0.0, 0.0)
    print("  tor_des=-0.05  -> OK (tor 用的是 'h', 有符号, 写法正确)")
except struct.error as e:
    print(f"  tor_des=-0.05 -> struct.error: {e}")
print("  代码: struct.pack('<hhIhh', tor, spd, pos, k_pos, k_spd)   <- 'I' = unsigned int32")
print("  手册 ControlData_t: int32_t pos_des;   /  STM32 RIS_Comd_t: int32_t pos_des;")
print("  -> 两个官方 C 定义都是有符号, 只有 Python 例程写成无符号. 修复: '<hhihh'")
print("  -> 影响: 任何负角度目标(机器人关节下半程!)直接抛异常, 例程完全无法做双向运动")

print()
print("=" * 78)
print("B4  crc_ccitt.h / servo_demo.crc32_lookup: len 必须是 4 的倍数, 但没说也没检查")
print("=" * 78)
print("  uint32_t crc32_lookup_byte_by_byte(const uint8_t* data, size_t len)")
print("  { ... size_t word_count = len / 4;  for (i = 0; i < word_count; ++i) ... }")
print("  函数名叫 byte_by_byte, 实际按 4 字节字处理 —— 这本身是算法要求(见 B5),")
print("  但 len 不是 4 的倍数时尾部字节被静默丢弃、不报错, 调用者拿不到任何提示。")
print("  python/servo_demo.crc32_lookup 用 'while i + 3 < n' 同样静默丢弃。")
for n in (16, 17, 19, 20, 21, 26):
    buf = bytes(range(n))
    off = official_crc(buf)
    ref = crc_official(buf[:n - n % 4])
    flag = "   <-- 尾部字节被丢弃" if n % 4 else ""
    print(f"  len={n:2d}: 官方函数 0x{off:08X} == 前 {n - n % 4:2d} 字节的结果 "
          f"0x{ref:08X} -> {off == ref}{flag}")
print("  当前调用点传 16 和 20, 都是 4 的倍数, 所以现在不出错。但:")
print("   (a) 文档从未说明'长度必须是 4 的倍数'这个约束;")
print("   (b) 使用者若按 protocol.md 的字段表对 19 字节 fbk 求 CRC, 会静默得到错误值;")
print("   (c) 头文件叫 crc_ccitt.h, 但算法是 CRC-32 (poly 0x04C11DB7), 与 CCITT(16bit,")
print("       0x1021) 无关, 命名会误导。")

print()
print("=" * 78)
print("B5  CRC32 的真实算法文档里没写, 也没给测试向量  <-- 最严重的一条")
print("=" * 78)
c_hw = struct.unpack_from("<I", STOP, 16)[0]
c_naive = crc_naive_bytes(STOP[:16])
c_real = crc_official(STOP[:16])
c_demo = servo_demo.crc32_lookup(STOP[:16])
print("  控制包测试向量 (CRC 覆盖 bytes 0..15, 共 16 字节):")
print(f"    {STOP[:16].hex(' ')}")
print(f"    硬件帧内 CRC 字节 {STOP[16:].hex(' ')} -> 0x{c_hw:08X}")
print(f"      CRC-32/MPEG-2 按内存字节顺序   -> 0x{c_naive:08X}  "
      f"{'MATCH' if c_naive == c_hw else 'WRONG'}")
print(f"      按 4 字节小端字处理(真实算法)  -> 0x{c_real:08X}  "
      f"{'MATCH' if c_real == c_hw else 'WRONG'}")
print(f"      官方 servo_demo.crc32_lookup   -> 0x{c_demo:08X}  "
      f"{'MATCH' if c_demo == c_hw else 'WRONG'}")

f_hw = struct.unpack_from("<I", FRAME, 22)[0]
f_naive = crc_naive_bytes(FRAME[2:22])
f_real = crc_official(FRAME[2:22])
f_demo = servo_demo.crc32_lookup(FRAME[2:22])
print("\n  反馈包测试向量 (CRC 覆盖 bytes 2..21, 共 20 字节, 不含帧头):")
print(f"    {FRAME[2:22].hex(' ')}")
print(f"    硬件帧内 CRC 字节 {FRAME[22:].hex(' ')} -> 0x{f_hw:08X}")
print(f"      按内存字节顺序                 -> 0x{f_naive:08X}  "
      f"{'MATCH' if f_naive == f_hw else 'WRONG'}")
print(f"      按 4 字节小端字处理(真实算法)  -> 0x{f_real:08X}  "
      f"{'MATCH' if f_real == f_hw else 'WRONG'}")
print(f"      官方 servo_demo.crc32_lookup   -> 0x{f_demo:08X}  "
      f"{'MATCH' if f_demo == f_hw else 'WRONG'}")

print("""
  protocol.md 只写了一句:
     "Polynomial: 0x04C11DB7 (same as IEEE 802.3, but bit-reversed processing)
      Initial value: 0xFFFFFFFF"
  问题:
   1) 这句话不足以复现算法。真实算法是: 把 buffer 当作 4 字节小端字数组, 每个字从 MSB
      到 LSB 依次送入 CRC-32/MPEG-2 (即每个 4 字节组内部倒序), 而不是按内存字节流顺序。
      用上面两个向量可以立刻验证: 按字节流顺序算会得到完全不同的值。
   2) "same as IEEE 802.3, but bit-reversed processing" 这个说法不准确:
        CRC-32/MPEG-2      : poly 0x04C11DB7, init 0xFFFFFFFF, refin=false, refout=false, xorout=0
        CRC-32/ISO-HDLC    : poly 0x04C11DB7(反射实现为 0xEDB88320), refin/refout=true,
        (IEEE 802.3)         xorout=0xFFFFFFFF
      两者是不同的算法。文档没写 refin/refout/xorout。
   3) 直接套标准库一定失败(zlib.crc32 等), 且没有任何测试向量可以自查。
  建议: spec 里补上完整参数 + 上面两个"帧实测"测试向量 + 一段 10 行的参考实现。
""")

print()
print("=" * 78)
print("B6  同一字段的枚举值在官方三处不一致")
print("=" * 78)
print("  RIS_Mode_t.status (STM32 protocol.h): 0.锁定 1.FOC闭环 2.编码器校准 3.保留")
print("  MotorCmd_t.mode  (STM32 protocol.h): 0:空闲 1:FOC控制  2:电机标定")
print("  MotorData_t.mode (STM32 protocol.h): 0:空闲 1:FOC控制  2:电机标定")
print("  使用手册 §9 的 RIS_Mode_t         : 0.锁定 1.FOC闭环   (只列了 2 个)")
print("  specs/protocol.md                 : 0 = stop & lock torque; 1 = hybrid closed-loop")
print("  而 modify_data() 里 SATURATE(mode, 0, 1) -> 2/3 在参考实现里根本发不出去")
print("  -> 3 bit 的模式字段实际只有 0/1 可用已被隐含, 但文档没说清楚。")

print()
print("=" * 78)
print("B7  舵机数量: 仓库说 16, 手册说 15")
print("=" * 78)
for f, pat in ((f"{UP}/specs/protocol.md", "Bus capacity"),
               (f"{UP}/README.md", "Bus ID")):
    for line in open(f, encoding="utf-8"):
        if pat in line:
            print(f"  {f.split('/')[-1]:16s}: {line.strip()}")
print("  README.zh-CN / 使用手册 §7: 最大支持一条总线上连接 15 个舵机（地址 0~14，15 为广播）")
print("  -> 英文文档写 'ID 0-15, ID 15 = broadcast' 字面自相矛盾(ID 15 既是舵机又是广播)")

print()
print("=" * 78)
print("B8  python/README.md 的接线说明与仓库其它文档冲突")
print("=" * 78)
print("  python/README.md  : 'Connect PC serial adapter (USB-to-TTL) to the servo bus'")
print("  stm32/porting-guide.md: 'You cannot connect MCU TX/RX pins directly to the servo")
print("                      - a half-duplex transceiver adapter is required.'")
print("  使用手册 §7       : 必须经'单总线串口转 USB 模块'")
print("  -> 单总线是半双工 1-Wire, 直连 USB-TTL 的 TX/RX 收不到回包(实测必须用官方转接板)")
print("  -> 另外 python/README.md 建议 'verify baud rate = 6 Mbps' 并给 /dev/ttyUSB0,")
print("     但官方转接板是 USB CDC, 枚举成 /dev/ttyACM*, 波特率是虚拟值; 文档未说明。")
