#!/usr/bin/env python3
"""Unitree J288/S288 digital servo protocol layer + blocking request/response client.

Ported from the official example (digital_servo v1.0.1, python/servo_demo.py).
Physical: half-duplex TTL multi-drop bus, 8N1, 6 Mbit/s fixed, IDs 0-15 (15 = broadcast).

Packet layout (see specs/protocol.md of the upstream repo):
  control  20 B: FE EE | mode | 00 | tor(i16) spd(i16) pos(i32) k_pos(i16) k_spd(i16) | crc32(0..15)
  feedback 26 B: FC EE | mode | 19 B fbk                                        | crc32(2..21)
"""

import struct

import json

import serial

# ==================== 协议常量 ====================
RATIO = 70070.0 / 243.0
M_PI = 3.14159265358979323846

# CRC32 查表
CRC32_TABLE = [
    0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005,
    0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD,
    0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
    0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3, 0x709F7B7A, 0x745E66CD,
    0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039, 0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5,
    0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
    0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022, 0xCA753D95,
    0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1, 0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D,
    0x34867077, 0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
    0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA,
    0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE, 0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02,
    0x5E9F46BF, 0x5A5E5B08, 0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
    0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692,
    0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6, 0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A,
    0xE0B41DE7, 0xE4750050, 0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
    0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34, 0xDC3ABDED, 0xD8FBA05A,
    0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637, 0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB,
    0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
    0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5, 0x3F9B762C, 0x3B5A6B9B,
    0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF, 0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623,
    0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
    0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604, 0xC960EBB3,
    0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7, 0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B,
    0x9B3660C6, 0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
    0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C,
    0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8, 0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24,
    0x119B4BE9, 0x155A565E, 0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
    0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654,
    0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0, 0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C,
    0xE3A1CBC1, 0xE760D676, 0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
    0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662, 0x933EB0BB, 0x97FFAD0C,
    0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668, 0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4
]

def crc32_lookup(data: bytes) -> int:
    crc = 0xFFFFFFFF
    n = len(data)
    i = 0
    while i + 3 < n:
        b0 = data[i + 3]
        b1 = data[i + 2]
        b2 = data[i + 1]
        b3 = data[i]
        crc = CRC32_TABLE[(crc >> 24) ^ b0] ^ ((crc << 8) & 0xFFFFFFFF)
        crc = CRC32_TABLE[(crc >> 24) ^ b1] ^ ((crc << 8) & 0xFFFFFFFF)
        crc = CRC32_TABLE[(crc >> 24) ^ b2] ^ ((crc << 8) & 0xFFFFFFFF)
        crc = CRC32_TABLE[(crc >> 24) ^ b3] ^ ((crc << 8) & 0xFFFFFFFF)
        i += 4
    return crc


# ==================== 打包与解析函数 ====================

def build_control_packet(motor_id: int, mode: int, timeout: int,
                        outputTor: float, outputSpd: float, outputPos: float,
                        Kp: float, Kd: float) -> bytes:
    motor_id = max(0, min(15, motor_id))
    mode = max(0, min(1, mode))
    timeout = 1 if timeout else 0
    Kp = max(0.0, min(2128.523254, Kp))
    Kd = max(0.0, min(21.285233, Kd))
    outputTor = max(-36.9093, min(36.908174, outputTor))
    outputSpd = max(-278.90143, min(278.90143, outputSpd))
    outputPos = max(-1428.018898, min(1428.018898, outputPos))

    mode_byte = ((motor_id & 0x0F) |
                 ((mode & 0x07) << 4) |
                 ((timeout & 0x01) << 7))

    k_pos_val = int(round(Kp / (RATIO * RATIO) * 1280000.0))
    k_spd_val = int(round(Kd / (RATIO * RATIO) * 128000000.0))
    pos_des_val = int(round(outputPos * RATIO * 32768.0 / M_PI / 2.0))
    spd_des_val = int(round(outputSpd * RATIO * 2.560 / M_PI / 2.0))
    tor_des_val = int(round(outputTor / RATIO * 256000.0))

    k_pos_val = max(-32768, min(32767, k_pos_val))
    k_spd_val = max(-32768, min(32767, k_spd_val))
    spd_des_val = max(-32768, min(32767, spd_des_val))
    tor_des_val = max(-32768, min(32767, tor_des_val))
    pos_des_val = max(-2147483648, min(2147483647, pos_des_val))

    # NOTE: pos_des is a *signed* int32 (upstream demo packs 'I' = unsigned and
    # therefore raises on any negative target).
    comd_bytes = struct.pack('<hhihh', tor_des_val, spd_des_val, pos_des_val, k_pos_val, k_spd_val)
    packet_without_crc = b'\xFE\xEE' + struct.pack('B', mode_byte) + b'\x00' + comd_bytes
    crc = crc32_lookup(packet_without_crc)
    full_packet = packet_without_crc + struct.pack('<I', crc)
    assert len(full_packet) == 20
    return full_packet


def parse_feedback_packet(data: bytes):
    if len(data) != 26:
        return None
    if data[0] != 0xFC or data[1] != 0xEE:
        return None

    mode_byte = data[2]
    motor_id = mode_byte & 0x0F
    mode = (mode_byte >> 4) & 0x07
    timeout = (mode_byte >> 7) & 0x01

    # 提取 19 字节的 fbk 数据
    fbk = data[3:22]  # 22 - 3 = 19
    if len(fbk) != 19:
        return None

    try:
        temp, sensor, vol, torque, speed, pos, MError, outpos_exflag, res = \
            struct.unpack('<bBBhhiIHH', fbk)
    except struct.error as e:
        print(f"[解析错误] {e}, fbk length={len(fbk)}")
        return None

    # 分离位域
    OutPos = outpos_exflag & 0x1FFF      # 低13位
    ExFlag = (outpos_exflag >> 13) & 0x07  # 高3位

    # CRC 校验：从 mode_byte 开始共 20 字节（data[2:22]）
    crc_received = struct.unpack('<I', data[22:26])[0]
    crc_computed = crc32_lookup(data[2:22])
    if crc_received != crc_computed:
        return None

    # 转换为物理量
    vol_f = vol / 2.0
    ExPos = 2 * M_PI * OutPos / 8192.0
    outputSpd = (speed / 2.56) * 2 * M_PI / RATIO
    outputTor = (torque / 256000.0) * RATIO
    outputPos = 2 * M_PI * pos / 32768.0 / RATIO

    return {
        'motor_id': motor_id,
        'mode': mode,
        'timeout': timeout,
        'Temp': temp,
        'sensor': sensor,
        'vol': vol_f,
        'MError': MError,
        'MWarn': ExFlag,
        'OutPos_raw': OutPos,
        # raw counts -- for BAM/friction work the physical units are quantised,
        # the counts are what you want to average
        'pos_raw': pos,
        'torque_raw': torque,
        'speed_raw': speed,
        'vol_raw': vol,
        'ExPos': ExPos,
        'outputTor': outputTor,
        'outputSpd': outputSpd,
        'outputPos': outputPos
    }



class MotorProtocolSync:
    def __init__(self, port, baudrate, timeout=0.1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def send_and_receive(self, motor_id, mode, timeout_flag,
                         outputTor, outputSpd, outputPos, Kp, Kd):
        # 1. 发送控制包
        cmd = build_control_packet(motor_id, mode, timeout_flag,
                                   outputTor, outputSpd, outputPos, Kp, Kd)
        self.ser.write(cmd)
        # print(f"[发送] ID={motor_id}, Torque={outputTor:.3f} Nm")

        # 2. 立即读取 26 字节反馈（阻塞，带超时）
        feedback = self.ser.read(26)
        if len(feedback) != 26:
            print(f"[错误] 未收到完整反馈包（收到 {len(feedback)} 字节）")
            return None

        # 3. 解析反馈
        parsed = parse_feedback_packet(feedback)
        if parsed is None:
            print("[错误] 反馈包校验失败或格式错误")
            return None

        return parsed

    def close(self):
        if self.ser.is_open:
            self.ser.close()




# ==================== helpers ====================

RESULT_MARK = "@@RESULT@@"


def emit_result(kind: str, **kw) -> None:
    """Print one machine-readable line, after the human output.

    `bam_unit.py` runs these CLIs as subprocesses and reads exactly this line back, so a
    per-unit record never depends on parsing prose. One line per measurement, always with
    the same marker, so a missing field is loud rather than silently absent.
    """
    print(f"{RESULT_MARK} {json.dumps({'kind': kind, **kw}, sort_keys=True, default=float)}")


def raw_to_vol(raw_vol: int) -> float:
    return raw_vol / 2.0


def _fmt(d, nd=4):
    return {k: (round(v, nd) if isinstance(v, float) else v) for k, v in d.items()}
