# protocol.py - 上下位机通信协议定义
# 与 ServoBoardMain/UpperCommTask.cpp 保持一致

import struct
from enum import Enum
from typing import List, Tuple, Optional

# ============ 电机控制模式 ============
class ControlMode(Enum):
    """电机控制模式"""
    JOINT_ANGLE = 0    # 目标关节角度控制（CMD_ANGLE_CTRL）
    DIRECT_MOTOR = 1   # 直接电机位置控制（CMD_MOTOR_POS）


# ============ 协议常量 ============
PROTOCOL_HEADER = 0xFE
PROTOCOL_TAIL = 0xFF

# 上行 (下位机 → 上位机) 包类型
# 与旧版终端监控脚本 client0312.py 保持一致：
# 0x01 角度传感器；0x02 校准反馈；0x03 伺服绝对角度；0x04 关节调试；0x05 伺服遥测
PACKET_TYPE_SENSOR = 0x01          # 角度传感器数据（编码器原始值）
PACKET_TYPE_CALIB_ACK = 0x02       # 校准反馈
PACKET_TYPE_SERVO_ANGLE = 0x03     # 伺服绝对角度 + 在线标志
PACKET_TYPE_JOINT_DEBUG = 0x04     # 单关节调试信息（target/actual/loop 输出）
PACKET_TYPE_SERVO_TELEM = 0x05     # 伺服遥测：速度/负载/电压/温度 + 在线标志

# 下行 (上位机 → 下位机) 指令
CMD_CALIBRATE = 0xCA
CMD_ANGLE_CTRL = 0xCB          # 后跟 21×4 字节 float (小端序)
CMD_START = 0xCC
CMD_STOP = 0xCD
CMD_RESET = 0xCE               # 重置
CMD_CALIB_DATA = 0xCF          # 后跟 21×4 字节 float (小端序)，21 个关节 0° 对应的编码器原始值（转为 float 发送）
CMD_MOTOR_POS = 0xD0           # 直接电机位置控制：后跟 21×2 字节大端，电机目标编码器原始值

# 设备参数
ENCODER_COUNT = 21
MOTOR_COUNT = 21
# 校准：负载阈值，超过此值判定为到达 0°（待 PACKET_TYPE_MOTOR_STATUS 支持负载后生效）
CALIB_LOAD_THRESHOLD = 0.3
# 触觉：5 指，每指 3 个传感器，每传感器 N×M 触点 + 三维合力 (见 data_models)
ERROR_VAL_FLAG = 0xFFFF
# Legacy disconnect sentinel used by client.py monitor path
DISCONNECT_SENTINEL = 0x7FFF

# ============ 通信参数（串口与队列）============
BAUDRATE = 921600
SERIAL_TIMEOUT = 0.1
RX_QUEUE_MAXSIZE = 200
TX_QUEUE_MAXSIZE = 100
RX_POLL_SLEEP = 0.005
TX_QUEUE_GET_TIMEOUT = 0.1


def parse_sensor_packet(payload: bytes) -> Tuple[List[int], List[bool]]:
    """解析 Type 0x01 传感器包: 21 个关节角编码器原始值 (各 2 字节大端)，非电机轴传感器"""
    angles = []
    errors = []
    if len(payload) < ENCODER_COUNT * 2:
        return angles, errors
    for i in range(ENCODER_COUNT):
        val = (payload[i * 2] << 8) | payload[i * 2 + 1]
        is_error = (val == ERROR_VAL_FLAG) or (val == DISCONNECT_SENTINEL)
        errors.append(is_error)
        angles.append(0 if is_error else val)
    return angles, errors


def parse_calib_ack(payload: bytes) -> Optional[int]:
    """解析 Type 0x02 校准反馈: 1=PENDING, 2=SUCCESS, 3=FAILED"""
    if len(payload) < 1:
        return None
    return payload[0]


def parse_servo_angle_packet(payload: bytes) -> Optional[Tuple[List[int], List[bool]]]:
    """
    解析 Type 0x03 伺服绝对角度包。
    帧格式与 client0312.py 保持一致:
        - 前 ENCODER_COUNT×4 字节: 伺服绝对角度（大端有符号 32 位整数）
        - 后 ENCODER_COUNT 字节:   对应伺服是否在线 (1=online, 0=offline)
    """
    expected_len = ENCODER_COUNT * 4 + ENCODER_COUNT
    if len(payload) != expected_len:
        return None
    angles: List[int] = []
    for i in range(ENCODER_COUNT):
        start = i * 4
        angles.append(struct.unpack(">i", payload[start:start + 4])[0])
    online_flags: List[bool] = []
    base = ENCODER_COUNT * 4
    for i in range(ENCODER_COUNT):
        online_flags.append(payload[base + i] == 1)
    return angles, online_flags


def parse_servo_telem_packet(
    payload: bytes,
) -> Optional[Tuple[List[int], List[int], List[int], List[int], List[bool]]]:
    """
    解析 Type 0x05 伺服遥测包。
    帧格式与 client0312.py 保持一致:
        每个伺服 7 字节:
            - speed: int16 大端
            - load:  int16 大端
            - volt:  uint8  (0.1V 单位)
            - temp:  uint8  (摄氏度)
            - online: uint8 (1 在线, 0 离线)
    """
    expected_len = ENCODER_COUNT * 7
    if len(payload) != expected_len:
        return None
    speeds: List[int] = []
    loads: List[int] = []
    volts: List[int] = []
    temps: List[int] = []
    online: List[bool] = []
    for i in range(ENCODER_COUNT):
        base = i * 7
        speeds.append(struct.unpack(">h", payload[base:base + 2])[0])
        loads.append(struct.unpack(">h", payload[base + 2:base + 4])[0])
        volts.append(payload[base + 4])
        temps.append(payload[base + 5])
        online.append(payload[base + 6] == 1)
    return speeds, loads, volts, temps, online


def parse_joint_debug_packet(
    payload: bytes,
) -> Optional[Tuple[int, bool, float, float, float, float, float]]:
    """
    解析 Type 0x04 关节调试包。
    帧格式与 client0312.py 保持一致 (大端):
        joint_index: uint8
        valid:       uint8 (1 有效)
        target_deg:  float32
        actual_deg:  float32
        loop1_out:   float32
        loop2_act:   float32
        loop2_out:   float32
    """
    expected_len = 2 + 5 * 4
    if len(payload) != expected_len:
        return None
    joint_index, valid, target_deg, actual_deg, loop1_out, loop2_act, loop2_out = struct.unpack(
        ">BBfffff", payload
    )
    return joint_index, valid == 1, target_deg, actual_deg, loop1_out, loop2_act, loop2_out


def parse_frame(data: bytes) -> Tuple[List[Tuple[int, bytes]], bytearray]:
    """从字节流解析完整帧，返回 ([(type, payload), ...], remaining_buffer)"""
    result = []
    buffer = bytearray(data)
    while len(buffer) >= 4:
        try:
            head_idx = buffer.index(PROTOCOL_HEADER)
        except ValueError:
            break
        buffer = buffer[head_idx:]
        if len(buffer) < 4:
            break
        payload_len = buffer[1]
        frame_len = 2 + payload_len
        if len(buffer) < frame_len:
            break
        frame = buffer[:frame_len]
        if frame[-1] != PROTOCOL_TAIL:
            buffer = buffer[1:]
            continue
        pkt_type = frame[2]
        payload = bytes(frame[3:-1])
        result.append((pkt_type, payload))
        buffer = buffer[frame_len:]
    return result, buffer


def build_calibrate_cmd() -> bytes:
    """构建校准指令"""
    return bytes([CMD_CALIBRATE])


def build_start_cmd() -> bytes:
    """构建启动指令"""
    return bytes([CMD_START])


def build_stop_cmd() -> bytes:
    """构建停止指令"""
    return bytes([CMD_STOP])


def build_reset_cmd() -> bytes:
    """构建重置指令"""
    return bytes([CMD_RESET])


def build_calib_data_cmd(zero_encoder_raw: List[int]) -> bytes:
    """构建校准数据指令: 0xCF + MOTOR_COUNT×float(4B LE)，关节 0° 时编码器原始值"""
    if len(zero_encoder_raw) < MOTOR_COUNT:
        raise ValueError(f"需要 {MOTOR_COUNT} 个编码器原始值")
    vals = [float(v) for v in zero_encoder_raw[:MOTOR_COUNT]]
    fmt = f"<{MOTOR_COUNT}f"
    payload = struct.pack(fmt, *vals)
    return bytes([CMD_CALIB_DATA]) + payload


def build_angle_cmd(angles: List[float]) -> bytes:
    """构建角度控制指令: 0xCB + 21×float(4B LE)，MOTOR_COUNT 个电机一起发送"""
    if len(angles) != MOTOR_COUNT:
        raise ValueError(f"需要 {MOTOR_COUNT} 个角度值")
    fmt = f"<{MOTOR_COUNT}f"
    payload = struct.pack(fmt, *angles[:MOTOR_COUNT])
    return bytes([CMD_ANGLE_CTRL]) + payload


def build_motor_pos_cmd(motor_pos_raw: List[int]) -> bytes:
    """构建直接电机位置控制指令: 0xD0 + 21×2 字节大端，MOTOR_COUNT 个电机目标编码器原始值"""
    if len(motor_pos_raw) < MOTOR_COUNT:
        raise ValueError(f"需要 {MOTOR_COUNT} 个编码器原始值")
    payload = bytearray()
    for i in range(MOTOR_COUNT):
        v = int(motor_pos_raw[i]) & 0xFFFF
        payload.append((v >> 8) & 0xFF)
        payload.append(v & 0xFF)
    return bytes([CMD_MOTOR_POS]) + bytes(payload)
