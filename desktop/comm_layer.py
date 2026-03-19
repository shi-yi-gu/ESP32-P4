# comm_layer.py - 下位机串口通信

import serial
import serial.tools.list_ports
import struct
import threading
import queue
import time
import os
from typing import Optional, List, Union

from protocol import (
    parse_frame,
    parse_sensor_packet,
    parse_calib_ack,
    parse_servo_angle_packet,
    parse_servo_telem_packet,
    parse_joint_debug_packet,
    build_calibrate_cmd,
    build_calib_data_cmd,
    build_motor_pos_cmd,
    build_start_cmd,
    build_stop_cmd,
    build_reset_cmd,
    build_angle_cmd,
    ENCODER_COUNT,
    MOTOR_COUNT,
    BAUDRATE,
    SERIAL_TIMEOUT,
    RX_QUEUE_MAXSIZE,
    TX_QUEUE_MAXSIZE,
    RX_POLL_SLEEP,
    TX_QUEUE_GET_TIMEOUT,
)
from data_models import HandModel, JointEncoder, MotorSlot, MotorState

# 发送队列项：命令字符串 / HandModel / ("calib_data", List[int])
SendCmd = Union[str, HandModel, tuple]


def list_ports() -> List[tuple]:
    return [(p.device, p.description) for p in serial.tools.list_ports.comports()]


def is_valid_port(port: str) -> bool:
    """检查 port 是否在当前可用串口列表中"""
    if not port or not port.strip():
        return False
    available = [p[0] for p in list_ports()]
    return port.strip() in available


def choose_serial_port_interactive() -> Optional[str]:
    """
    Interactive serial-port selector for console/development mode.
    Returns selected port path, or None if user exits.
    """
    while True:
        os.system("cls" if os.name == "nt" else "clear")
        print("Scanning serial ports...")
        ports = list_ports()
        if not ports:
            print("No serial port detected.")
            print("Press Enter to retry, or Ctrl+C to exit.")
            try:
                input()
            except KeyboardInterrupt:
                return None
            continue

        print("\nAvailable ports:")
        for idx, (device, desc) in enumerate(ports):
            print(f"  [{idx}] {device} ({desc})")

        try:
            user_input = input(
                f"\nSelect index [0-{len(ports) - 1}] or press Enter for [0]: "
            ).strip()
            if user_input == "":
                return ports[0][0]
            index = int(user_input)
            if 0 <= index < len(ports):
                return ports[index][0]
            print("Invalid index.")
            time.sleep(1.0)
        except ValueError:
            print("Please input a number.")
            time.sleep(1.0)
        except KeyboardInterrupt:
            return None


class LowerComputerComm:
    """下位机通信 - 协议 [0xFE][LEN][TYPE][PAYLOAD][0xFF]"""

    def __init__(self, port: str, baudrate: int = BAUDRATE):
        self.port = (port or "").strip()
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.rx_queue: queue.Queue = queue.Queue(maxsize=RX_QUEUE_MAXSIZE)
        self.tx_queue: queue.Queue = queue.Queue(maxsize=TX_QUEUE_MAXSIZE)
        self.running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None
        self._rx_buffer = bytearray()
        self._last_model: Optional[HandModel] = None

        if self.port:
            if not is_valid_port(self.port):
                raise ValueError(f"端口 '{self.port}' 不在当前可用串口列表中，请检查设备是否连接")
            if not self.connect():
                raise RuntimeError(f"无法打开端口 '{self.port}'，请检查是否被占用或权限不足")

    def connect(self) -> bool:
        if self.serial and getattr(self.serial, "is_open", False):
            return True
        if not self.port:
            return False
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=SERIAL_TIMEOUT)
            self.running = True
            self._rx_buffer = bytearray()
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
            self._rx_thread.start()
            self._tx_thread.start()
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.serial = None

    def _rx_loop(self):
        while self.running and self.serial and self.serial.is_open:
            try:
                if self.serial.in_waiting:
                    chunk = self.serial.read(self.serial.in_waiting)
                    self._rx_buffer.extend(chunk)
                    packets, self._rx_buffer = parse_frame(bytes(self._rx_buffer))
                    model = self._process_packets(packets)
                    if model:
                        try:
                            self.rx_queue.put_nowait(model)
                        except queue.Full:
                            pass
                else:
                    time.sleep(RX_POLL_SLEEP)
            except Exception as e:
                print(f"RX 错误: {e}")

    def _process_packets(self, packets: List[tuple]) -> Optional[HandModel]:
        model = HandModel()
        if self._last_model:
            # 继承上一帧状态，避免未更新字段在不同包类型间被清空
            for i, e in enumerate(model.encoders):
                if i < len(self._last_model.encoders):
                    e.encoder_id = self._last_model.encoders[i].encoder_id
                    e.raw = self._last_model.encoders[i].raw
                    e.error = self._last_model.encoders[i].error
                    e.target_angle_deg = self._last_model.encoders[i].target_angle_deg
                    e.zero_raw = self._last_model.encoders[i].zero_raw
            for i, m in enumerate(model.motors):
                if i < len(self._last_model.motors):
                    m.motor_id = self._last_model.motors[i].motor_id
                    m.error = self._last_model.motors[i].error
                    m.state = self._last_model.motors[i].state
            model.servo_angles = list(self._last_model.servo_angles)
            model.servo_online = list(self._last_model.servo_online)
            model.servo_speed = list(self._last_model.servo_speed)
            model.servo_load = list(self._last_model.servo_load)
            model.servo_voltage = list(self._last_model.servo_voltage)
            model.servo_temperature = list(self._last_model.servo_temperature)
            model.servo_telem_online = list(self._last_model.servo_telem_online)
            model.joint_debug_valid = list(self._last_model.joint_debug_valid)
            model.joint_debug_target_deg = list(self._last_model.joint_debug_target_deg)
            model.joint_debug_actual_deg = list(self._last_model.joint_debug_actual_deg)
            model.joint_debug_loop1_output = list(self._last_model.joint_debug_loop1_output)
            model.joint_debug_loop2_actual = list(self._last_model.joint_debug_loop2_actual)
            model.joint_debug_loop2_output = list(self._last_model.joint_debug_loop2_output)
            model.calib_status = self._last_model.calib_status
            model.has_sensor_data = bool(self._last_model.has_sensor_data)
            model.has_servo_angle_data = bool(self._last_model.has_servo_angle_data)
        model.timestamp = time.time()

        emitted = False
        for pkt_type, payload in packets:
            if pkt_type == 0x01:
                angles, errors = parse_sensor_packet(payload)
                if angles:
                    for i in range(min(ENCODER_COUNT, len(angles))):
                        if i < len(model.encoders):
                            model.encoders[i].encoder_id = i
                            model.encoders[i].raw = angles[i]
                            model.encoders[i].error = errors[i] if i < len(errors) else False
                        if i < len(model.motors):
                            model.motors[i].motor_id = i
                            model.motors[i].error = errors[i] if i < len(errors) else False
                            model.motors[i].state = (
                                MotorState.ERROR if model.motors[i].error else MotorState.RUNNING
                            )
                    model.has_sensor_data = True
                    emitted = True
            elif pkt_type == 0x02:
                code = parse_calib_ack(payload)
                if code == 1:
                    model.calib_status = "PENDING"
                elif code == 2:
                    model.calib_status = "SUCCESS"
                elif code == 3:
                    model.calib_status = "FAILED"
                emitted = True
            elif pkt_type == 0x03:
                parsed = parse_servo_angle_packet(payload)
                if parsed is not None:
                    angles, online_flags = parsed
                    for i in range(min(MOTOR_COUNT, len(angles))):
                        model.servo_angles[i] = int(angles[i])
                        model.servo_online[i] = bool(online_flags[i]) if i < len(online_flags) else False
                    model.has_servo_angle_data = True
                    emitted = True
            elif pkt_type == 0x05:
                parsed = parse_servo_telem_packet(payload)
                if parsed is not None:
                    speeds, loads, volts, temps, online = parsed
                    for i in range(min(MOTOR_COUNT, len(speeds))):
                        model.servo_speed[i] = int(speeds[i])
                        model.servo_load[i] = int(loads[i])
                        model.servo_voltage[i] = int(volts[i])
                        model.servo_temperature[i] = int(temps[i])
                        model.servo_telem_online[i] = bool(online[i]) if i < len(online) else False
                    emitted = True
            elif pkt_type == 0x04:
                parsed = parse_joint_debug_packet(payload)
                if parsed is not None:
                    (
                        joint_index,
                        valid,
                        target_deg,
                        actual_deg,
                        loop1_out,
                        loop2_act,
                        loop2_out,
                    ) = parsed
                    if 0 <= joint_index < ENCODER_COUNT:
                        model.joint_debug_valid[joint_index] = valid
                        model.joint_debug_target_deg[joint_index] = float(target_deg)
                        model.joint_debug_actual_deg[joint_index] = float(actual_deg)
                        model.joint_debug_loop1_output[joint_index] = float(loop1_out)
                        model.joint_debug_loop2_actual[joint_index] = float(loop2_act)
                        model.joint_debug_loop2_output[joint_index] = float(loop2_out)
                    emitted = True

        self._last_model = model
        return model if emitted else None

    def send_command(self, cmd: SendCmd):
        """发送：'calibrate'/'start'/'stop'/'reset' / ("calib_data", zero_raw_list) / ("motor_pos", raw_list) / HandModel"""
        try:
            self.tx_queue.put_nowait(cmd)
        except queue.Full:
            pass

    def _tx_loop(self):
        while self.running and self.serial and self.serial.is_open:
            try:
                cmd = self.tx_queue.get(timeout=TX_QUEUE_GET_TIMEOUT)
                data = self._encode_command(cmd)
                if data:
                    self.serial.write(data)
            except queue.Empty:
                pass
            except Exception as e:
                print(f"TX 错误: {e}")

    def _encode_command(self, cmd: SendCmd) -> Optional[bytes]:
        if cmd == "calibrate":
            return build_calibrate_cmd()
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "calib_data":
            return build_calib_data_cmd(cmd[1])
        if isinstance(cmd, tuple) and len(cmd) == 2 and cmd[0] == "motor_pos":
            return build_motor_pos_cmd(cmd[1])
        if cmd == "start":
            return build_start_cmd()
        if cmd == "stop":
            return build_stop_cmd()
        if cmd == "reset":
            return build_reset_cmd()
        if isinstance(cmd, HandModel):
            angles = cmd.target_angles
            if len(angles) < MOTOR_COUNT:
                angles = list(angles) + [0.0] * (MOTOR_COUNT - len(angles))
            return build_angle_cmd(angles[:MOTOR_COUNT])
        return None
