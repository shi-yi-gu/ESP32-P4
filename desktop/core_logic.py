# core_logic.py - 灵巧手控制器核心逻辑

import threading
import queue
from typing import List, Optional

from comm_layer import LowerComputerComm, list_ports
from data_models import HandModel
from hand_geometry import load_calib_zero_raw, has_calib_file
from protocol import MOTOR_COUNT, ControlMode

ANGLE_LIMITS: List[tuple] = [(0.0, 90.0)] * MOTOR_COUNT


class HandController:
    """灵巧手控制器（HandModel：MOTOR_COUNT 个电机，无 start_idx/end_idx）"""

    def __init__(self):
        self.comm: Optional[LowerComputerComm] = None
        self.current_state = HandModel()
        if not has_calib_file():
            self.current_state.calib_status = "NO_CALIB"
        self.state_lock = threading.Lock()
        self.running = False
        self.update_callbacks: List[callable] = []
        self._pid_enabled = False
        self._paused = False
        self._control_mode = ControlMode.JOINT_ANGLE

    def set_control_mode(self, mode: ControlMode):
        """设置电机控制模式：JOINT_ANGLE 或 DIRECT_MOTOR"""
        self._control_mode = mode

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def set_pid_control(self, enabled: bool):
        self._pid_enabled = enabled

    def pause(self):
        """暂停：停止向下位机发送控制指令（数据接收照常）"""
        self._paused = True

    def resume(self):
        """恢复：重新允许发送控制指令"""
        self._paused = False

    def is_paused(self) -> bool:
        return self._paused

    def toggle_pause(self) -> bool:
        """切换暂停状态，返回当前是否已暂停"""
        self._paused = not self._paused
        return self._paused

    def initialize(self, comm_port: str) -> bool:
        try:
            self.comm = LowerComputerComm(comm_port)
        except (ValueError, RuntimeError) as e:
            print(f"连接失败: {e}")
            return False
        if not self.comm.connect():
            return False
        self.running = True
        threading.Thread(target=self._update_loop, daemon=True).start()
        # 成功连接后，若有 calib_deg.json 则传输给下位机并写入本地编码器零点；否则标记为未校准
        zero_raw = load_calib_zero_raw()
        if zero_raw:
            self.comm.send_command(("calib_data", zero_raw))
            with self.state_lock:
                self.current_state.set_encoder_zeros(zero_raw)
        else:
            with self.state_lock:
                self.current_state.calib_status = "NO_CALIB"
        return True

    def shutdown(self):
        self.running = False
        if self.comm:
            self.comm.disconnect()

    def _update_loop(self):
        while self.running:
            try:
                new_model = self.comm.rx_queue.get(timeout=0.1)
                with self.state_lock:
                    self.current_state.copy_state_from(new_model)
                    self.current_state.calib_status = new_model.calib_status
                    self.current_state.tactile_data = new_model.tactile_data
                    self.current_state.timestamp = new_model.timestamp
                for cb in self.update_callbacks:
                    try:
                        cb(self.current_state)
                    except Exception:
                        pass
            except queue.Empty:
                continue

    def set_target_angles(self, angles: List[float]):
        """设置全部 MOTOR_COUNT 个电机的目标关节角度（度），JOINT_ANGLE 模式；暂停时不发送"""
        if self._paused:
            return
        if len(angles) != MOTOR_COUNT:
            raise ValueError(f"需要 {MOTOR_COUNT} 个角度值")
        angles = self._validate_angles(angles)
        with self.state_lock:
            self.current_state.set_target_angles(angles)
        if self.comm and self._control_mode == ControlMode.JOINT_ANGLE:
            self.comm.send_command(self.current_state)

    def set_motor_positions_raw(self, motor_pos_raw: List[int]):
        """设置全部 MOTOR_COUNT 个电机的目标编码器原始值，DIRECT_MOTOR 模式；暂停时不发送"""
        if self._paused:
            return
        if len(motor_pos_raw) < MOTOR_COUNT:
            raise ValueError(f"需要 {MOTOR_COUNT} 个编码器原始值")
        raw = [int(v) & 0xFFFF for v in motor_pos_raw[:MOTOR_COUNT]]
        if self.comm and self._control_mode == ControlMode.DIRECT_MOTOR:
            self.comm.send_command(("motor_pos", raw))

    def _validate_angles(self, angles: List[float]) -> List[float]:
        out = []
        for i, a in enumerate(angles):
            if i < len(ANGLE_LIMITS):
                lo, hi = ANGLE_LIMITS[i]
                out.append(max(lo, min(hi, float(a))))
            else:
                out.append(float(a))
        return out

    def calibrate(self, progress_cb=None, done_cb=None):
        """
        执行上位机主导的校准：逐电机寻找 0° 点，保存至 calib_deg.json 并发送给下位机。
        在后台线程中运行，progress_cb(current, total, msg)、done_cb(success, zero_raw_or_none)。
        """
        if self._paused:
            return
        if not self.comm:
            if done_cb:
                done_cb(False, None)
            return

        def _run():
            from calibration import run_calibration
            result = run_calibration(self, progress_cb=progress_cb)
            if done_cb:
                done_cb(result is not None, result)

        threading.Thread(target=_run, daemon=True).start()

    def start(self):
        if self._paused:
            return
        self.comm and self.comm.send_command("start")

    def stop(self):
        """停止：始终允许发送（暂停时也可紧急停止）"""
        self.comm and self.comm.send_command("stop")

    def reset(self):
        """发送重置指令给下位机；暂停时不发送"""
        if self._paused:
            return
        self.comm and self.comm.send_command("reset")

    def register_update_callback(self, callback):
        self.update_callbacks.append(callback)
