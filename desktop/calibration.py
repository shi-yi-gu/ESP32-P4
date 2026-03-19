# calibration.py - 上位机主导的关节角校准
# 校准逻辑：对每个电机，初始负载为 0，逐步控制转动直至负载显著增加，此时为关节 0°。
# 电机控制模式见 protocol.ControlMode

from typing import Optional, Callable, List

from protocol import MOTOR_COUNT, ControlMode, CALIB_LOAD_THRESHOLD
from hand_geometry import save_calib_zero_raw


def run_calibration(
    controller,
    progress_cb: Optional[Callable[[int, int, str], None]] = None,
    get_load: Optional[Callable[[int], float]] = None,
    control_mode: ControlMode = ControlMode.JOINT_ANGLE,
) -> Optional[List[int]]:
    """
    上位机校准流程：逐电机寻找 0° 点（负载开始显著增加处），记录编码器值并保存。

    Args:
        controller: HandController 实例
        progress_cb: 进度回调 (current_motor, total, message)
        get_load: 获取指定电机负载的回调 motor_id -> float，0~1 归一化。若为 None 则使用占位（恒为 0）
        control_mode: 控制模式，目前仅 JOINT_ANGLE 可用

    Returns:
        成功时返回 MOTOR_COUNT 个 zero_encoder_raw，失败返回 None

    说明：
        - 若协议尚无电机负载反馈，get_load 需由外部提供或下位机扩展 PACKET_TYPE_MOTOR_STATUS
        - 直接电机控制模式需协议新增 CMD_MOTOR_RAW
    """
    import time

    total = MOTOR_COUNT
    zero_raw: List[int] = []
    _get_load = get_load or (lambda _: 0.0)

    controller.start()
    time.sleep(0.2)

    try:
        for motor_id in range(total):
            if progress_cb:
                progress_cb(motor_id, total, f"校准电机 {motor_id}")

            # 仅移动当前电机，其余保持 0
            angles = [0.0] * MOTOR_COUNT
            step_deg = 2.0
            max_steps = 90
            found = False

            for step in range(max_steps):
                angles[motor_id] = step * step_deg
                controller.set_target_angles(angles)
                time.sleep(0.1)

                state = controller.current_state
                with controller.state_lock:
                    load = _get_load(motor_id)
                    enc = state.encoders[motor_id].raw if motor_id < len(state.encoders) else 0

                if load >= CALIB_LOAD_THRESHOLD:
                    zero_raw.append(enc)
                    found = True
                    break

            if not found:
                # 无负载反馈时，以当前编码器或 0 作为占位
                with controller.state_lock:
                    enc = controller.current_state.encoders[motor_id].raw if motor_id < len(controller.current_state.encoders) else 0
                zero_raw.append(enc)

        while len(zero_raw) < MOTOR_COUNT:
            zero_raw.append(0)

        if save_calib_zero_raw(zero_raw):
            if controller.comm:
                controller.comm.send_command(("calib_data", zero_raw))
            with controller.state_lock:
                controller.current_state.set_encoder_zeros(zero_raw[:MOTOR_COUNT])
                controller.current_state.calib_status = "SUCCESS"
            return zero_raw[:MOTOR_COUNT]
    except Exception:
        return None
    finally:
        controller.stop()

    return None
