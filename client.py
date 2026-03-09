import os
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

import serial
import serial.tools.list_ports

try:
    import msvcrt
except ImportError:
    msvcrt = None

try:
    from colorama import Back, Fore, Style, init

    init(autoreset=True)
except ImportError:
    class _DummyColor:
        def __getattr__(self, _name):
            return ""

    Fore = Back = Style = _DummyColor()


TARGET_BAUDRATE = 921600
ENCODER_COUNT = 21

PACKET_HEADER = 0xFE
PACKET_TAIL = 0xFF
PACKET_TYPE_SENSOR = 0x01
PACKET_TYPE_CALIB_ACK = 0x02
PACKET_TYPE_SERVO_ANGLE = 0x03
PACKET_TYPE_JOINT1_DEBUG = 0x04

CMD_CALIBRATE = b"\xCA"
DISCONNECT_SENTINEL = 0x7FFF


@dataclass
class JointDebugInfo:
    valid: bool = False
    target_deg: float = 0.0
    mag_actual_deg: float = 0.0
    loop1_output: float = 0.0
    loop2_actual: float = 0.0
    loop2_output: float = 0.0
    last_update: float = 0.0


@dataclass
class MachineState:
    mapped_counts: List[int] = field(default_factory=lambda: [0] * ENCODER_COUNT)
    mapped_disconnect: List[bool] = field(default_factory=lambda: [False] * ENCODER_COUNT)

    servo_angles: List[int] = field(default_factory=lambda: [0] * ENCODER_COUNT)
    servo_online: List[bool] = field(default_factory=lambda: [False] * ENCODER_COUNT)

    joint1_debug: JointDebugInfo = field(default_factory=JointDebugInfo)

    calib_status: str = "IDLE"
    calib_timestamp: float = 0.0

    last_update: float = 0.0
    connected_port: Optional[str] = None
    running: bool = True
    ser: Optional[serial.Serial] = None

    lock: threading.Lock = field(default_factory=threading.Lock)


state = MachineState()


def process_sensor_packet(payload: bytes) -> None:
    if len(payload) != ENCODER_COUNT * 2:
        return

    with state.lock:
        for i in range(ENCODER_COUNT):
            raw_u16 = (payload[i * 2] << 8) | payload[i * 2 + 1]
            if raw_u16 == DISCONNECT_SENTINEL:
                state.mapped_disconnect[i] = True
                state.mapped_counts[i] = 0
            else:
                state.mapped_disconnect[i] = False
                state.mapped_counts[i] = struct.unpack(">h", payload[i * 2:i * 2 + 2])[0]

        state.last_update = time.time()


def process_calib_ack_packet(payload: bytes) -> None:
    if len(payload) < 1:
        return

    code = payload[0]
    with state.lock:
        if code == 1:
            state.calib_status = "PENDING"
        elif code == 2:
            state.calib_status = "SUCCESS"
            state.calib_timestamp = time.time()
        elif code == 3:
            state.calib_status = "FAILED"
            state.calib_timestamp = time.time()


def process_servo_angle_packet(payload: bytes) -> None:
    expected_len = ENCODER_COUNT * 4 + ENCODER_COUNT
    if len(payload) != expected_len:
        return

    with state.lock:
        for i in range(ENCODER_COUNT):
            state.servo_angles[i] = struct.unpack(">i", payload[i * 4:i * 4 + 4])[0]

        online_offset = ENCODER_COUNT * 4
        for i in range(ENCODER_COUNT):
            state.servo_online[i] = payload[online_offset + i] == 1

        state.last_update = time.time()


def process_joint1_debug_packet(payload: bytes) -> None:
    expected_len = 1 + 5 * 4
    if len(payload) != expected_len:
        return

    valid, target_deg, mag_actual_deg, loop1_output, loop2_actual, loop2_output = struct.unpack(">Bfffff", payload)

    with state.lock:
        state.joint1_debug.valid = valid == 1
        state.joint1_debug.target_deg = target_deg
        state.joint1_debug.mag_actual_deg = mag_actual_deg
        state.joint1_debug.loop1_output = loop1_output
        state.joint1_debug.loop2_actual = loop2_actual
        state.joint1_debug.loop2_output = loop2_output
        state.joint1_debug.last_update = time.time()
        state.last_update = state.joint1_debug.last_update


def parse_stream(buffer: bytearray) -> bytearray:
    while len(buffer) >= 4:
        try:
            header_index = buffer.index(PACKET_HEADER)
        except ValueError:
            return bytearray()

        if header_index > 0:
            buffer = buffer[header_index:]

        if len(buffer) < 2:
            break

        payload_len = buffer[1]
        frame_len = payload_len + 2
        if len(buffer) < frame_len:
            break

        frame = buffer[:frame_len]
        buffer = buffer[frame_len:]

        if frame[-1] != PACKET_TAIL:
            continue

        packet_type = frame[2]
        payload = frame[3:-1]

        if packet_type == PACKET_TYPE_SENSOR:
            process_sensor_packet(payload)
        elif packet_type == PACKET_TYPE_CALIB_ACK:
            process_calib_ack_packet(payload)
        elif packet_type == PACKET_TYPE_SERVO_ANGLE:
            process_servo_angle_packet(payload)
        elif packet_type == PACKET_TYPE_JOINT1_DEBUG:
            process_joint1_debug_packet(payload)

    return buffer


def serial_thread(port_name: str) -> None:
    try:
        ser = serial.Serial(port_name, TARGET_BAUDRATE, timeout=0.1)
        with state.lock:
            state.ser = ser
            state.connected_port = port_name

        buffer = bytearray()
        while state.running:
            if ser.in_waiting:
                buffer.extend(ser.read(ser.in_waiting))
                buffer = parse_stream(buffer)
            else:
                time.sleep(0.005)
    except Exception as exc:
        with state.lock:
            state.connected_port = None
            state.running = False
        print(f"\nSerial error: {exc}")


def format_mapped_cell(index: int, mapped_count: int, disconnected: bool) -> str:
    if disconnected:
        return f"[{index:02d}] {Back.RED}{Fore.WHITE} DISCONNECT {Style.RESET_ALL}"

    deg = mapped_count * 360.0 / 16384.0
    color = Fore.CYAN if index % 2 == 0 else Fore.WHITE
    return f"[{index:02d}] {color}{mapped_count:6d} ({deg:7.2f} deg){Style.RESET_ALL}"


def format_servo_cell(index: int, angle: int, online: bool) -> str:
    if not online:
        return f"[{index:02d}] {Back.RED}{Fore.WHITE} OFFLINE {Style.RESET_ALL}"

    color = Fore.GREEN if index % 2 == 0 else Fore.YELLOW
    return f"[{index:02d}] {color}{angle:7d}{Style.RESET_ALL}"


def render_calib_status(calib_status: str, calib_timestamp: float) -> str:
    elapsed = time.time() - calib_timestamp

    if calib_status == "IDLE":
        return f"{Fore.WHITE}Waiting command...{Style.RESET_ALL}"
    if calib_status == "PENDING":
        return f"{Fore.YELLOW}Sending calibration request...{Style.RESET_ALL}"
    if calib_status == "SUCCESS":
        if elapsed < 5.0:
            return f"{Back.GREEN}{Fore.WHITE} Calibration SUCCESS {Style.RESET_ALL}"
        return ""
    if calib_status == "FAILED":
        if elapsed < 5.0:
            return f"{Back.RED}{Fore.WHITE} Calibration FAILED {Style.RESET_ALL}"
        return ""
    return ""


def print_ui() -> None:
    with state.lock:
        connected_port = state.connected_port
        last_update = state.last_update
        mapped_counts = list(state.mapped_counts)
        mapped_disconnect = list(state.mapped_disconnect)
        servo_angles = list(state.servo_angles)
        servo_online = list(state.servo_online)
        joint1_debug = JointDebugInfo(
            valid=state.joint1_debug.valid,
            target_deg=state.joint1_debug.target_deg,
            mag_actual_deg=state.joint1_debug.mag_actual_deg,
            loop1_output=state.joint1_debug.loop1_output,
            loop2_actual=state.joint1_debug.loop2_actual,
            loop2_output=state.joint1_debug.loop2_output,
            last_update=state.joint1_debug.last_update,
        )
        calib_status = state.calib_status
        calib_timestamp = state.calib_timestamp

    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()

    title = f"{Back.BLUE}{Fore.WHITE}  ServoBoard Monitor  {Style.RESET_ALL}"
    print(title)

    port_text = (
        f"{Fore.GREEN}Connected: {connected_port}{Style.RESET_ALL}"
        if connected_port
        else f"{Fore.RED}Disconnected{Style.RESET_ALL}"
    )

    latency_ms = (time.time() - last_update) * 1000.0 if last_update > 0 else 9999.0
    latency_color = Fore.GREEN if latency_ms < 100 else Fore.YELLOW if latency_ms < 500 else Fore.RED
    print(f"Status: {port_text} | Latency: {latency_color}{latency_ms:.0f} ms{Style.RESET_ALL}")
    print("-" * 88)

    print(f"{Style.BRIGHT}Mapped encoder data (count | degree):{Style.RESET_ALL}")
    rows = (ENCODER_COUNT + 2) // 3
    for row in range(rows):
        cells = []
        for col in range(3):
            idx = row + col * rows
            if idx < ENCODER_COUNT:
                cells.append(format_mapped_cell(idx, mapped_counts[idx], mapped_disconnect[idx]))
        print("   ".join(cells))

    print("-" * 88)

    print(f"{Style.BRIGHT}Servo absolute positions:{Style.RESET_ALL}")
    for row in range(rows):
        cells = []
        for col in range(3):
            idx = row + col * rows
            if idx < ENCODER_COUNT:
                cells.append(format_servo_cell(idx, servo_angles[idx], servo_online[idx]))
        print("   ".join(cells))

    print("-" * 88)

    debug_age_ms = (time.time() - joint1_debug.last_update) * 1000.0 if joint1_debug.last_update > 0 else 9999.0
    debug_status = "VALID" if joint1_debug.valid else "INVALID"
    debug_status_color = Fore.GREEN if joint1_debug.valid else Fore.RED

    print(f"{Style.BRIGHT}Joint-1 Closed-loop Debug (packet 0x04):{Style.RESET_ALL}")
    print(f"  data_status     : {debug_status_color}{debug_status}{Style.RESET_ALL}")
    print(f"  targetDegs      : {joint1_debug.target_deg:10.4f}")
    print(f"  magActualDegs   : {joint1_debug.mag_actual_deg:10.4f}")
    print(f"  _pids[1][0].Out : {joint1_debug.loop1_output:10.4f}")
    print(f"  loop2Actual     : {joint1_debug.loop2_actual:10.4f}")
    print(f"  _pids[1][1].Out : {joint1_debug.loop2_output:10.4f}")
    print(f"  debug_latency   : {debug_age_ms:10.1f} ms")

    print("-" * 88)
    print("Controls: press 'c' to send calibration command, press 'q' to quit.")

    status_line = render_calib_status(calib_status, calib_timestamp)
    print(f"Calibration: {status_line}")


def choose_serial_port() -> Optional[str]:
    while True:
        os.system("cls" if os.name == "nt" else "clear")
        print("Scanning serial ports...")

        ports = list(serial.tools.list_ports.comports())
        if not ports:
            print(f"{Fore.RED}No serial port detected.{Style.RESET_ALL}")
            print("Press Enter to retry, or Ctrl+C to exit.")
            try:
                input()
            except KeyboardInterrupt:
                return None
            continue

        print("\nAvailable ports:")
        for idx, port in enumerate(ports):
            print(f"  [{idx}] {port.device} ({port.description})")

        try:
            user_input = input(
                f"\nSelect index [0-{len(ports) - 1}] or press Enter for [0]: "
            ).strip()
            if user_input == "":
                return ports[0].device

            index = int(user_input)
            if 0 <= index < len(ports):
                return ports[index].device

            print(f"{Fore.YELLOW}Invalid index.{Style.RESET_ALL}")
            time.sleep(1)
        except ValueError:
            print(f"{Fore.YELLOW}Please input a number.{Style.RESET_ALL}")
            time.sleep(1)
        except KeyboardInterrupt:
            return None


def handle_keyboard_input() -> None:
    if os.name == "nt" and msvcrt is not None and msvcrt.kbhit():
        key = msvcrt.getch().lower()
        if key == b"q":
            state.running = False
        elif key == b"c":
            with state.lock:
                ser = state.ser
            if ser and ser.is_open:
                ser.write(CMD_CALIBRATE)
                with state.lock:
                    state.calib_status = "PENDING"
                    state.calib_timestamp = time.time()


def main() -> None:
    selected_port = choose_serial_port()
    if not selected_port:
        return

    print(f"Connecting to {selected_port} ...")
    worker = threading.Thread(target=serial_thread, args=(selected_port,), daemon=True)
    worker.start()

    try:
        while state.running:
            print_ui()
            handle_keyboard_input()
            time.sleep(0.06)
    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        with state.lock:
            if state.ser and state.ser.is_open:
                state.ser.close()
        print("\nMonitor exited.")


if __name__ == "__main__":
    main()
