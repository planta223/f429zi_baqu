import argparse
import os
import struct
import threading
import time
import sys

# ============================================================
# Dependency check
# ============================================================
try:
    import can
except ImportError:
    print("[ERROR] python-can 패키지를 찾을 수 없습니다.")
    print('설치:')
    print('  python -m pip install "python-can[gs-usb]"')
    sys.exit(1)

try:
    import usb.core
    import usb.util
except ImportError:
    print("[ERROR] PyUSB 모듈을 찾을 수 없습니다.")
    print('gs_usb 의존성을 포함하여 설치:')
    print('  python -m pip install "python-can[gs-usb]"')
    sys.exit(1)

try:
    import libusb_package
except ImportError:
    print("[ERROR] libusb-package module not found.")
    print("Install:")
    print("  python -m pip install libusb-package")
    sys.exit(1)


# PyUSB's default Windows lookup does not search inside libusb-package.
# Add the bundled DLL directory for gs-usb, and use its backend for scanning.
LIBUSB_DLL_PATH = libusb_package.get_library_path()

if LIBUSB_DLL_PATH is None:
    print("[ERROR] libusb-1.0 library not found in libusb-package.")
    sys.exit(1)

os.environ["PATH"] = (
    str(LIBUSB_DLL_PATH.parent)
    + os.pathsep
    + os.environ.get("PATH", "")
)

LIBUSB_BACKEND = libusb_package.get_libusb1_backend()

if LIBUSB_BACKEND is None:
    print(f"[ERROR] failed to load libusb backend: {LIBUSB_DLL_PATH}")
    sys.exit(1)


# ============================================================
# candleLight / gs_usb configuration
# ============================================================
# 현재 장치 관리자에서 확인된 candleLight USB CAN adapter:
#   VID = 0x1D50
#   PID = 0x606F
#
# Windows에서는 COM 포트가 생성되지 않는 것이 정상입니다.
# python-can의 gs_usb backend가 WinUSB를 통해 직접 접근합니다.
CANDLELIGHT_VID = 0x1D50
CANDLELIGHT_PID = 0x606F

DEFAULT_CHANNEL = 0
DEFAULT_BITRATE = 500_000


# ============================================================
# Steering CAN protocol
# ============================================================
CAN_ID_STEER_REQUEST = 0x100
CAN_ID_STEER_STATUS = 0x180

CAN_REQUEST_DLC = 3
CAN_STATUS_DLC = 5

CAN_REQUEST_STEER_SCALE = 100.0   # raw = deg * 100
CAN_STATUS_STEER_SCALE = 100.0    # raw = deg * 100

CAN_REQUEST_ESTOP_MASK = 0x80

CAN_STATUS_FLAG_REACHED = 1 << 0
CAN_STATUS_FLAG_CONTROL_ENABLED = 1 << 1
CAN_STATUS_FLAG_SVON_ENABLED = 1 << 2
CAN_STATUS_FLAG_MOTOR_OUTPUT_ACTIVE = 1 << 3
CAN_STATUS_FLAG_ENCODER_INITIALIZED = 1 << 4

STEER_MIN_DEG = -30.0
STEER_MAX_DEG = 30.0

TX_PERIOD_S = 0.02            # 50 Hz
STATUS_PRINT_PERIOD_S = 0.20  # status 자동 출력 최대 5 Hz


# ============================================================
# Global state
# ============================================================
bus = None
running = True

tx_enabled = False
target_deg = 0.0
auto_print_status = True

latest_status = None

latest_status_lock = threading.Lock()
tx_state_lock = threading.Lock()


# ============================================================
# Utility
# ============================================================
def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value


def deg_to_request_raw(deg):
    deg = clamp(float(deg), STEER_MIN_DEG, STEER_MAX_DEG)
    raw = int(round(deg * CAN_REQUEST_STEER_SCALE))
    return int(clamp(raw, -32768, 32767))


def status_raw_to_deg(raw):
    return float(raw) / CAN_STATUS_STEER_SCALE


def flags_to_text(flags):
    names = []

    if flags & CAN_STATUS_FLAG_REACHED:
        names.append("REACHED")
    if flags & CAN_STATUS_FLAG_CONTROL_ENABLED:
        names.append("CONTROL")
    if flags & CAN_STATUS_FLAG_SVON_ENABLED:
        names.append("SVON")
    if flags & CAN_STATUS_FLAG_MOTOR_OUTPUT_ACTIVE:
        names.append("MOTOR")
    if flags & CAN_STATUS_FLAG_ENCODER_INITIALIZED:
        names.append("ENCODER")

    return "|".join(names) if names else "-"


# ============================================================
# USB device discovery
# ============================================================
def find_candlelight_devices():
    """
    VID/PID가 현재 장치와 일치하는 USB 장치를 찾습니다.
    """
    devices = list(
        usb.core.find(
            find_all=True,
            idVendor=CANDLELIGHT_VID,
            idProduct=CANDLELIGHT_PID,
            backend=LIBUSB_BACKEND,
        )
        or []
    )
    return devices


def safe_usb_string(dev, index):
    if not index:
        return None

    try:
        return usb.util.get_string(dev, index)
    except Exception:
        return None


def print_usb_devices(devices):
    if not devices:
        print(
            f"[USB] candleLight device not found "
            f"(VID=0x{CANDLELIGHT_VID:04X}, PID=0x{CANDLELIGHT_PID:04X})"
        )
        return

    print(f"[USB] found {len(devices)} candleLight device(s)")

    for i, dev in enumerate(devices):
        product = safe_usb_string(dev, getattr(dev, "iProduct", 0))
        manufacturer = safe_usb_string(dev, getattr(dev, "iManufacturer", 0))
        serial = safe_usb_string(dev, getattr(dev, "iSerialNumber", 0))

        print(
            f"  [{i}] "
            f"VID=0x{dev.idVendor:04X} "
            f"PID=0x{dev.idProduct:04X} "
            f"bus={getattr(dev, 'bus', '?')} "
            f"address={getattr(dev, 'address', '?')}"
        )

        if manufacturer:
            print(f"      manufacturer = {manufacturer}")
        if product:
            print(f"      product      = {product}")
        if serial:
            print(f"      serial       = {serial}")

        # Release the PyUSB handle before python-can opens the same device.
        usb.util.dispose_resources(dev)


# ============================================================
# CAN bus open
# ============================================================
def open_bus(channel, bitrate):
    """
    candleLight / gs_usb 전용 bus open.

    channel은 gs_usb device index이며 0부터 시작합니다.
    COM 포트는 사용하지 않습니다.
    """
    return can.Bus(
        interface="gs_usb",
        channel=int(channel),
        bitrate=int(bitrate),
    )


# ============================================================
# TX packet functions
# ============================================================
def send_steer_once(deg):
    """
    Steering Request
      ID       = 0x100
      Standard = 11-bit
      DLC      = 3

      Byte 0~1 = steer raw, little-endian int16
      Byte 2   = flags
    """
    global bus

    raw = deg_to_request_raw(deg)

    data = struct.pack(
        "<hB",
        raw,
        0x00,
    )

    if len(data) != CAN_REQUEST_DLC:
        raise RuntimeError("Unexpected Steering Request DLC")

    msg = can.Message(
        arbitration_id=CAN_ID_STEER_REQUEST,
        is_extended_id=False,
        is_remote_frame=False,
        data=data,
    )

    bus.send(msg)
    return raw


def send_estop_once():
    """
    CAN E-Stop
      ID       = 0x100
      steer    = 0
      flags    = 0x80

    현재 firmware 정책:
      - E-Stop 수신 시 normal pending request 폐기
      - 이후 새 normal request가 들어오면 ESTOP 해제 / 정상 제어 복귀
    """
    global bus

    data = struct.pack(
        "<hB",
        0,
        CAN_REQUEST_ESTOP_MASK,
    )

    msg = can.Message(
        arbitration_id=CAN_ID_STEER_REQUEST,
        is_extended_id=False,
        is_remote_frame=False,
        data=data,
    )

    bus.send(msg)


# ============================================================
# Periodic TX thread
# ============================================================
def tx_loop():
    global running

    tx_count = 0

    while running:
        with tx_state_lock:
            enabled = tx_enabled
            deg = target_deg

        if enabled:
            try:
                raw = send_steer_once(deg)
                tx_count += 1

                if tx_count % 50 == 0:
                    print(
                        f"[TX] ID=0x{CAN_ID_STEER_REQUEST:03X} "
                        f"target={deg:+.2f} deg raw={raw}"
                    )

            except can.CanError as e:
                print(f"[CAN TX ERROR] {e}")
            except Exception as e:
                print(f"[TX ERROR] {e}")

        time.sleep(TX_PERIOD_S)


# ============================================================
# RX Status
# ============================================================
def decode_status(msg):
    if msg.is_extended_id:
        return None

    if msg.is_remote_frame:
        return None

    if msg.arbitration_id != CAN_ID_STEER_STATUS:
        return None

    if len(msg.data) != CAN_STATUS_DLC:
        print(
            f"[RX WARN] ID=0x{CAN_ID_STEER_STATUS:03X} "
            f"unexpected DLC={len(msg.data)}"
        )
        return None

    actual_raw, target_raw, flags = struct.unpack(
        "<hhB",
        bytes(msg.data),
    )

    return {
        "timestamp": time.time(),
        "actual_raw": actual_raw,
        "target_raw": target_raw,
        "actual_deg": status_raw_to_deg(actual_raw),
        "target_deg": status_raw_to_deg(target_raw),
        "flags": flags,
        "reached": bool(flags & CAN_STATUS_FLAG_REACHED),
        "control_enabled": bool(flags & CAN_STATUS_FLAG_CONTROL_ENABLED),
        "svon_enabled": bool(flags & CAN_STATUS_FLAG_SVON_ENABLED),
        "motor_output_active": bool(flags & CAN_STATUS_FLAG_MOTOR_OUTPUT_ACTIVE),
        "encoder_initialized": bool(flags & CAN_STATUS_FLAG_ENCODER_INITIALIZED),
    }


def print_status(status, prefix="[RX]"):
    if status is None:
        print(
            f"{prefix} no 0x{CAN_ID_STEER_STATUS:03X} "
            f"status received yet"
        )
        return

    print(
        f"{prefix} ID=0x{CAN_ID_STEER_STATUS:03X} "
        f"actual={status['actual_deg']:+.2f} deg "
        f"target={status['target_deg']:+.2f} deg "
        f"flags=0x{status['flags']:02X} "
        f"[{flags_to_text(status['flags'])}]"
    )


def rx_loop():
    global running
    global latest_status

    last_print_time = 0.0

    while running:
        try:
            msg = bus.recv(timeout=0.1)

        except can.CanError as e:
            print(f"[CAN RX ERROR] {e}")
            time.sleep(0.1)
            continue

        except Exception as e:
            print(f"[RX ERROR] {e}")
            time.sleep(0.1)
            continue

        if msg is None:
            continue

        status = decode_status(msg)

        if status is None:
            continue

        with latest_status_lock:
            latest_status = status

        now = time.monotonic()

        if (
            auto_print_status
            and (now - last_print_time) >= STATUS_PRINT_PERIOD_S
        ):
            print_status(status)
            last_print_time = now


# ============================================================
# CLI commands
# ============================================================
def set_periodic_target(deg):
    global tx_enabled
    global target_deg

    deg = clamp(
        float(deg),
        STEER_MIN_DEG,
        STEER_MAX_DEG,
    )

    with tx_state_lock:
        target_deg = deg
        tx_enabled = True

    print(
        f"[TX] periodic steering enabled: "
        f"{target_deg:+.2f} deg @ {1.0 / TX_PERIOD_S:.0f} Hz"
    )


def stop_periodic_tx():
    global tx_enabled

    with tx_state_lock:
        tx_enabled = False

    print("[TX] periodic steering stopped")


def command_estop():
    global tx_enabled

    # periodic normal frame이 E-Stop 직후 전송되면
    # firmware 정책상 ESTOP이 다시 해제될 수 있으므로
    # 먼저 normal periodic TX부터 정지합니다.
    with tx_state_lock:
        tx_enabled = False

    send_estop_once()

    print("[TX] CAN E-Stop sent. Periodic steering is stopped.")
    print(
        "[INFO] 이후 's <deg>'를 입력하면 새 normal request가 전송됩니다."
    )


def show_latest_status():
    with latest_status_lock:
        status = None if latest_status is None else dict(latest_status)

    print_status(status, prefix="[STATUS]")


def set_auto_print(value):
    global auto_print_status

    auto_print_status = bool(value)

    print(
        "[RX] automatic status print "
        + ("ON" if auto_print_status else "OFF")
    )


def print_help():
    print()
    print("commands:")
    print("  s <deg>       : steering target을 50 Hz로 지속 송신")
    print("                  예) s 10, s -5.5, s 0")
    print("  <deg>         : 숫자만 입력해도 s <deg>와 동일")
    print("  once <deg>    : steering request 1회 송신")
    print("  e             : CAN E-Stop 1회 송신 + periodic TX 정지")
    print("  idle          : periodic TX 정지 (timeout 시험용)")
    print("  status        : 가장 최근 0x180 Status 1회 출력")
    print("  rx on         : Status 자동 출력 ON")
    print("  rx off        : Status 자동 출력 OFF")
    print("  help          : 도움말")
    print("  q             : 종료")
    print()


# ============================================================
# Arguments
# ============================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "STM32 steering candleLight/gs_usb CAN debugger"
        )
    )

    parser.add_argument(
        "--channel",
        type=int,
        default=DEFAULT_CHANNEL,
        help="gs_usb device index, 0부터 시작 (default: 0)",
    )

    parser.add_argument(
        "--bitrate",
        type=int,
        default=DEFAULT_BITRATE,
        help=(
            "CAN bitrate. STM32 설정과 반드시 일치해야 함 "
            "(default: 500000)"
        ),
    )

    parser.add_argument(
        "--scan",
        action="store_true",
        help="USB candleLight 장치만 검색하고 종료",
    )

    return parser.parse_args()


# ============================================================
# Main
# ============================================================
def main():
    global bus
    global running
    global tx_enabled

    args = parse_args()

    print("STM32 candleLight / gs_usb steering debugger")
    print(
        f"USB VID/PID = "
        f"0x{CANDLELIGHT_VID:04X}/0x{CANDLELIGHT_PID:04X}"
    )
    print(f"channel     = {args.channel}")
    print(f"bitrate     = {args.bitrate}")
    print(f"TX ID       = 0x{CAN_ID_STEER_REQUEST:03X}")
    print(f"RX ID       = 0x{CAN_ID_STEER_STATUS:03X}")
    print()

    # 1. USB 장치 탐색
    try:
        devices = find_candlelight_devices()
    except Exception as e:
        print("[USB SCAN FAILED]")
        print(e)
        print()
        print('먼저 다음을 설치했는지 확인하십시오:')
        print('  python -m pip install "python-can[gs-usb]"')
        sys.exit(1)

    print_usb_devices(devices)
    print()

    if args.scan:
        if devices:
            print("[OK] candleLight USB device detected.")
            sys.exit(0)

        print("[FAIL] candleLight USB device was not detected.")
        sys.exit(1)

    if not devices:
        print("[CAN OPEN SKIPPED]")
        print("장치 관리자에서 다음을 확인하십시오:")
        print("  candleLight USB to CAN adapter")
        print("  VID_1D50 / PID_606F")
        print("  WinUSB")
        sys.exit(1)

    if args.channel < 0 or args.channel >= len(devices):
        print(
            f"[ERROR] channel {args.channel} is invalid. "
            f"Detected device count = {len(devices)}"
        )
        sys.exit(1)

    # 2. gs_usb bus open
    try:
        bus = open_bus(
            channel=args.channel,
            bitrate=args.bitrate,
        )

    except Exception as e:
        print("[CAN OPEN FAILED]")
        print(type(e).__name__ + ":", e)
        print()
        print("확인할 것:")
        print(
            "  1. python -m pip install "
            '"python-can[gs-usb]" 실행 여부'
        )
        print("  2. 장치 관리자에서 WinUSB / 오류 없음인지")
        print("  3. 다른 CAN 프로그램이 장치를 점유하고 있지 않은지")
        print("  4. --channel 값이 맞는지 (보통 0)")
        print("  5. STM32와 bitrate가 동일한지")
        sys.exit(1)

    print("[CAN] gs_usb bus opened")
    print()

    # 3. TX/RX threads
    tx_thread = threading.Thread(
        target=tx_loop,
        daemon=True,
        name="can-tx",
    )

    rx_thread = threading.Thread(
        target=rx_loop,
        daemon=True,
        name="can-rx",
    )

    tx_thread.start()
    rx_thread.start()

    print_help()

    try:
        while True:
            cmd = input("can> ").strip()

            if not cmd:
                continue

            lower = cmd.lower()

            if lower == "q":
                break

            if lower in ("help", "h", "?"):
                print_help()
                continue

            if lower == "idle":
                stop_periodic_tx()
                continue

            if lower in ("e", "estop"):
                try:
                    command_estop()
                except can.CanError as e:
                    print(f"[CAN TX ERROR] {e}")
                continue

            if lower == "status":
                show_latest_status()
                continue

            if lower == "rx on":
                set_auto_print(True)
                continue

            if lower == "rx off":
                set_auto_print(False)
                continue

            if lower.startswith("once "):
                try:
                    deg = float(cmd.split(maxsplit=1)[1])
                    deg = clamp(
                        deg,
                        STEER_MIN_DEG,
                        STEER_MAX_DEG,
                    )

                    raw = send_steer_once(deg)

                    print(
                        f"[TX ONCE] "
                        f"target={deg:+.2f} deg raw={raw}"
                    )

                except ValueError:
                    print("invalid degree")
                except can.CanError as e:
                    print(f"[CAN TX ERROR] {e}")

                continue

            if lower.startswith("s "):
                try:
                    deg = float(cmd.split(maxsplit=1)[1])
                    set_periodic_target(deg)

                except ValueError:
                    print("invalid degree")

                continue

            # 숫자만 입력해도 periodic steering target
            try:
                deg = float(cmd)
                set_periodic_target(deg)
                continue

            except ValueError:
                pass

            print("invalid command. 'help'를 입력하십시오.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt")

    finally:
        running = False

        with tx_state_lock:
            tx_enabled = False

        time.sleep(0.15)

        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                pass

        print("closed")


if __name__ == "__main__":
    main()
