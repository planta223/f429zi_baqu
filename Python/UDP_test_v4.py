"""STM32 steering UDP tester; standard library only.

Run: python Python/UDP_test_v4.py
Current parameters are read from Core/Inc/config.h. If it is unavailable,
the bundled defaults match the 2026-10-22 steering configuration.
UDP_status_rx.py can run in another console; this sender never binds :5002.

PC: 9 bytes, <ffB (float32 steering degrees, float32 speed=0, uint8 misc).
ASMS: unchanged 5 bytes, <BHh (mode, speed=0, int16 joystick raw).
PC float32 requires matching firmware; UDP_test_v3.py uses the old int32 format.
The PC misc brake bit is not a steering E-Stop; ASMS mode 3 is the E-Stop path.

The TX worker never prints or writes files. A separate reporter records
successful send counts, send age, maximum periodic gap and timeout-sized gaps.
These are PC send statistics, not acknowledgements from the STM32.
"""

import argparse
import csv
from dataclasses import dataclass, fields
import locale
import math
from pathlib import Path
import re
import socket
import struct
import subprocess
import threading
import time


STM32_IP = "10.177.21.4"
PC_SOURCE_IP = "10.177.21.1"
ASMS_SOURCE_IP = "10.177.21.5"
ETH_IF_NAME = "이더넷"
ETHERNET_MASK = "255.255.255.0"
TX_PERIOD_S = 0.02
SOCKET_TIMEOUT_S = 0.05
STEER_MODE_AUTO = 1
STEER_MODE_MANUAL = 2
STEER_MODE_ESTOP = 3
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "Core/Inc/config.h"


@dataclass(frozen=True)
class FirmwareParameters:
    steering_min_deg: float = -17.01
    steering_max_deg: float = 17.01
    pc_scale: float = 1.0
    pc_polarity: int = 1
    asms_min: int = -2048
    asms_center: int = 0
    asms_max: int = 2047
    asms_deadband: int = 50
    asms_scale: float = 17.01
    asms_polarity: int = -1
    timeout_ms: int = 300
    udp_port: int = 5000
    status_port: int = 5002
    homing_enabled: int = 1
    boot_count: int = 78749
    reached_band_deg: float = 0.070875
    gear_ratio: float = 125.92635
    encoder_counts_per_rev: float = 12000.0
    kp: float = 400.0
    homing_return_time_ms: int = 5000


CONFIG_MACROS = dict(zip(
    (field.name for field in fields(FirmwareParameters)),
    ("CONTROL_TARGET_MIN_STEERING_DEG", "CONTROL_TARGET_MAX_STEERING_DEG",
     "PC_STEER_SCALE", "PC_STEER_POLARITY", "ASMS_ADC_MIN_RAW",
     "ASMS_ADC_CENTER_RAW", "ASMS_ADC_MAX_RAW", "ASMS_ADC_DEADBAND_RAW",
     "ASMS_STEER_SCALE", "ASMS_STEER_POLARITY", "ETHERNET_TIMEOUT_MS",
     "ETHERNET_UDP_PORT", "ETHERNET_STATUS_DEST_PORT",
     "STEERING_BOOT_CENTERING_ENABLE", "ENCODER_BOOT_INITIAL_COUNT",
     "CONTROL_REACHED_BAND_STEERING_DEG", "STEERING_GEAR_RATIO",
     "ENCODER_COUNT_PER_MOTOR_REV", "CONTROL_DEFAULT_KP",
     "HOMING_RETURN_TIME_MS")))


def load_parameters(path):
    defaults = FirmwareParameters()
    if not path.is_file():
        return defaults, "bundled defaults (config.h not found)"
    text = re.sub(r"/\*.*?\*/", "", path.read_text(encoding="utf-8-sig"),
                  flags=re.DOTALL)
    values = {}
    for name, macro in CONFIG_MACROS.items():
        match = re.search(r"^\s*#define\s+" + macro + r"\s+([^\r\n]+)",
                          text, re.MULTILINE)
        if match is None:
            raise ValueError(f"Missing firmware parameter: {macro}")
        literal = match[1].split("//", 1)[0].strip().strip("() ")
        literal = re.sub(r"[fFuUlL]+$", "", literal)
        number = float(literal)
        if not math.isfinite(number):
            raise ValueError(f"Non-finite firmware parameter: {macro}")
        if isinstance(getattr(defaults, name), int):
            if not number.is_integer():
                raise ValueError(f"Expected integer firmware parameter: {macro}")
            number = int(number)
        values[name] = number
    params = FirmwareParameters(**values)
    if not (params.steering_min_deg < 0 < params.steering_max_deg and
            params.pc_scale > 0 and params.pc_polarity in (-1, 1) and
            -32768 <= params.asms_min < params.asms_center < params.asms_max <= 32767 and
            params.asms_deadband >= 0 and params.asms_scale > 0 and
            params.asms_polarity in (-1, 1) and params.timeout_ms > 0 and
            1 <= params.udp_port <= 65535 and 1 <= params.status_port <= 65535 and
            params.homing_enabled in (0, 1) and params.reached_band_deg > 0 and
            params.gear_ratio > 0 and params.encoder_counts_per_rev > 0 and
            params.homing_return_time_ms > 0):
        raise ValueError("Unsupported or invalid firmware parameters")
    return params, str(path)


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def pc_packet(params, degrees, estop=False):
    if not math.isfinite(degrees):
        raise ValueError("Steering angle must be finite")
    requested = clamp(degrees, params.steering_min_deg, params.steering_max_deg)
    # Apply the configured polarity/scale, then transmit IEEE-754 float32.
    unit = params.pc_scale * params.pc_polarity
    raw = requested / unit
    try:
        packet = struct.pack("<ffB", raw, 0.0, 0x80 if estop else 0)
    except (OverflowError, struct.error) as exc:
        raise ValueError("Steering value cannot be represented as float32") from exc
    wire_raw = struct.unpack_from("<f", packet)[0]
    return packet, wire_raw * unit


def asms_packet(params, mode, raw):
    raw = clamp(int(raw), params.asms_min, params.asms_max)
    return struct.pack("<BHh", mode, 0, raw)


def asms_degrees(params, raw):
    offset = clamp(int(raw), params.asms_min, params.asms_max) - params.asms_center
    if abs(offset) <= params.asms_deadband:
        return 0.0
    span = (params.asms_max - params.asms_center if offset >= 0 else
            params.asms_center - params.asms_min)
    return clamp(params.asms_polarity * params.asms_scale * offset / span,
                 params.steering_min_deg, params.steering_max_deg)


class SteeringTransmitter:
    """Serialize state changes and sends; never do console/file I/O under lock."""

    def __init__(self, params, pc_sock, asms_sock):
        self.params = params
        self.pc_sock = pc_sock
        self.asms_sock = asms_sock
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.mode = "idle"
        self.target_deg = 0.0
        self.asms_raw = params.asms_center
        self.last_periodic = None
        self.stats = dict(sent=0, pc_sent=0, asms_sent=0, periodic_sent=0,
                          one_shot_sent=0, errors=0, last_error="",
                          last_send_epoch=None, last_send_monotonic=None,
                          last_gap_ms=None, max_gap_ms=0.0, timeout_gaps=0)

    def start(self):
        self.thread = threading.Thread(target=self._run, name="udp-tx", daemon=True)
        self.thread.start()

    def set_mode(self, mode):
        if mode not in ("idle", "pc", "asms"):
            raise ValueError("Invalid TX mode")
        with self.lock:
            if mode == "idle" or self.mode == "idle":
                # Intentional idle time is not a missed periodic deadline.
                self.last_periodic = None
            self.mode = mode

    def set_target(self, degrees):
        _, applied = pc_packet(self.params, degrees)
        with self.lock:
            self.target_deg = applied
        return applied

    def set_asms(self, raw):
        with self.lock:
            self.asms_raw = clamp(int(raw), self.params.asms_min, self.params.asms_max)
            return self.asms_raw

    def zero(self, source):
        if source not in ("pc", "asms"):
            raise ValueError("Invalid homing source")
        with self.lock:
            if self.mode == "idle":
                self.last_periodic = None
            self.target_deg = 0.0
            self.asms_raw = self.params.asms_center
            self.mode = source

    def _send_locked(self, source, packet, periodic):
        sock = self.pc_sock if source == "pc" else self.asms_sock
        try:
            size = sock.sendto(packet, (STM32_IP, self.params.udp_port))
            if size != len(packet):
                raise OSError("Incomplete UDP datagram send")
        except OSError as exc:
            self.stats["errors"] += 1
            self.stats["last_error"] = str(exc)
            return False
        now = time.monotonic()
        self.stats["sent"] += 1
        self.stats[source + "_sent"] += 1
        self.stats["last_send_epoch"] = time.time()
        self.stats["last_send_monotonic"] = now
        if periodic:
            self.stats["periodic_sent"] += 1
            if self.last_periodic is not None:
                gap = (now - self.last_periodic) * 1000.0
                self.stats["last_gap_ms"] = gap
                self.stats["max_gap_ms"] = max(self.stats["max_gap_ms"], gap)
                if gap > self.params.timeout_ms:
                    self.stats["timeout_gaps"] += 1
            self.last_periodic = now
        else:
            self.stats["one_shot_sent"] += 1
        return True

    def one_shot(self, source, estop=False):
        """Stop periodic TX and finish any in-flight send before the one-shot."""
        if source not in ("pc", "asms"):
            raise ValueError("Invalid command source")
        with self.lock:
            self.mode = "idle"
            self.last_periodic = None
            if source == "pc":
                packet, _ = pc_packet(self.params, self.target_deg, estop=estop)
            else:
                packet = asms_packet(self.params,
                                     STEER_MODE_ESTOP if estop else STEER_MODE_AUTO,
                                     self.params.asms_center)
            return self._send_locked(source, packet, periodic=False)

    def snapshot(self):
        with self.lock:
            result = dict(self.stats, mode=self.mode, target_deg=self.target_deg,
                          asms_raw=self.asms_raw)
        last = result.pop("last_send_monotonic")
        result["send_age_ms"] = None if last is None else (time.monotonic() - last) * 1000.0
        result["worker_alive"] = self.thread is not None and self.thread.is_alive()
        return result

    def _run(self):
        deadline = time.monotonic()
        while not self.stop_event.is_set():
            with self.lock:
                if self.mode == "pc":
                    packet, _ = pc_packet(self.params, self.target_deg)
                    self._send_locked("pc", packet, periodic=True)
                elif self.mode == "asms":
                    packet = asms_packet(self.params, STEER_MODE_MANUAL, self.asms_raw)
                    self._send_locked("asms", packet, periodic=True)
            deadline += TX_PERIOD_S
            now = time.monotonic()
            if deadline <= now:
                deadline = now + TX_PERIOD_S  # No catch-up bursts after a stall.
            self.stop_event.wait(max(0.0, deadline - now))

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
            if self.thread.is_alive():
                raise RuntimeError("TX worker did not stop; check socket timeout")
        self.set_mode("idle")


def status_text(state):
    age = "--" if state["send_age_ms"] is None else f'{state["send_age_ms"]:.0f}'
    gap = "--" if state["last_gap_ms"] is None else f'{state["last_gap_ms"]:.1f}'
    return (f'[TX] mode={state["mode"]} target={state["target_deg"]:g} deg '
            f'asms_raw={state["asms_raw"]} sent={state["sent"]} '
            f'age={age}ms gap={gap}ms max_gap={state["max_gap_ms"]:.1f}ms '
            f'timeout_gaps={state["timeout_gaps"]} errors={state["errors"]} '
            f'worker_alive={state["worker_alive"]}' +
            (f' last_error={state["last_error"]}' if state["last_error"] else ""))


def report_loop(tx, log_path, console=True):
    """A blocked console or disk writer never holds the TX lock."""
    handle = None
    writer = None
    file_error = ""
    try:
        if log_path is not None:
            try:
                handle = log_path.open("w", encoding="utf-8", newline="")
                writer = csv.DictWriter(handle,
                                        fieldnames=["pc_time", *tx.snapshot().keys()])
                writer.writeheader()
                handle.flush()
            except OSError as exc:
                file_error = f"[LOG ERROR] {exc}"
        while not tx.stop_event.wait(1.0):
            state = tx.snapshot()
            if writer is not None:
                try:
                    writer.writerow(dict(pc_time=time.time(), **state))
                    handle.flush()
                except OSError as exc:
                    file_error = f"[LOG ERROR] {exc}"
                    writer = None
            if console:
                try:
                    if file_error:
                        print(file_error, flush=True)
                        file_error = ""
                    print(status_text(state), flush=True)
                except (OSError, ValueError):
                    console = False
    finally:
        if handle is not None:
            handle.close()


def can_bind(ip):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind((ip, 0))
            return True
        except OSError:
            return False


def netsh(action, ip):
    args = ["netsh", "interface", "ipv4", action, "address",
            f"name={ETH_IF_NAME}", f"address={ip}"]
    if action == "add":
        args += [f"mask={ETHERNET_MASK}", "store=active"]
    result = subprocess.run(args, capture_output=True,
                            encoding=locale.getpreferredencoding(False), errors="replace")
    output = result.stdout + result.stderr
    if output.strip():
        print(output.strip())
    return result.returncode, output


def ensure_source_ip(ip, added_ips):
    if can_bind(ip):
        print(f"[IP READY] {ip}; reusing existing address")
        return True
    code, output = netsh("add", ip)
    if code == 0:
        added_ips.add(ip)
    elif not any(word in output.lower() for word in
                 ("already exists", "object already exists", "개체가 이미 있습니다")):
        print(f"[IP FAILED] {ip}; check administrator rights and adapter {ETH_IF_NAME}")
        return False
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        if can_bind(ip):
            print(f"[IP READY] {ip}")
            return True
        time.sleep(0.1)
    print(f"[IP FAILED] {ip}; address is not bindable (DAD/duplicate address)")
    return False


def create_socket(ip):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.settimeout(SOCKET_TIMEOUT_S)
        sock.bind((ip, 0))
        return sock
    except OSError:
        sock.close()
        raise


def print_status(tx):
    print(status_text(tx.snapshot()))
    print("PC send success is not an STM32 ACK. Check UDP_status_rx.py / debugger.")


def steering_mode(tx, mode):
    tx.set_mode(mode)
    params = tx.params
    e_action = "PC brake flag" if mode == "pc" else "ASMS E-Stop"
    print(f"[{mode.upper()} MODE] 50 Hz; q=idle/back, e={e_action}, status=TX diagnostics")
    if mode == "pc":
        print(f"Target degrees: {params.steering_min_deg:g} .. {params.steering_max_deg:g}; "
              "wire=float32 degrees (no integer-degree rounding)")
        print("After homing, use explicit 'auto' before switching MANUAL/ESTOP -> PC.")
    else:
        print(f"ASMS raw: {params.asms_min} .. {params.asms_max}; "
              f"center={params.asms_center}, deadband=+/-{params.asms_deadband}")
    while True:
        value = input(f"{mode} target> ").strip().lower()
        if value in ("q", "idle"):
            tx.set_mode("idle")
            return
        if value == "e":
            ok = tx.one_shot(mode, estop=True)
            print(f"[{e_action.upper()}] {'sent once' if ok else 'send failed'}; periodic TX idle")
            return
        if value in ("status", "s"):
            print_status(tx)
            continue
        try:
            if mode == "pc":
                requested = float(value)
                applied = tx.set_target(requested)
                print(f"PC target={applied:g} deg (requested {requested:g})")
            else:
                raw = tx.set_asms(int(value))
                print(f"ASMS raw={raw}; expected target={asms_degrees(params, raw):+.3f} deg")
        except ValueError as exc:
            print(f"Invalid target: {exc}")


def cli(tx):
    print("commands: pc, asms, zero, zero_pc, auto, epc, easms, status, idle, q")
    print("epc sends the PC brake flag; easms sends the steering E-Stop (ASMS mode 3).")
    print("zero: ASMS MANUAL center at 50 Hz; zero_pc: PC 0 deg at 50 Hz.")
    print("Both can start homing. Continue sending zero until homing completes.")
    print(f"idle/q stop TX; the MCU's existing {tx.params.timeout_ms} ms timeout policy applies.")
    while True:
        command = input("\nmode> ").strip().lower()
        if command == "q":
            tx.set_mode("idle")
            return
        if command == "idle":
            tx.set_mode("idle")
        elif command in ("pc", "asms"):
            steering_mode(tx, command)
        elif command in ("zero", "zero_pc"):
            source = "pc" if command == "zero_pc" else "asms"
            tx.zero(source)
            print(f"[ZERO] {source.upper()} 0 deg at 50 Hz; idle to stop")
        elif command in ("auto", "epc", "easms"):
            source = "pc" if command == "epc" else "asms"
            ok = tx.one_shot(source, estop=(command != "auto"))
            print(f"[{command.upper()}] {'sent once' if ok else 'send failed'}; periodic TX idle")
        elif command in ("status", "s"):
            print_status(tx)
        else:
            print("Unknown command")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--no-periodic-log", action="store_true",
                        help="disable periodic console output; CSV logging remains active")
    parser.add_argument("--log-file", type=Path,
                        help="CSV diagnostics path (default: Python/logs/UDP_test_v4_<time>.csv)")
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        params, config_source = load_parameters(args.config)
    except (OSError, ValueError) as exc:
        print(f"[CONFIG ERROR] {exc}")
        return 1
    print("STM32 UDP steering debugger v4 (PC float32 / ASMS int16)")
    print("[PROTOCOL] PC=<ffB, 9 bytes; requires float32 firmware. ASMS=<BHh, 5 bytes.")
    print(f"[CONFIG] {config_source}")
    print(f"[LIMITS] {params.steering_min_deg:g} .. {params.steering_max_deg:g} deg; "
          f"ASMS scale={params.asms_scale:g}, polarity={params.asms_polarity:+d}")
    print(f"[HOMING] enabled={params.homing_enabled}, boot_count={params.boot_count}, "
          f"reached_band={params.reached_band_deg:g} deg, "
          f"return_ramp={params.homing_return_time_ms} ms")
    print(f"[CALIBRATION] ratio={params.gear_ratio:g}, "
          f"encoder/rev={params.encoder_counts_per_rev:g}, Kp={params.kp:g}")
    print(f"[NETWORK] target={STM32_IP}:{params.udp_port}, "
          f"sources={PC_SOURCE_IP}/{ASMS_SOURCE_IP}, adapter={ETH_IF_NAME}")
    print(f"[TIMING] TX=50 Hz; MCU timeout={params.timeout_ms} ms; "
          "console/CSV reporting runs separately")
    log_path = args.log_file or (Path(__file__).resolve().parent / "logs" /
                                f"UDP_test_v4_{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns()}.csv")
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        print(f"[LOG ERROR] {exc}")
        return 1
    print(f"[CSV] {log_path}")
    added_ips = set()
    sockets = []
    tx = None
    reporter = None
    try:
        for ip in (PC_SOURCE_IP, ASMS_SOURCE_IP):
            if not ensure_source_ip(ip, added_ips):
                return 1
            sockets.append(create_socket(ip))
        tx = SteeringTransmitter(params, sockets[0], sockets[1])
        print(f"[SOCKETS] PC={sockets[0].getsockname()}, ASMS={sockets[1].getsockname()}; "
              f"feedback :{params.status_port} is free for UDP_status_rx.py")
        tx.start()
        reporter = threading.Thread(target=report_loop,
                                    args=(tx, log_path, not args.no_periodic_log),
                                    name="udp-reporter", daemon=True)
        reporter.start()
        cli(tx)
    except (KeyboardInterrupt, EOFError):
        pass
    except OSError as exc:
        print(f"[NETWORK ERROR] {exc}")
        return 1
    finally:
        if tx is not None:
            tx.stop()
        if reporter is not None:
            reporter.join(timeout=0.2)
        for sock in sockets:
            sock.close()
        for ip in list(added_ips):
            netsh("delete", ip)
        if tx is not None:
            print_status(tx)
        print("closed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
