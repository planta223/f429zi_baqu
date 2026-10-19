import sys
import time
import statistics
from collections import Counter

# CandoCAN_test.py가 있는 경로
sys.path.insert(
    0,
    r"E:\Workspace\STM32CubeIDE\baqu_steering_f429zi\Python"
)

# libusb 초기화와 candleLight open_bus() 재사용
import CandoCAN_test as ct

DURATION_SECONDS = 10
STATUS_CAN_ID = 0x180

bus = ct.open_bus(
    channel=0,
    bitrate=500_000,
)

start = time.monotonic()
status_timestamps = []
error_ids = Counter()

# [status 프레임 수, error 프레임 수]
buckets = [
    [0, 0] for _ in range(DURATION_SECONDS)
]

try:
    while time.monotonic() - start < DURATION_SECONDS:
        msg = bus.recv(timeout=0.1)

        if msg is None:
            continue

        elapsed = time.monotonic() - start
        index = min(
            DURATION_SECONDS - 1,
            int(elapsed),
        )

        if msg.is_error_frame:
            buckets[index][1] += 1
            error_ids[f"0x{msg.arbitration_id:X}"] += 1

        elif msg.arbitration_id == STATUS_CAN_ID:
            buckets[index][0] += 1
            status_timestamps.append(msg.timestamp)

finally:
    bus.shutdown()

for second, (status_count, error_count) in enumerate(buckets):
    print(
        f"{second:02d}-{second + 1:02d}s "
        f"status={status_count:3d} "
        f"errors={error_count:4d}"
    )

intervals_ms = [
    (current - previous) * 1000.0
    for previous, current in zip(
        status_timestamps,
        status_timestamps[1:],
    )
]

total_errors = sum(
    error_count
    for _, error_count in buckets
)

print(f"total_status={len(status_timestamps)}")
print(f"total_errors={total_errors}")

if intervals_ms:
    ordered = sorted(intervals_ms)
    p99_index = min(
        len(ordered) - 1,
        int(len(ordered) * 0.99),
    )

    print(
        "interval_ms "
        f"avg={statistics.mean(intervals_ms):.3f} "
        f"min={min(intervals_ms):.3f} "
        f"p99={ordered[p99_index]:.3f} "
        f"max={max(intervals_ms):.3f}"
    )

print("error_ids=", dict(error_ids))
