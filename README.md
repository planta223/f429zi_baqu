# baqu_steering_f429zi

바쿠 조향팀 - STM32 기반 자율주행차 조향 모터 제어 펌웨어입니다.

본 프로젝트는 STM32 NUCLEO-F429ZI 보드를 기반으로 LS Electric 서보드라이브 및 서보모터를 이용하여 자율주행차의 조향축을 제어하기 위한 펌웨어입니다. STM32는 Ethernet UDP를 통해 PC(자율주행) 또는 ASMS(조이스틱)로부터 조향 명령을 수신하고, TIM1 PWM 및 DIR GPIO를 이용해 서보드라이브에 Pulse/Direction 명령을 출력합니다. 또한 TIM2 Encoder Mode를 통해 엔코더 피드백을 읽고, 누적 엔코더 카운트를 기반으로 현재 모터각 및 조향각을 계산하여 목표 조향각을 추종합니다. 별도의 ESTOP 스위치 및 통신 timeout 정책을 통해 서보모터의 토크 활성화/비활성화 가능합니다.

## 사용 하드웨어

| 항목 | 사용 부품 / 설정 |
|---|---|
| MCU 보드 | STM32 NUCLEO-F429ZI |
| 서보드라이브 | LS Electric XDL-L7S 계열 |
| 서보모터 | LS Electric XML-FBL04AMK1 |
| 조향 구동부 | 서보모터 + 감속기 + 최종 조향 전달 기구 |
| 라인드라이버 | AM26LS31 |
| 라인리시버 | AM26LS32ACN |
| NPN 달링턴 트랜지스터 | ULN2003APG |

## Flow

```text
PC / ASMS
   ↓ Ethernet UDP
STM32 NUCLEO-F429ZI
   ↓ UDP packet parsing
Target steering angle [deg]
   ↓ Position control
Control output frequency [Hz]
   ↓ TIM1 PWM + DIR GPIO
Line driver
   ↓
LS Electric Servo Drive
   ↓
Servo Motor
   ↓
Reducer / Steering Mechanism
   ↓ Encoder feedback
Line receiver
   ↓
STM32 TIM2 Encoder Mode
   ↓
Current motor angle / Current steering angle
```

## 프로젝트 파일 구성

현재 커밋과 호환되지 않을 수 있습니다.

```text
baqu_steering_f429zi.ioc

Core/
 ├─ Inc/
 │   ├─ config.h
 │   ├─ ethernet.h
 │   ├─ encoder.h
 │   ├─ motor.h
 │   └─ control.h
 │
 └─ Src/
     ├─ main.c
     ├─ ethernet.c
     ├─ encoder.c
     ├─ motor.c
     └─ control.c

Python/
 └─ UDP_test.py
```

## 주요 파라미터

| 제어 주기 | `CONTROL_PERIOD_MS = 1U` |
| 최종 기어비 | `STEERING_GEAR_RATIO = 6.03448f` |
| 모터 1회전당 엔코더 카운트 | `ENCODER_COUNT_PER_MOTOR_REV = 24000.0f` |
| 조향축 기계적 양의 한계 | `STEERING_MECHANICAL_MAX_DEG = 40.0f` |
| 조향축 기계적 음의 한계 | `STEERING_MECHANICAL_MIN_DEG = -40.0f` |
| 제어 목표 조향각 상한 | `CONTROL_TARGET_MAX_STEERING_DEG = 30.0f` |
| 제어 목표 조향각 하한 | `CONTROL_TARGET_MIN_STEERING_DEG = -30.0f` |
| 제어 도달 판정각 | `CONTROL_REACHED_BAND_STEERING_DEG = 0.2f` |
| P gain | `CONTROL_DEFAULT_KP = 500.0f` |
| I gain | `CONTROL_DEFAULT_KI = 0.0f` |
| D gain | `CONTROL_DEFAULT_KD = 0.0f` |
| PWM 최소 주파수 | `MOTOR_MIN_FREQ_HZ = 150U` |
| PWM 최대 주파수 | `MOTOR_MAX_FREQ_HZ = 600000U` |
| 방향 반전 대기 시간 | `MOTOR_DIRECTION_GUARD_MS = 1U` |
| UDP 수신 포트 | `ETHERNET_UDP_PORT = 5000U` |
| ASMS 중립 deadband | `ETHERNET_ASMS_ADC_DEADBAND_RAW = 50` |
| 통신 timeout | `ETHERNET_TIMEOUT_MS = 300U` |

현재 제어 게인은 `KP = 500`, `KI = 0`, `KD = 0`으로 설정되어 있으므로 실질적으로 P 제어만 사용합니다. 도달 판정 범위는 `±0.2 deg`입니다.

## 제어 디버깅 변수

현재 Live Expressions에서 아래의 주요 엔코더 debug 변수를 확인할 수 있습니다.

```c
g_dbg_encoder_raw_count
g_dbg_encoder_delta_count
g_dbg_encoder_total_count
g_dbg_encoder_motor_deg
g_dbg_encoder_steering_deg
```

## 통신 및 패킷 구조

STM32는 고정 IP를 사용합니다.

| 항목 | 값 |
|---|---|
| STM32 IP | `10.177.21.4` |
| Subnet Mask | `255.255.255.0` |
| UDP Port | `5000` |
| DHCP | Disabled |

PC 테스트 스크립트 기준 설정은 다음과 같습니다.

| 항목 | 값 |
|---|---|
| PC IP | `10.177.21.1` |
| Subnet Mask | `255.255.255.0` |
| Adapter Name | `이더넷` |

`UDP_test.py`는 Windows 환경에서 `netsh` 명령을 이용해 PC 이더넷 어댑터에 임시 IP를 추가합니다. 따라서 관리자 권한 CMD 또는 PowerShell에서 실행해야 합니다.

UDP packet은 packet size를 기준으로 ASMS packet과 PC packet을 구분합니다.

| Source | Packet size | 용도 |
|---|---:|---|
| ASMS | 5 bytes | 모드 전환 및 조이스틱 기반 수동 조향 |
| PC | 9 bytes | AUTO mode에서 목표 조향각 송신 |

현재 `ETHERNET_USE_IP_FILTER = 0U`이므로 IP 마지막 옥텟은 검사하지 않고 packet size만으로 source를 구분합니다. 실제 통합 단계에서는 `ETHERNET_USE_IP_FILTER = 1U` 적용을 검토해야 합니다.

ASMS packet 구조는 다음과 같습니다.

| Byte | 자료형 | 내용 |
|---|---|---|
| `byte[0]` | `uint8_t` | mode |
| `byte[1]` | `uint8_t` | unused |
| `byte[2]` | `uint8_t` | unused |
| `byte[3:4]` | `int16_t` | joystick ADC raw, little-endian |

ASMS mode 값은 다음과 같습니다.

| Mode | 값 | 의미 |
|---|---:|---|
| `STEER_MODE_AUTO` | 1 | PC packet 기반 자동 조향 |
| `STEER_MODE_MANUAL` | 2 | ASMS 조이스틱 기반 수동 조향 |
| `STEER_MODE_ESTOP` | 3 | 비상정지 |

ASMS ADC 입력 범위는 다음과 같습니다.

| 항목 | 값 |
|---|---:|
| ADC raw minimum | `-2048` |
| ADC raw center | `0` |
| ADC raw maximum | `2047` |
| Deadband | `±50` |
| 변환 후 목표 조향각 범위 | `-60 deg ~ +60 deg` |

PC packet 구조는 다음과 같습니다.

| Byte | 자료형 | 내용 |
|---|---|---|
| `byte[0:3]` | `int32_t` | steering raw, little-endian |
| `byte[4:7]` | `uint32_t` | speed raw, little-endian |
| `byte[8]` | `uint8_t` | misc, bit7 = emergency stop |

현재 설정에서 PC steering raw는 degree 단위로 해석됩니다.

```text
pc_steer_raw = 10
→ target_steering_deg = 10 deg
```

PC packet은 ASMS packet을 통해 AUTO mode로 전환된 상태에서만 유효합니다.

## 시행착오

아래 글은 현재 커밋과 호환되지 않습니다.

현재 소스 상 목표 조향각은 `-60 deg ~ +60 deg`로 설정되어 있으나, 실제 조향축은 그보다 작은 각도까지만 회전하는 현상이 있습니다. 이 문제는 실제 최종 조향 전달비와 `STEERING_GEAR_RATIO` 불일치, 서보드라이브 표시 카운트와 STM32 엔코더 누적 카운트 불일치, TIM2 Encoder Mode의 pulse 누락, 최종 조향 기구의 추가 감속비 또는 유격 미반영, 실제 조향축 각도 측정 기준과 STM32 계산 기준 불일치 등으로 인해 발생할 수 있습니다. 따라서 목표 조향각별 실제 조향축 각도를 측정하고, 서보드라이브 표시 count와 STM32 `g_dbg_encoder_total_count`를 비교한 뒤, 실제 조향각 기준으로 `STEERING_GEAR_RATIO`를 재산정해야 합니다.

또한 조이스틱을 좌우로 반복 조작한 뒤 중립으로 복귀했을 때 실제 조향이 좌측 방향으로 누적되는 현상이 있습니다. 현재 의심 원인은 STM32 엔코더 count와 서보드라이브 표시 count의 불일치, TIM2 Encoder Mode의 고속 엔코더 신호 누락, A/B상 신호 품질 저하, 엔코더 입력 필터 설정 부적합, 서보드라이브의 single-turn / multi-turn 표시 기준과 STM32 누적 count 기준의 불일치입니다. 이를 확인하기 위해 저속, 중속, 고속 조건에서 엔코더 count 누락 여부를 확인하고, CW / CCW 방향별 count 대칭성을 검증해야 합니다. 필요 시 TIM2 Encoder Mode 대신 A/B STEP 또는 edge count 기반 수신 방식도 검토해야 합니다.

안전장치도 추가해야 합니다. 현재 소프트웨어에는 목표 조향각 제한, 출력 제한, timeout, ESTOP가 구현되어 있으나 제어 실패, 엔코더 누락, 기구 걸림, 탈조, 과토크 상황에서는 기구적 손상 가능성이 있습니다. 따라서 서보드라이브 토크 상한 설정, 서보드라이브 alarm output 확인 및 STM32 입력 연동, 좌우 limit switch 추가, home 또는 center sensor 추가, 하드웨어 ESTOP 회로 구성, 시운전용 저속 제한 profile 추가, 제어 실패 시 PWM 차단 및 서보 enable 차단 구조 검토가 필요합니다.

통신 구조도 보완이 필요합니다. 현재는 packet size만으로 ASMS packet과 PC packet을 구분하므로, 실제 통합 단계에서는 `ETHERNET_USE_IP_FILTER = 1U` 적용을 검토해야 합니다. 또한 ASMS source IP와 PC source IP를 분리하고, packet checksum 또는 sequence number를 추가하여 잘못된 packet 수신 시 무시하는 방어 로직을 강화할 필요가 있습니다.