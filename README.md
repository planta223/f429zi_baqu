# baqu_steering_f429zi

바쿠 조향팀 - STM32 기반 조향 모터 제어 펌웨어입니다.

## 프로젝트 개요

본 프로젝트는 STM32 NUCLEO-F429ZI 보드를 기반으로 서보드라이브 및 서보모터를 이용해 조향축을 제어하기 위한 펌웨어입니다.

STM32는 TIM1 PWM으로 Pulse 신호를 출력하고, GPIO로 Direction 신호를 출력합니다.  
TIM2 Encoder Mode를 이용해 드라이브 측 엔코더 피드백을 읽고, 이를 바탕으로 모터축 각도와 감속기 이후 조향축 각도를 계산합니다.

현재 구조는 기존 복잡한 `position_control`, `pulse_control`, `encoder_reader` 계열을 단순화하여 다음 3개 모듈 중심으로 재구성했습니다.

- `encoder.c/h` : TIM2 엔코더 카운트 측정 및 각도 환산
- `motor.c/h` : TIM1 PWM Pulse 출력 및 DIR 제어
- `control.c/h` : 목표 조향각 기반 폐루프 위치제어

현재 감속기가 장착된 상태이며, `g_test_target_steering_deg`는 **감속기 이후 조향축 기준 각도**입니다.  
내부 제어에서는 조향각을 감속비에 따라 모터축 각도로 변환한 뒤 제어합니다.

---

## 사용 보드 및 하드웨어

### MCU 보드

- Board: NUCLEO-F429ZI
- MCU: STM32F429ZITx
- IDE: STM32CubeIDE
- CubeMX: STM32CubeMX 기반 `.ioc` 설정 사용

### 구동계

- Servo Drive: LS ELECTRIC XDL-L7SA004BAA
- Servo Motor: XML-FBL04AMK1
- Motor Power: 400 W
- Rated Speed: 3000 rpm
- Encoder: Serial BiSS 계열, 19-bit급 해상도 기준
- Gear Ratio: `12.5 : 1`

### 인터페이스

- Pulse 출력: TIM1_CH1, PE9
- Direction 출력: PE10, `DIR_PIN`
- Encoder A/B 입력: TIM2_CH1 PA0, TIM2_CH2 PB3
- Debug UART: USART3, ST-LINK Virtual COM Port
- Ethernet: RMII + LwIP
- Watchdog: IWDG 사용

---

## 현재 구현 상태

### 완료된 기능

- TIM2 32-bit Encoder Mode 설정
- 엔코더 누적 count 기반 모터축 각도 계산
- 감속비 기반 조향축 각도 계산
- TIM1 PWM 기반 Pulse 출력
- PE10 GPIO 기반 Direction 출력
- 방향 전환 시 PWM 정지 후 guard time 적용
- Live Expressions 기반 임시 테스트 변수 구성
- Open-loop 구동 확인
  - `g_test_motor_freq_hz = 200`
  - `g_test_motor_freq_hz = 0`
  - `g_test_motor_freq_hz = -200`
  - `g_test_motor_freq_hz = 0`
- 폐루프 소각도 구동 확인
  - `g_test_target_steering_deg = 0.2f`
  - 목표 방향 이동 확인
  - reached 판정 후 motor stop 확인

### 현재 단계

현재는 정밀 튜닝 전 단계입니다.

- Open-loop 양방향 구동은 확인됨
- Encoder 방향과 motor 방향은 기본적으로 맞는 것으로 확인됨
- 폐루프 위치제어는 소각도에서 동작 확인됨
- 다음 단계는 Hz 제한값 단계적 증가 및 Teleplot 기반 PI 튜닝

---

## .ioc 설정 요약

### Pin 설정

- USB, LSE: Reset state
- RMII: 유지
- USART3: ST-LINK VCP용 유지
- User Button 및 LED 유지
- SWD: TMS, TCK 유지
- TIM1_CH1: PE9, PWM Pulse 출력
- TIM2_CH1: PA0, Encoder A 입력
- TIM2_CH2: PB3, Encoder B 입력
- GPIO Output: PE10, `DIR_PIN`

### TIM1 설정

- Mode: PWM Generation CH1
- Prescaler: 179
- Counter Period / ARR: 999
- Pulse: 500
- Auto-reload preload: Enable

현재 APB2 timer clock 기준 TIM1 tick은 1 MHz로 사용합니다.

### TIM2 설정

- Mode: Encoder Mode TI1 and TI2
- Counter Period: `4294967295`
- Polarity: Rising Edge
- Input Filter: 4
- Prescaler Division: No division

TIM2는 32-bit encoder counter로 사용합니다.

### Clock 설정

- SYSCLK: 180 MHz
- HCLK: 180 MHz
- APB1 Prescaler: 4
- APB2 Prescaler: 2

### 기타

- USART3: 기본 설정 유지
- ETH: RMII + LwIP
- LWIP: DHCP disable, 고정 IP 사용
- IWDG: Prescaler 64, Reload 999
- Stack/Heap size: 0x1000
- Generate peripheral initialization as pair of `.c/.h` files per peripheral: Enable

---

## 주요 상수

### Encoder

```c
#define ENCODER_TIMER_CENTER           2147483648UL
#define ENCODER_COUNTER_PERIOD         4294967295UL

#define ENCODER_PPR                    524288.0f
#define ENCODER_QUADRATURE             4.0f
#define ENCODER_COUNT_PER_REV          (ENCODER_PPR * ENCODER_QUADRATURE)
#define ENCODER_DEG_PER_COUNT          (360.0f / ENCODER_COUNT_PER_REV)

#define ENCODER_COUNT_POLARITY         1