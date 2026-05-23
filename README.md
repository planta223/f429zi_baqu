# baqu_steering_f429zi

바쿠 조향팀 - STM32 기반 조향 모터 제어 펌웨어입니다.

## 프로젝트 개요

본 프로젝트는 STM32 NUCLEO-F429ZI 보드를 기반으로 LS ELECTRIC 서보드라이브 및 서보모터를 이용해 조향축을 제어하기 위한 펌웨어입니다.

STM32는 TIM1 PWM으로 Pulse 신호를 출력하고, GPIO로 Direction 신호를 출력합니다.  
TIM2 Encoder Mode를 이용해 드라이브 측 엔코더 피드백을 읽고, 누적 count를 기준으로 모터축 각도와 감속기 이후 조향축 각도를 계산합니다.

현재 구조는 기존의 복잡한 `position_control`, `pulse_control`, `encoder_reader` 계열을 정리하고, 다음 3개 모듈 중심으로 단순화했습니다.

- `encoder.c/h` : TIM2 엔코더 count 측정, 누적 count 관리, 모터축/조향축 각도 환산
- `motor.c/h` : TIM1 PWM Pulse 출력, Direction GPIO 제어, 방향 전환 guard 처리
- `control.c/h` : 목표 조향각 기반 폐루프 위치제어 및 PI/PID 튜닝 변수 관리

현재 감속기가 장착된 상태이며, `g_test_target_steering_deg`는 **감속기 이후 조향축 기준 각도**입니다.  
내부 제어에서는 조향축 목표각을 감속비에 따라 모터축 목표각으로 변환한 뒤 제어합니다.

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

### 신호 인터페이스

- Pulse 출력: TIM1_CH1, PE9
- Direction 출력: PE10, `DIR_PIN`
- Encoder A 입력: TIM2_CH1, PA0
- Encoder B 입력: TIM2_CH2, PB3
- Debug UART: USART3, ST-LINK Virtual COM Port
- Ethernet: RMII + LwIP
- Watchdog: IWDG 사용

### 하드웨어 경로

- 모터 명령 송신  
  STM32 → 라인드라이버 → 레벨시프터 → 서보드라이브 → 서보모터

- 엔코더 데이터 수신  
  서보모터 → 서보드라이브 → 레벨시프터 → 라인리시버 → STM32

---

## 현재 소스 구조

```text
Core/
├── Inc/
│   ├── config.h
│   ├── encoder.h
│   ├── motor.h
│   └── control.h
│
└── Src/
    ├── main.c
    ├── encoder.c
    ├── motor.c
    └── control.c