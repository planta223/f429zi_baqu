# baqu_steering_f429zi

바쿠 조향팀 - STM32 기반 조향 모터 제어 펌웨어입니다.

## 프로젝트 개요

본 프로젝트는 STM32 NUCLEO-F429ZI 보드를 기반으로 서보드라이브 및 서보모터를 이용해 자율주행차 조향축을 제어하기 위한 펌웨어입니다.

STM32는 Ethernet UDP를 통해 상위 제어 명령을 수신하고, TIM1 PWM 및 DIR GPIO를 이용하여 서보드라이브에 Pulse/Direction 명령을 출력합니다.  
TIM2 Encoder Mode를 이용해 드라이브 측 엔코더 피드백을 읽고, 이를 바탕으로 현재 조향각을 계산하여 폐루프 위치 제어를 수행합니다.

현재 펌웨어는 다음 모듈 중심으로 구성됩니다.

- `ethernet.c` : UDP packet 수신 및 PC/ASMS 명령 해석
- `encoder.c` : TIM2 Encoder Mode 기반 누적 카운트 및 각도 계산
- `motor.c` : TIM1 PWM 주파수 출력 및 DIR 제어
- `control.c` : 목표 조향각 기반 P/PI/PID 위치 제어
- `config.h` : 제어 주기, 기어비, 엔코더 카운트, Ethernet packet 설정

---

## 사용 하드웨어

- STM32 NUCLEO-F429ZI
- LS Electric XDL-L7S 계열 서보드라이브
- LS Electric XML-FBL04AMK1 서보모터
- 1차 감속기
- 최종 조향 전달 기구
- AM26LS31 라인드라이버
- AM26LS32ACN 라인리시버
- Ethernet UDP 통신용 LAN 연결

---

## 현재 통신 구조

### STM32 IP 설정

현재 STM32는 고정 IP를 사용합니다.

```text
STM32 IP      : 10.177.21.4
Subnet Mask   : 255.255.255.0
UDP Port      : 5000
DHCP          : Disabled