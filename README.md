# baqu_steering_f429zi

바쿠 조향팀 - Ethernet 기반 STM32 조향 모터 제어 프로젝트입니다.

## 프로젝트 개요

본 프로젝트는 STM32 NUCLEO-F429ZI 보드를 기반으로 조향 모터를 제어하기 위한 펌웨어입니다.  
외부 상위 제어기는 Ethernet UDP 명령을 통해 목표 조향각을 전달하고, STM32는 엔코더 피드백을 이용해 조향 위치 및 속도를 계산한 뒤 모터 구동 펄스를 출력합니다.

## 사용 보드

- Board: NUCLEO-F429ZI
- MCU: STM32F429ZITx
- IDE: STM32CubeIDE
- CubeMX: STM32CubeMX 기반 `.ioc` 설정 사용

## 주요 기능

- Ethernet RMII + LwIP 기반 UDP 통신
- TIM2 Encoder Mode 기반 엔코더 카운트 측정
- TIM1 PWM 기반 Pulse 출력
- GPIO 기반 Direction 출력
- USART3 기반 ST-LINK Virtual COM Port 로그
- IWDG 기반 main loop hang 감시
- 향후 PI 제어 기반 조향 모터 위치 제어 구현 예정

## .ioc 설정

baqu_steering_f428zi 재개발

ioc 핀 설정
- USB, LSE reset state
- RMII, USART3, UserBtn 및 LED, TMS 및 TCK 유지
- TIM1_CH1(PE9), TIM2_CH1(PA0), TIM2_CH2(PB3), GPIO_OUT(PE10, DIR_PIN)

ioc 파라미터 설정
- TIM1 : PWM generation (Prescaler 179, ARR 999, Pulse 500, prload enable)
- TIM2 : Encoder mode T1&T2 (Prescaler 65535, Rising edge, filter 4)
- USART3 : 기본 설정 유지
- ETH : Rx Buffers Length 1536
- LWIP : DHCP disable, IP 주소 세개 합의값으로 지정
- IWDG : clock period 64, reload 999

그외 설정
- Stack/Heap size 0x1000으로 확보
- SYSCLK & HCLK 180MHz, APB1 prescaler 4, APB2 prescaler 2
- 다음 체크 : Generate peripheral initialization as a pair of '.c/.h' files per peripheral