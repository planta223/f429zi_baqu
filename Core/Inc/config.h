/*
 * config.h
 *
 *  Created on: May 22, 2026
 *      Author: kyubeom
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include <stdint.h>

/* =========================================
 * System constants
 * ========================================= */
// 제어주기
#define CONTROL_PERIOD_MS              1U

/* =========================================
 * Hardware constants
 * ========================================= */
#define STEERING_GEAR_RATIO            50.0f   // 감속비
#define ENCODER_COUNT_PER_MOTOR_REV    12000.0f   // 1회전당 카운트 (실측 후 확정됨)
#define MOTOR_DRIVER_MAX_FREQ_HZ       1000000U   // 드라이브 line-driver 입력 사양 상한
#define ENCODER_COUNT_POLARITY         1 	      // 엔코더 방향 (실측 후 확정됨)
#define MOTOR_DIR_ACTIVE_HIGH_FOR_CW   1 		  // 모터회전 방향 (실측 후 확정됨)

#define STEERING_MECHANICAL_MAX_DEG     40.0f     // 조향축 기계적 양의 한계 [deg]
#define STEERING_MECHANICAL_MIN_DEG    -40.0f     // 조향축 기계적 음의 한계 [deg]


/* =========================================
 * svon.c
 * ========================================= */
#define SVON_ACTIVE_HIGH    1U


/* =========================================
 * encoder.c
 * ========================================= */
#define ENCODER_COUNTER_PERIOD         4294967295UL // 32-bit TIM period: 0 ~ 4294967295
#define ENCODER_COUNTER_CENTER         2147483648UL // center of 32-bit counter range


/* =========================================
 * motor.c
 * ========================================= */
#define MOTOR_MIN_FREQ_HZ               150U      // 현재 TIM1 PSC=17에서 실질적 저주파 하한 근처
#define MOTOR_MAX_FREQ_HZ               600000U   // 개루프 제어 운용 상한 (MOTOR_DRIVER_MAX_FREQ_HZ 이하로 설정할 것)
#define MOTOR_DIRECTION_GUARD_MS        1U        // 방향 반전시 대기시간(ms)


/* =========================================
 * control.c : 수행
 * ========================================= */
#define CONTROL_TARGET_MAX_STEERING_DEG       30.0f      // 제어 목표 조향각 상한 [deg] (STEERING_MECHANICAL_MAX_DEG 이하로 설정할 것)
#define CONTROL_TARGET_MIN_STEERING_DEG      -30.0f      // 제어 목표 조향각 하한 [deg]
#define CONTROL_REACHED_BAND_STEERING_DEG      0.5f       // 도달 판정 조향각 오차 범위 [deg]

#define CONTROL_DEFAULT_KP                   400.0f     // P 게인 [Hz/motor_deg]
#define CONTROL_DEFAULT_KI                     0.0f       // I 게인
#define CONTROL_DEFAULT_KD                     0.0f       // D 게인
#define CONTROL_INTEGRAL_LIMIT                50.0f    // 적분항 누적 제한
#define CONTROL_OUTPUT_LIMIT_HZ            600000.0f   // 폐루프 제어 운용 상한 (MOTOR_MAX_FREQ_HZ 이하로 설정할 것)


/* =========================================
 * comm_manager.c : 해석
 * ========================================= */

#define COMM_MODE_ETHERNET_ONLY     0U // 수정 금지. ethernet 단독 (대회 기본값)
#define COMM_MODE_CAN_ONLY          1U // 수정 금지. can 단독
#define COMM_MODE_BOTH              2U // 수정 금지. ethernet + can 동시, 제어권은 Ethernet
#define COMM_MODE                   COMM_MODE_ETHERNET_ONLY

#define COMM_TIMEOUT_POLICY_HOLD        0U // 수정 금지. 마지막 유효 목표 조향각 유지
#define COMM_TIMEOUT_POLICY_RELEASE     1U // 수정 금지. Servo off
#define ETHERNET_TIMEOUT_MS             300U
#define ETHERNET_TIMEOUT_POLICY         COMM_TIMEOUT_POLICY_RELEASE
#define CAN_TIMEOUT_MS                  300U
#define CAN_TIMEOUT_POLICY              COMM_TIMEOUT_POLICY_RELEASE

/* Ethernet ASMS Command */
#define ASMS_STEER_SCALE                30.0f
#define ASMS_STEER_POLARITY            -1
#define ASMS_ADC_MIN_RAW               (-2048)
#define ASMS_ADC_CENTER_RAW             0
#define ASMS_ADC_MAX_RAW                2047
#define ASMS_ADC_DEADBAND_RAW           50

/* Ethernet PC Command */
#define PC_STEER_SCALE                  1.0f
#define PC_STEER_POLARITY               1
#define PC_ALLOW_AUTO_ENTRY             1U // NONE 상태에서 PC command만으로 AUTO 진입 허용

/* CAN Request */
#define CAN_REQUEST_STEER_SCALE         100.0f // 0.01 deg/LSB, raw = deg × 100

/* CAN Status */
#define CAN_STATUS_PERIOD_MS            20U
#define CAN_STATUS_STEER_SCALE          100.0f // 0.01 deg/LSB, raw = deg × 100


/* =========================================
 * comm_ethernet.c : 파싱 1
 * ========================================= */
#define ETHERNET_UDP_PORT                 5000U
#define ETHERNET_USE_IP_FILTER            1U // PC/ASMS 송신 IP 검사 여부

/* ASMS UDP packet */
#define ETHERNET_ASMS_PACKET_SIZE         5U
#define ETHERNET_ASMS_MODE_OFFSET         0U // Byte 0
#define ETHERNET_ASMS_SPEED_OFFSET        1U // Byte 1,2. 사용 안함.
#define ETHERNET_ASMS_STEER_OFFSET        3U // Byte 3,4
#define ETHERNET_ASMS_IP_LAST_OCTET       5U // 10.177.21.5

/* PC UDP packet */
#define ETHERNET_PC_PACKET_SIZE           9U
#define ETHERNET_PC_STEER_OFFSET          0U // Byte 0,1,2,3
#define ETHERNET_PC_SPEED_OFFSET          4U // Byte 4,5,6,7. 사용 안함.
#define ETHERNET_PC_MISC_OFFSET           8U // Byte 8
#define ETHERNET_PC_MISC_ESTOP_MASK       0x80U // Byte 8(MISC)의 bit 7(MSB): E-Stop
#define ETHERNET_PC_IP_LAST_OCTET         1U // 10.177.21.1


/* =========================================
 * comm_can.c : 파싱 2
 * ========================================= */
/* CAN Hardware */ 
#define COMM_CAN_HANDLE       hcan1 
#define COMM_CAN_RX_IRQn      CAN1_RX0_IRQn

 /* CAN ID */
#define CAN_ID_STEER_REQUEST              0x100U
#define CAN_ID_STEER_STATUS               0x180U

/* CAN DLC */
#define CAN_STEER_REQUEST_DLC             3U
#define CAN_STEER_STATUS_DLC              5U

/* Steering Request - RX */
#define CAN_REQUEST_STEER_OFFSET          0U // Byte 0~1: Requested steering angle
#define CAN_REQUEST_FLAGS_OFFSET          2U // Byte 2: Flags
#define CAN_REQUEST_ESTOP_MASK            0x80U // Flags bit 7(MSB): E-Stop

/* Steering Status - TX */
#define CAN_STATUS_ACTUAL_OFFSET          0U // Byte 0~1: Actual steering angle
#define CAN_STATUS_TARGET_OFFSET          2U // Byte 2~3: Applied target angle
#define CAN_STATUS_FLAGS_OFFSET           4U // Byte 4: Status flags

#define CAN_STATUS_FLAG_REACHED               (1U << 0) // bit0 : 목표 조향값 도달 여부
#define CAN_STATUS_FLAG_CONTROL_ENABLED       (1U << 1) // bit1 : 제어 활성화 여부
#define CAN_STATUS_FLAG_SVON_ENABLED          (1U << 2) // bit2 : SVON 활성화 여부
#define CAN_STATUS_FLAG_MOTOR_OUTPUT_ACTIVE   (1U << 3) // bit3 : PULSE/PWM 출력 활성화 여부
#define CAN_STATUS_FLAG_ENCODER_INITIALIZED   (1U << 4) // bit4 : 엔코더 초기화 여부

#endif /* INC_CONFIG_H_ */