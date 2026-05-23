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
 * System timing
 * ========================================= */
#define CONTROL_PERIOD_MS              1U

/* =========================================
 * Hardware constants
 * ========================================= */
#define STEERING_GEAR_RATIO            12.5f      // 감속비
#define ENCODER_COUNT_PER_MOTOR_REV    24000.0f   // 1회전당 카운트 (실측 후 확정 필요)
#define MOTOR_DRIVER_MAX_FREQ_HZ       1000000U   // 드라이브 line-driver 입력 사양 상한
#define ENCODER_COUNT_POLARITY         -1 	      // 엔코더 방향 (실측 후 확정됨)
#define MOTOR_DIR_ACTIVE_HIGH_FOR_CW   0 		  // 모터회전 방향 (실측 후 확정됨)

/* =========================================
 * Runtime tuning variables
 * Watch/change in Live Expressions
 * ========================================= */
extern volatile float g_control_kp;
extern volatile float g_control_ki;
extern volatile float g_control_kd;
extern volatile float g_control_integral_limit;
extern volatile float g_control_output_limit_hz;

/* =========================================
 * encoder.c
 * ========================================= */
#define ENCODER_COUNTER_PERIOD         4294967295UL // 32-bit TIM period: 0 ~ 4294967295
#define ENCODER_COUNTER_CENTER         2147483648UL // center of 32-bit counter range

/* =========================================
 * motor.c
 * ========================================= */
#define MOTOR_MIN_FREQ_HZ               150U      // 현재 TIM1 PSC=17에서 실질적 저주파 하한 근처
#define MOTOR_MAX_FREQ_HZ               100000U   // 개루프 제어 운용 상한 (MOTOR_DRIVER_MAX_FREQ_HZ 이하로 설정할 것)
#define MOTOR_DIRECTION_GUARD_MS        1U        // 방향 반전시 대기시간(ms)

/* =========================================
 * control.c
 * ========================================= */
#define CONTROL_TARGET_MAX_STEERING_DEG      40.0f      // 목표 조향각 상한 [deg]
#define CONTROL_TARGET_MIN_STEERING_DEG     -40.0f      // 목표 조향각 하한 [deg]
#define CONTROL_REACHED_BAND_STEERING_DEG    0.2f       // 도달 판정 조향각 오차 범위 [deg]

#define CONTROL_DEFAULT_KP                   50.0f      // P 게인 [Hz/motor_deg]
#define CONTROL_DEFAULT_KI                   0.0f       // I 게인
#define CONTROL_DEFAULT_KD                   0.0f       // D 게인
#define CONTROL_INTEGRAL_LIMIT               1000.0f    // 적분항 누적 제한
#define CONTROL_OUTPUT_LIMIT_HZ              10000.0f   // 폐루프 제어 운용 상한 (MOTOR_MAX_FREQ_HZ 이하로 설정할 것)

#endif /* INC_CONFIG_H_ */
