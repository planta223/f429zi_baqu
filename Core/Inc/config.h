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
#define CONTROL_PERIOD_S               0.001f

/* =========================================
 * Runtime tuning variables
 * Watch/change in Live Expressions
 * ========================================= */
extern volatile float g_control_kp;
extern volatile float g_control_ki;
extern volatile float g_control_kd;
extern volatile float g_control_integral_limit;
extern volatile float g_control_output_limit_hz;

extern volatile float g_control_reached_band_motor_deg;
extern volatile uint32_t g_control_reached_time_ms;

extern volatile uint32_t g_motor_min_freq_hz;
extern volatile uint32_t g_motor_max_freq_hz;

/* =========================================
 * encoder.c
 * ========================================= */
#define ENCODER_TIMER_CENTER           2147483648UL
#define ENCODER_COUNTER_PERIOD         4294967295UL

#define ENCODER_PPR                    524288.0f
#define ENCODER_QUADRATURE             4.0f
#define ENCODER_COUNT_PER_REV          (ENCODER_PPR * ENCODER_QUADRATURE)
#define ENCODER_DEG_PER_COUNT          (360.0f / ENCODER_COUNT_PER_REV)

#define ENCODER_COUNT_POLARITY         -1

#define STEERING_GEAR_RATIO            12.5f
#define MOTOR_DEG_PER_STEERING_DEG     STEERING_GEAR_RATIO
#define STEERING_DEG_PER_MOTOR_DEG     (1.0f / STEERING_GEAR_RATIO)

/* =========================================
 * motor.c
 * ========================================= */
#define MOTOR_MIN_FREQ_HZ                 10U
#define MOTOR_MAX_FREQ_HZ                 100000U

#define MOTOR_DIR_ACTIVE_HIGH_FOR_CW      0
#define MOTOR_DIRECTION_GUARD_MS          1U

/* =========================================
 * control.c
 * ========================================= */
#define CONTROL_TARGET_MAX_STEERING_DEG      45.0f
#define CONTROL_TARGET_MIN_STEERING_DEG     -45.0f

#define CONTROL_DEFAULT_KP                   50.0f
#define CONTROL_DEFAULT_KI                   0.0f
#define CONTROL_DEFAULT_KD                   0.0f

#define CONTROL_INTEGRAL_LIMIT               1000.0f
#define CONTROL_OUTPUT_LIMIT_HZ              100000.0f

#define CONTROL_REACHED_BAND_STEERING_DEG    0.2f
#define CONTROL_REACHED_BAND_MOTOR_DEG       (CONTROL_REACHED_BAND_STEERING_DEG * MOTOR_DEG_PER_STEERING_DEG)
#define CONTROL_REACHED_TIME_MS              100U

#endif /* INC_CONFIG_H_ */
