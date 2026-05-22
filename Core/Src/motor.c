/*
 * motor.c
 *
 *  Created on: May 22, 2026
 *      Author: kyubeom
 */

#include "motor.h"
#include "tim.h"
#include "main.h"
#include "config.h"

volatile uint32_t g_motor_min_freq_hz = MOTOR_MIN_FREQ_HZ;
volatile uint32_t g_motor_max_freq_hz = MOTOR_MAX_FREQ_HZ;

#define MOTOR_TIMER        htim1
#define MOTOR_PWM_CHANNEL  TIM_CHANNEL_1

typedef enum {
    MOTOR_REVERSE_IDLE = 0U,
    MOTOR_REVERSE_WAIT_STOP,
    MOTOR_REVERSE_WAIT_DIR_SETTLE
} MotorReverseState_t;

static Motor_State_t motor;

static TIM_HandleTypeDef *motor_htim = &MOTOR_TIMER;

static MotorDirection_t pending_direction = MOTOR_DIR_CCW;
static uint32_t pending_freq_hz = 0U;
static uint32_t reverse_deadline_ms = 0U;
static MotorReverseState_t reverse_state = MOTOR_REVERSE_IDLE;

static uint32_t Motor_GetTimerClockHz(void)
{
    RCC_ClkInitTypeDef clk_init = {0};
    uint32_t flash_latency = 0U;
    uint32_t pclk = HAL_RCC_GetPCLK2Freq();

    HAL_RCC_GetClockConfig(&clk_init, &flash_latency);

    if (clk_init.APB2CLKDivider == RCC_HCLK_DIV1) {
        return pclk;
    }

    return pclk * 2U;
}

static uint8_t Motor_DeadlineExpired(uint32_t deadline_ms)
{
    return ((int32_t)(HAL_GetTick() - deadline_ms) >= 0) ? 1U : 0U;
}

static uint32_t Motor_ClampFrequency(uint32_t freq_hz)
{
    uint32_t min_freq = g_motor_min_freq_hz;
    uint32_t max_freq = g_motor_max_freq_hz;

    if (max_freq < min_freq) {
        max_freq = min_freq;
    }

    if (freq_hz < min_freq) {
        return min_freq;
    }

    if (freq_hz > max_freq) {
        return max_freq;
    }

    return freq_hz;
}

static void Motor_ApplyDirection(MotorDirection_t dir)
{
    GPIO_PinState pin_state = GPIO_PIN_RESET;

    if (dir == MOTOR_DIR_CW) {
        pin_state = (MOTOR_DIR_ACTIVE_HIGH_FOR_CW != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    } else {
        pin_state = (MOTOR_DIR_ACTIVE_HIGH_FOR_CW != 0) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(DIR_PIN_GPIO_Port, DIR_PIN_Pin, pin_state);
    motor.direction = dir;
}

static uint32_t Motor_CalcAppliedFrequency(uint32_t period_counts)
{
    uint32_t timer_clock_hz = Motor_GetTimerClockHz();
    uint32_t prescaler = motor_htim->Instance->PSC + 1U;
    uint64_t denominator = (uint64_t)prescaler * (uint64_t)period_counts;

    if (denominator == 0U) {
        return 0U;
    }

    return (uint32_t)(((uint64_t)timer_clock_hz + (denominator / 2U)) / denominator);
}

static void Motor_ApplyPwmFrequency(uint32_t freq_hz)
{
    uint32_t timer_clock_hz = Motor_GetTimerClockHz();
    uint32_t prescaler = motor_htim->Instance->PSC + 1U;
    uint64_t denominator = (uint64_t)prescaler * (uint64_t)freq_hz;
    uint64_t period_counts = 0U;
    uint32_t arr = 0U;
    uint32_t ccr = 0U;

    if (denominator == 0U) {
        motor.applied_freq_hz = 0U;
        return;
    }

    period_counts = ((uint64_t)timer_clock_hz + (denominator / 2U)) / denominator;

    if (period_counts < 2U) {
        period_counts = 2U;
    }

    if (period_counts > 65536U) {
        period_counts = 65536U;
    }

    arr = (uint32_t)(period_counts - 1U);
    ccr = (uint32_t)(period_counts / 2U);

    if (ccr == 0U) {
        ccr = 1U;
    }

    if (ccr > arr) {
        ccr = arr;
    }

    __HAL_TIM_SET_AUTORELOAD(motor_htim, arr);
    __HAL_TIM_SET_COMPARE(motor_htim, MOTOR_PWM_CHANNEL, ccr);

    motor.applied_freq_hz = Motor_CalcAppliedFrequency((uint32_t)period_counts);
}

static void Motor_StartOutput(uint32_t freq_hz)
{
    Motor_ApplyPwmFrequency(freq_hz);

    if (motor.output_active == 0U) {
        if (HAL_TIM_PWM_Start(motor_htim, MOTOR_PWM_CHANNEL) == HAL_OK) {
            motor.output_active = 1U;
        } else {
            motor.output_active = 0U;
            motor.applied_freq_hz = 0U;
        }
    }
}

static void Motor_StopInternal(void)
{
    HAL_TIM_PWM_Stop(motor_htim, MOTOR_PWM_CHANNEL);

    motor.output_active = 0U;
    motor.applied_freq_hz = 0U;
}

static void Motor_BeginReverseGuard(MotorDirection_t dir, uint32_t freq_hz)
{
    pending_direction = dir;
    pending_freq_hz = freq_hz;

    Motor_StopInternal();

    reverse_deadline_ms = HAL_GetTick() + MOTOR_DIRECTION_GUARD_MS;
    reverse_state = MOTOR_REVERSE_WAIT_STOP;
    motor.reverse_guard_active = 1U;
}

static void Motor_ServiceReverseGuard(void)
{
    if (reverse_state == MOTOR_REVERSE_IDLE) {
        motor.reverse_guard_active = 0U;
        return;
    }

    if (reverse_state == MOTOR_REVERSE_WAIT_STOP) {
        if (Motor_DeadlineExpired(reverse_deadline_ms) == 0U) {
            return;
        }

        Motor_ApplyDirection(pending_direction);

        reverse_deadline_ms = HAL_GetTick() + MOTOR_DIRECTION_GUARD_MS;
        reverse_state = MOTOR_REVERSE_WAIT_DIR_SETTLE;
        motor.reverse_guard_active = 1U;
        return;
    }

    if (reverse_state == MOTOR_REVERSE_WAIT_DIR_SETTLE) {
        if (Motor_DeadlineExpired(reverse_deadline_ms) == 0U) {
            return;
        }

        reverse_state = MOTOR_REVERSE_IDLE;
        motor.reverse_guard_active = 0U;

        if (pending_freq_hz > 0U) {
            Motor_StartOutput(pending_freq_hz);
        }
    }
}

void Motor_Init(void)
{
    motor.requested_freq_hz = 0;
    motor.applied_freq_hz = 0U;
    motor.direction = MOTOR_DIR_CCW;
    motor.output_active = 0U;
    motor.reverse_guard_active = 0U;

    pending_direction = MOTOR_DIR_CCW;
    pending_freq_hz = 0U;
    reverse_deadline_ms = 0U;
    reverse_state = MOTOR_REVERSE_IDLE;

    Motor_ApplyDirection(MOTOR_DIR_CCW);
    Motor_StopInternal();
}

void Motor_SetFrequency(int32_t freq_hz)
{
    MotorDirection_t target_direction = MOTOR_DIR_CCW;
    uint32_t target_freq_hz = 0U;

    motor.requested_freq_hz = freq_hz;

    Motor_ServiceReverseGuard();

    if (freq_hz == 0) {
        pending_freq_hz = 0U;
        reverse_state = MOTOR_REVERSE_IDLE;
        motor.reverse_guard_active = 0U;
        Motor_StopInternal();
        return;
    }

    if (freq_hz > 0) {
        target_direction = MOTOR_DIR_CW;
        target_freq_hz = Motor_ClampFrequency((uint32_t)freq_hz);
    } else {
        target_direction = MOTOR_DIR_CCW;
        target_freq_hz = Motor_ClampFrequency((uint32_t)(-freq_hz));
    }

    if (reverse_state != MOTOR_REVERSE_IDLE) {
        pending_direction = target_direction;
        pending_freq_hz = target_freq_hz;
        return;
    }

    if (target_direction != motor.direction) {
        Motor_BeginReverseGuard(target_direction, target_freq_hz);
        return;
    }

    Motor_StartOutput(target_freq_hz);
}

void Motor_Stop(void)
{
    motor.requested_freq_hz = 0;
    pending_freq_hz = 0U;
    reverse_state = MOTOR_REVERSE_IDLE;
    motor.reverse_guard_active = 0U;

    Motor_StopInternal();
}

uint8_t Motor_IsOutputActive(void)
{
    return motor.output_active;
}

Motor_State_t Motor_GetState(void)
{
    return motor;
}
