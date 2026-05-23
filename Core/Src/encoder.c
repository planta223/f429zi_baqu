/*
 * encoder.c
 *
 *  Created on: May 22, 2026
 *      Author: kyubeom
 */


#include "encoder.h"
#include "tim.h"
#include "config.h"

#define ENCODER_TIMER	htim2

static Encoder_t encoder;

static int32_t Encoder_ApplyPolarity(int32_t count)
{
#if ENCODER_COUNT_POLARITY < 0
    return -count;
#else
    return count;
#endif
}

static float Encoder_CountToMotorDeg(int64_t count)
{
    return ((float)count * 360.0f) / ENCODER_COUNT_PER_MOTOR_REV;
}

static float Encoder_MotorDegToSteeringDeg(float motor_deg)
{
    return motor_deg / STEERING_GEAR_RATIO;
}

static float Encoder_CountToMotorVelocityDps(int32_t delta)
{
    return ((float)delta * 360.0f / ENCODER_COUNT_PER_MOTOR_REV) *
           (1000.0f / (float)CONTROL_PERIOD_MS);
}

void Encoder_Init(void)
{
    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, ENCODER_COUNTER_CENTER);

    if (HAL_TIM_Encoder_Start(&ENCODER_TIMER, TIM_CHANNEL_ALL) != HAL_OK) {
        encoder.raw_count = 0U;
        encoder.prev_raw_count = 0U;
        encoder.delta_count = 0;
        encoder.total_count = 0;

        encoder.motor_deg = 0.0f;
        encoder.steering_deg = 0.0f;
        encoder.motor_velocity_dps = 0.0f;
        encoder.steering_velocity_dps = 0.0f;

        encoder.initialized = 0U;
        return;
    }

    encoder.raw_count = (uint32_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER);
    encoder.prev_raw_count = encoder.raw_count;
    encoder.delta_count = 0;
    encoder.total_count = 0;

    encoder.motor_deg = 0.0f;
    encoder.steering_deg = 0.0f;
    encoder.motor_velocity_dps = 0.0f;
    encoder.steering_velocity_dps = 0.0f;

    encoder.initialized = 1U;
}

void Encoder_Update(void)
{
    uint32_t raw;
    int32_t signed_delta;
    int32_t delta;

    if (encoder.initialized == 0U) {
        return;
    }

    raw = (uint32_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER);

    /*
     * 32-bit timer wrap-around handling.
     * Valid if the count change per update is within int32_t range.
     */
    signed_delta = (int32_t)(raw - encoder.prev_raw_count);
    delta = Encoder_ApplyPolarity(signed_delta);

    encoder.raw_count = raw;
    encoder.delta_count = delta;
    encoder.total_count += (int64_t)delta;

    encoder.motor_deg = Encoder_CountToMotorDeg(encoder.total_count);
    encoder.steering_deg = Encoder_MotorDegToSteeringDeg(encoder.motor_deg);

    encoder.motor_velocity_dps = Encoder_CountToMotorVelocityDps(delta);
    encoder.steering_velocity_dps =
        Encoder_MotorDegToSteeringDeg(encoder.motor_velocity_dps);

    encoder.prev_raw_count = raw;
}

void Encoder_Reset(void)
{
    if (encoder.initialized == 0U) {
        return;
    }

    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, ENCODER_COUNTER_CENTER);

    encoder.raw_count = (uint32_t)ENCODER_COUNTER_CENTER;
    encoder.prev_raw_count = (uint32_t)ENCODER_COUNTER_CENTER;
    encoder.delta_count = 0;
    encoder.total_count = 0;

    encoder.motor_deg = 0.0f;
    encoder.steering_deg = 0.0f;
    encoder.motor_velocity_dps = 0.0f;
    encoder.steering_velocity_dps = 0.0f;
}

int32_t Encoder_GetDeltaCount(void)
{
    return encoder.delta_count;
}

int64_t Encoder_GetTotalCount(void)
{
    return encoder.total_count;
}

uint32_t Encoder_GetRawCount(void)
{
    return encoder.raw_count;
}

float Encoder_GetMotorDeg(void)
{
    return encoder.motor_deg;
}

float Encoder_GetSteeringDeg(void)
{
    return encoder.steering_deg;
}

float Encoder_GetMotorVelocityDps(void)
{
    return encoder.motor_velocity_dps;
}

float Encoder_GetSteeringVelocityDps(void)
{
    return encoder.steering_velocity_dps;
}

uint8_t Encoder_IsInitialized(void)
{
    return encoder.initialized;
}

Encoder_t Encoder_GetState(void)
{
    return encoder;
}
