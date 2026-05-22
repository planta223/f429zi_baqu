/*
 * control.c
 *
 *  Created on: May 22, 2026
 *      Author: kyubeom
 */

#include "control.h"
#include "encoder.h"
#include "motor.h"
#include "config.h"

#include <math.h>

volatile float g_control_kp = CONTROL_DEFAULT_KP;
volatile float g_control_ki = CONTROL_DEFAULT_KI;
volatile float g_control_kd = CONTROL_DEFAULT_KD;
volatile float g_control_integral_limit = CONTROL_INTEGRAL_LIMIT;
volatile float g_control_output_limit_hz = CONTROL_OUTPUT_LIMIT_HZ;

volatile float g_control_reached_band_motor_deg = CONTROL_REACHED_BAND_MOTOR_DEG;
volatile uint32_t g_control_reached_time_ms = CONTROL_REACHED_TIME_MS;

static Control_PID_t pid;
static Control_State_t control;

static void Control_UpdateTuningParams(void)
{
    pid.kp = g_control_kp;
    pid.ki = g_control_ki;
    pid.kd = g_control_kd;

    pid.integral_limit = g_control_integral_limit;
    if (pid.integral_limit < 0.0f) {
        pid.integral_limit = -pid.integral_limit;
    }

    pid.output_limit = g_control_output_limit_hz;
    if (pid.output_limit < 0.0f) {
        pid.output_limit = -pid.output_limit;
    }
}

static float Control_ClampFloat(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }

    if (value < min_value) {
        return min_value;
    }

    return value;
}

static float Control_SteeringDegToMotorDeg(float steering_deg)
{
    return steering_deg * MOTOR_DEG_PER_STEERING_DEG;
}

static float Control_ApplyOutputLimit(float output)
{
    if (output > pid.output_limit) {
        return pid.output_limit;
    }

    if (output < -pid.output_limit) {
        return -pid.output_limit;
    }

    return output;
}

static float Control_CalcPID(float error_motor_deg)
{
    float p_term = 0.0f;
    float i_term = 0.0f;
    float d_term = 0.0f;
    float derivative = 0.0f;
    float output = 0.0f;

    control.integral += error_motor_deg * CONTROL_PERIOD_S;

    if (control.integral > pid.integral_limit) {
        control.integral = pid.integral_limit;
    } else if (control.integral < -pid.integral_limit) {
        control.integral = -pid.integral_limit;
    }

    derivative = (error_motor_deg - control.prev_error) / CONTROL_PERIOD_S;
    control.prev_error = error_motor_deg;

    p_term = pid.kp * error_motor_deg;
    i_term = pid.ki * control.integral;
    d_term = pid.kd * derivative;

    output = p_term + i_term + d_term;
    output = Control_ApplyOutputLimit(output);

    return output;
}

void Control_Init(void)
{
	Control_UpdateTuningParams();

    control.enabled = 0U;
    control.reached = 0U;

    control.target_steering_deg = 0.0f;
    control.target_motor_deg = 0.0f;

    control.current_steering_deg = 0.0f;
    control.current_motor_deg = 0.0f;

    control.error_motor_deg = 0.0f;
    control.output_freq_hz = 0.0f;

    control.integral = 0.0f;
    control.prev_error = 0.0f;

    control.reached_time_ms = 0U;

    Motor_Stop();
}

void Control_Enable(void)
{
    control.enabled = 1U;
    control.reached = 0U;
    control.reached_time_ms = 0U;

    control.integral = 0.0f;

    control.current_motor_deg = Encoder_GetMotorDeg();
    control.current_steering_deg = Encoder_GetSteeringDeg();

    control.error_motor_deg = control.target_motor_deg - control.current_motor_deg;
    control.prev_error = control.error_motor_deg;
}

void Control_Disable(void)
{
    control.enabled = 0U;
    control.output_freq_hz = 0.0f;
    control.integral = 0.0f;
    control.reached_time_ms = 0U;

    Motor_Stop();
}

void Control_Reset(void)
{
    control.target_steering_deg = 0.0f;
    control.target_motor_deg = 0.0f;

    control.current_motor_deg = Encoder_GetMotorDeg();
    control.current_steering_deg = Encoder_GetSteeringDeg();

    control.error_motor_deg = control.target_motor_deg - control.current_motor_deg;
    control.output_freq_hz = 0.0f;

    control.integral = 0.0f;
    control.prev_error = control.error_motor_deg;

    control.reached = 0U;
    control.reached_time_ms = 0U;

    Motor_Stop();
}

void Control_SetTargetSteeringDeg(float steering_deg)
{
    float clamped_steering_deg = Control_ClampFloat(steering_deg,
                                                    CONTROL_TARGET_MIN_STEERING_DEG,
                                                    CONTROL_TARGET_MAX_STEERING_DEG);

    control.target_steering_deg = clamped_steering_deg;
    control.target_motor_deg = Control_SteeringDegToMotorDeg(clamped_steering_deg);

    control.reached = 0U;
    control.reached_time_ms = 0U;
    control.integral = 0.0f;

    control.current_motor_deg = Encoder_GetMotorDeg();
    control.current_steering_deg = Encoder_GetSteeringDeg();
    control.error_motor_deg = control.target_motor_deg - control.current_motor_deg;
    control.prev_error = control.error_motor_deg;
}

void Control_SetPID(float kp, float ki, float kd)
{
    g_control_kp = kp;
    g_control_ki = ki;
    g_control_kd = kd;

    control.integral = 0.0f;
    control.prev_error = control.error_motor_deg;
}

void Control_Update(void)
{
    float abs_error = 0.0f;
    float output = 0.0f;
    float reached_band = 0.0f;
    uint32_t reached_time = 0U;

    Control_UpdateTuningParams();

    control.current_motor_deg = Encoder_GetMotorDeg();
    control.current_steering_deg = Encoder_GetSteeringDeg();
    control.error_motor_deg = control.target_motor_deg - control.current_motor_deg;

    if (control.enabled == 0U) {
        control.output_freq_hz = 0.0f;
        return;
    }

    abs_error = fabsf(control.error_motor_deg);

    reached_band = g_control_reached_band_motor_deg;
    reached_time = g_control_reached_time_ms;

    if (reached_band < 0.0f) {
        reached_band = -reached_band;
    }

    if (reached_time == 0U) {
        reached_time = CONTROL_PERIOD_MS;
    }

    if (abs_error <= reached_band) {
        if (control.reached_time_ms < reached_time) {
            control.reached_time_ms += CONTROL_PERIOD_MS;
        }

        if (control.reached_time_ms >= reached_time) {
            control.reached = 1U;
            control.output_freq_hz = 0.0f;
            control.integral = 0.0f;
            Motor_Stop();
            return;
        }
    } else {
        control.reached = 0U;
        control.reached_time_ms = 0U;
    }

    output = Control_CalcPID(control.error_motor_deg);
    control.output_freq_hz = output;

    Motor_SetFrequency((int32_t)output);
}

uint8_t Control_IsEnabled(void)
{
    return control.enabled;
}

uint8_t Control_IsReached(void)
{
    return control.reached;
}

Control_State_t Control_GetState(void)
{
    return control;
}
