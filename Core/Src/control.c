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

    /*
     * control 출력 제한은 motor 운용 상한을 초과하지 않도록 한 번 더 제한한다.
     */
    if (pid.output_limit > (float)MOTOR_MAX_FREQ_HZ) {
        pid.output_limit = (float)MOTOR_MAX_FREQ_HZ;
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
    return steering_deg * STEERING_GEAR_RATIO;
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
    const float dt_s = (float)CONTROL_PERIOD_MS / 1000.0f;

    float p_term = 0.0f;
    float i_term = 0.0f;
    float d_term = 0.0f;
    float derivative = 0.0f;

    float integral_candidate = 0.0f;
    float unsat_output = 0.0f;
    float sat_output = 0.0f;

    uint8_t allow_integral = 1U;

    derivative = (error_motor_deg - control.prev_error) / dt_s;

    /*
     * 먼저 현재 integral 기준으로 출력이 saturation되는지 본다.
     */
    p_term = pid.kp * error_motor_deg;
    i_term = pid.ki * control.integral;
    d_term = pid.kd * derivative;

    unsat_output = p_term + i_term + d_term;
    sat_output = Control_ApplyOutputLimit(unsat_output);

    /*
     * 출력이 +limit에 걸려 있고 error가 양수면,
     * integral이 더 커져 saturation을 악화시키므로 적분 중단.
     *
     * 출력이 -limit에 걸려 있고 error가 음수면,
     * integral이 더 작아져 saturation을 악화시키므로 적분 중단.
     */
    if ((unsat_output > pid.output_limit) && (error_motor_deg > 0.0f)) {
        allow_integral = 0U;
    } else if ((unsat_output < -pid.output_limit) && (error_motor_deg < 0.0f)) {
        allow_integral = 0U;
    }

    if (allow_integral != 0U) {
        integral_candidate = control.integral + (error_motor_deg * dt_s);

        if (integral_candidate > pid.integral_limit) {
            integral_candidate = pid.integral_limit;
        } else if (integral_candidate < -pid.integral_limit) {
            integral_candidate = -pid.integral_limit;
        }

        control.integral = integral_candidate;
    }

    /*
     * integral 반영 후 최종 출력 재계산.
     */
    p_term = pid.kp * error_motor_deg;
    i_term = pid.ki * control.integral;
    d_term = pid.kd * derivative;

    unsat_output = p_term + i_term + d_term;
    sat_output = Control_ApplyOutputLimit(unsat_output);

    control.prev_error = error_motor_deg;

    return sat_output;
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

    Motor_Stop();
}

void Control_Enable(void)
{
    control.enabled = 1U;
    control.reached = 0U;

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
    control.reached = 0U;

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

    control.current_motor_deg = Encoder_GetMotorDeg();
    control.current_steering_deg = Encoder_GetSteeringDeg();

    control.error_motor_deg = control.target_motor_deg - control.current_motor_deg;
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
    float reached_band_motor_deg = 0.0f;

    Control_UpdateTuningParams();

    control.current_motor_deg = Encoder_GetMotorDeg();
    control.current_steering_deg = Encoder_GetSteeringDeg();
    control.error_motor_deg = control.target_motor_deg - control.current_motor_deg;

    if (control.enabled == 0U) {
        control.output_freq_hz = 0.0f;
        return;
    }

    abs_error = fabsf(control.error_motor_deg);

    reached_band_motor_deg =
        CONTROL_REACHED_BAND_STEERING_DEG * STEERING_GEAR_RATIO;

    if (reached_band_motor_deg < 0.0f) {
        reached_band_motor_deg = -reached_band_motor_deg;
    }

    /*
     * 목표 근처에서는 PID 출력을 내지 않고 즉시 정지한다.
     * MOTOR_MIN_FREQ_HZ에 의해 작은 출력이 최소 주파수로 강제되는 것을 방지한다.
     */
    if (abs_error <= reached_band_motor_deg) {
        control.reached = 1U;
        control.output_freq_hz = 0.0f;
        control.integral = 0.0f;

        if (Motor_IsOutputActive() != 0U) {
            Motor_Stop();
        }

        return;
    }

    control.reached = 0U;

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
