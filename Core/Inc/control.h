/*
 * control.h
 *
 *  Created on: May 22, 2026
 *      Author: kyubeom
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} Control_PID_t;

typedef struct {
    uint8_t enabled;
    uint8_t reached;

    float target_steering_deg;
    float target_motor_deg;

    float current_steering_deg;
    float current_motor_deg;

    float error_motor_deg;
    float output_freq_hz;

    float integral;
    float prev_error;

    uint32_t reached_time_ms;
} Control_State_t;

void Control_Init(void);
void Control_Enable(void);
void Control_Disable(void);
void Control_Reset(void);

void Control_SetTargetSteeringDeg(float steering_deg);
void Control_SetPID(float kp, float ki, float kd);

void Control_Update(void);

uint8_t Control_IsEnabled(void);
uint8_t Control_IsReached(void);
Control_State_t Control_GetState(void);

#endif /* INC_CONTROL_H_ */
