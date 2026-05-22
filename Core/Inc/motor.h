/*
 * motor.h
 *
 *  Created on: May 22, 2026
 *      Author: kyubeom
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>

typedef enum {
    MOTOR_DIR_CCW = 0,
    MOTOR_DIR_CW  = 1
} MotorDirection_t;

typedef struct {
    int32_t requested_freq_hz;
    uint32_t applied_freq_hz;
    MotorDirection_t direction;
    uint8_t output_active;
    uint8_t reverse_guard_active;
} Motor_State_t;

void Motor_Init(void);
void Motor_SetFrequency(int32_t freq_hz);
void Motor_Stop(void);

uint8_t Motor_IsOutputActive(void);
Motor_State_t Motor_GetState(void);

#endif /* INC_MOTOR_H_ */
