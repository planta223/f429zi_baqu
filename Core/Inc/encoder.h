/*
 * encoder.h
 *
 *  Created on: May 22, 2026
 *      Author: kyubeom
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>

typedef struct {
    uint32_t raw_count;
    uint32_t prev_raw_count;
    int32_t delta_count;
    int64_t total_count;

    float motor_deg;
    float steering_deg;
    float motor_velocity_dps;
    float steering_velocity_dps;

    uint8_t initialized;
} Encoder_t;

void Encoder_Init(void);
void Encoder_Update(void);
void Encoder_Reset(void);

int32_t Encoder_GetDeltaCount(void);
int64_t Encoder_GetTotalCount(void);
uint32_t Encoder_GetRawCount(void);
float Encoder_GetMotorDeg(void);
float Encoder_GetSteeringDeg(void);
float Encoder_GetMotorVelocityDps(void);
float Encoder_GetSteeringVelocityDps(void);
uint8_t Encoder_IsInitialized(void);
Encoder_t Encoder_GetState(void);

#endif /* INC_ENCODER_H_ */
