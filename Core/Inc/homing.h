/*
 * homing.h
 *
 *  Created on: Oct 19, 2026
 *      Author: kyubeom
 */

#ifndef INC_HOMING_H_
#define INC_HOMING_H_

#include <stdbool.h>

typedef enum {
    HOMING_STATE_WAIT_ZERO = 0,
    HOMING_STATE_MOVING,
    HOMING_STATE_READY
} Homing_State_t;

/* 부팅 시 한 번 호출. */
void Homing_Init(void);

/*
 * 명령 적용 전 호출.
 *
 * is_pc_command:
 *   true  = 상위제어 Ethernet PC 명령
 *   false = ASMS / CAN 명령
 *
 * E-Stop, 제어권, 패킷 유효성 검사는 호출자가 먼저 수행한다.
 * 이 함수는 상태를 변경하지 않는다.
 */
bool Homing_CheckCommand(
    float steering_deg,
    bool is_pc_command);

/*
 * 실제 목표 적용과 Control/SVON 활성화에 성공한 뒤 호출.
 */
void Homing_OnTargetApplied(
    float steering_deg,
    bool is_pc_command);

/*
 * Encoder_Update(), Control_Update() 이후 호출.
 *
 * pc_command_valid:
 *   PC 제어권과 통신 유효성이 현재 유지되고 있는지 여부.
 *
 * 모터 제어나 timeout 처리는 수행하지 않는다.
 */
void Homing_Update(bool pc_command_valid);

Homing_State_t Homing_GetState(void);
bool Homing_IsReady(void);

#endif /* INC_HOMING_H_ */
