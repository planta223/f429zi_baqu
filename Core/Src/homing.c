/*
 * homing.c
 *
 *  Created on: Oct 19, 2026
 *      Author: kyubeom
 */

#include "homing.h"

#include "config.h"
#include "control.h"
#include "encoder.h"
#include "svon.h"

#include <math.h>

#ifndef STEERING_BOOT_CENTERING_ENABLE
#error "Define STEERING_BOOT_CENTERING_ENABLE in config.h"
#endif

#if ((STEERING_BOOT_CENTERING_ENABLE != 0U) && \
     (STEERING_BOOT_CENTERING_ENABLE != 1U))
#error "STEERING_BOOT_CENTERING_ENABLE must be 0 or 1"
#endif

/* Homing_Init() 이전에는 명령을 제한한다. */
static Homing_State_t homing_state = HOMING_STATE_WAIT_ZERO;

static bool Homing_IsZeroCommand(
    float steering_deg,
    bool is_homing_source)
{
    return is_homing_source &&
           isfinite(steering_deg) &&
           (steering_deg == 0.0f);
}

void Homing_Init(void)
{
#if STEERING_BOOT_CENTERING_ENABLE
    homing_state = HOMING_STATE_WAIT_ZERO;
#else
    homing_state = HOMING_STATE_READY;
#endif
}

bool Homing_CheckCommand(
    float steering_deg,
    bool is_homing_source)
{
    if (!isfinite(steering_deg)) {
        return false;
    }

    switch (homing_state) {
    case HOMING_STATE_READY:
        return true;

    case HOMING_STATE_WAIT_ZERO:
    case HOMING_STATE_MOVING:
        /* 중앙 복귀 전에는 PC / ASMS의 해석된 0도만 허용. */
        return Homing_IsZeroCommand(
            steering_deg,
            is_homing_source);

    default:
        return false;
    }
}

void Homing_OnTargetApplied(
    float steering_deg,
    bool is_homing_source)
{
    if (homing_state != HOMING_STATE_WAIT_ZERO) {
        return;
    }

    if (!Homing_IsZeroCommand(
            steering_deg,
            is_homing_source)) {
        return;
    }

    /*
     * 패킷 수신 시점이 아니라
     * 목표 적용과 제어 활성화에 성공한 시점에 전환한다.
     */
    homing_state = HOMING_STATE_MOVING;
}

void Homing_Update(bool command_valid)
{
    Control_State_t control_state;

    if (homing_state != HOMING_STATE_MOVING) {
        return;
    }

    /*
     * 통신 단절, E-Stop, 제어권 변경 중에는
     * 완료 처리하지 않는다.
     * 상태와 encoder 좌표는 유지한다.
     */
    if (!command_valid ||
        Encoder_IsInitialized() == 0U ||
        SVON_IsEnabled() == 0U) {
        return;
    }

    control_state = Control_GetState();

    if (control_state.enabled == 0U ||
        control_state.reached == 0U) {
        return;
    }

    if (!isfinite(control_state.target_steering_deg) ||
        !isfinite(control_state.current_steering_deg)) {
        return;
    }

    if (control_state.target_steering_deg != 0.0f) {
        return;
    }

    if (fabsf(control_state.current_steering_deg) >
        CONTROL_REACHED_BAND_STEERING_DEG) {
        return;
    }

    /*
     * 중앙 복귀 완료.
     * Encoder_Reset()은 호출하지 않는다.
     */
    homing_state = HOMING_STATE_READY;
}

Homing_State_t Homing_GetState(void)
{
    return homing_state;
}

bool Homing_IsReady(void)
{
    return homing_state == HOMING_STATE_READY;
}
