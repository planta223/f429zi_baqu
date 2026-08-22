/*
 * svon.c
 *
 *  Created on: Aug 22, 2026
 *      Author: kyubeom
 */


#include "svon.h"
#include "main.h"
#include "config.h"

static uint8_t svon_enabled = 0U;

static void SVON_WriteOutput(uint8_t enable)
{
    GPIO_PinState pin_state;

#if SVON_ACTIVE_HIGH
    pin_state = (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
#else
    pin_state = (enable != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET;
#endif

    HAL_GPIO_WritePin(SVON_PIN_GPIO_Port, SVON_PIN_Pin, pin_state);
}

void SVON_Init(void)
{
    /*
     * MCU 부팅 시에는 반드시 Servo OFF 상태로 시작한다.
     */
    svon_enabled = 0U;
    SVON_WriteOutput(0U);
}

void SVON_Enable(void)
{
    SVON_WriteOutput(1U);
    svon_enabled = 1U;
}

void SVON_Disable(void)
{
    SVON_WriteOutput(0U);
    svon_enabled = 0U;
}

uint8_t SVON_IsEnabled(void)
{
    return svon_enabled;
}
