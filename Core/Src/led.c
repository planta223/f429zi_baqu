/*
 * led.c
 *
 *  Created on: Sep 19, 2026
 *      Author: kyubeom
 */


#include "led.h"
#include "main.h"


void LED_Init(void)
{
    LED_GREEN_OFF();
    LED_BLUE_OFF();
    LED_RED_OFF();
}


void LED_GREEN_ON(void)
{
    HAL_GPIO_WritePin(
        LD1_GPIO_Port,
        LD1_Pin,
        GPIO_PIN_SET
    );
}


void LED_GREEN_OFF(void)
{
    HAL_GPIO_WritePin(
        LD1_GPIO_Port,
        LD1_Pin,
        GPIO_PIN_RESET
    );
}


void LED_BLUE_ON(void)
{
    HAL_GPIO_WritePin(
        LD2_GPIO_Port,
        LD2_Pin,
        GPIO_PIN_SET
    );
}


void LED_BLUE_OFF(void)
{
    HAL_GPIO_WritePin(
        LD2_GPIO_Port,
        LD2_Pin,
        GPIO_PIN_RESET
    );
}


void LED_RED_ON(void)
{
    HAL_GPIO_WritePin(
        LD3_GPIO_Port,
        LD3_Pin,
        GPIO_PIN_SET
    );
}


void LED_RED_OFF(void)
{
    HAL_GPIO_WritePin(
        LD3_GPIO_Port,
        LD3_Pin,
        GPIO_PIN_RESET
    );
}
