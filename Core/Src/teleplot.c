/*
 * teleplot.c
 *
 *  Created on: May 23, 2026
 *      Author: kyubeom
 */

#include "teleplot.h"

#include "usart.h"
#include "control.h"
#include "encoder.h"
#include "config.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define TELEPLOT_UART              huart3
#define TELEPLOT_PERIOD_MS         50U
#define TELEPLOT_TX_TIMEOUT_MS     20U

volatile uint32_t g_teleplot_update_count = 0U;
volatile uint32_t g_teleplot_tx_ok_count = 0U;
volatile uint32_t g_teleplot_tx_fail_count = 0U;
volatile uint32_t g_teleplot_last_status = 0U;

static uint32_t teleplot_last_tx_ms = 0U;

static void Teleplot_SendLine(const char *line)
{
    HAL_StatusTypeDef status;

    if (line == NULL) {
        return;
    }

    status = HAL_UART_Transmit(&TELEPLOT_UART,
                               (uint8_t *)line,
                               (uint16_t)strlen(line),
                               TELEPLOT_TX_TIMEOUT_MS);

    g_teleplot_last_status = (uint32_t)status;

    if (status == HAL_OK) {
        g_teleplot_tx_ok_count++;
    } else {
        g_teleplot_tx_fail_count++;
    }
}

static void Teleplot_SendFloat(const char *name, float value)
{
    char buf[96];

    int32_t scaled;
    int32_t integer;
    int32_t frac;
    int n;

    if (name == NULL) {
        return;
    }

    if (value >= 0.0f) {
        scaled = (int32_t)(value * 1000.0f + 0.5f);
    } else {
        scaled = (int32_t)(value * 1000.0f - 0.5f);
    }

    integer = scaled / 1000;
    frac = scaled % 1000;

    if (frac < 0) {
        frac = -frac;
    }

    /*
     * Teleplot VSCode Serial format:
     * >name:value\n
     */
    if ((scaled < 0) && (integer == 0)) {
        n = snprintf(buf, sizeof(buf), ">%s:-0.%03ld\n",
                     name,
                     (long)frac);
    } else {
        n = snprintf(buf, sizeof(buf), ">%s:%ld.%03ld\n",
                     name,
                     (long)integer,
                     (long)frac);
    }

    if ((n > 0) && (n < (int)sizeof(buf))) {
        Teleplot_SendLine(buf);
    }
}

void Teleplot_Init(void)
{
    teleplot_last_tx_ms = 0U;

    g_teleplot_update_count = 0U;
    g_teleplot_tx_ok_count = 0U;
    g_teleplot_tx_fail_count = 0U;
    g_teleplot_last_status = 0U;
}

void Teleplot_Update(uint32_t now_ms)
{
    Control_State_t control;
    Encoder_t encoder;

    float error_steering_deg = 0.0f;

    if ((uint32_t)(now_ms - teleplot_last_tx_ms) < TELEPLOT_PERIOD_MS) {
        return;
    }

    teleplot_last_tx_ms = now_ms;
    g_teleplot_update_count++;

    control = Control_GetState();
    encoder = Encoder_GetState();

    error_steering_deg = control.error_motor_deg / STEERING_GEAR_RATIO;

    Teleplot_SendFloat("target",  control.target_steering_deg);
    Teleplot_SendFloat("current", control.current_steering_deg);
    Teleplot_SendFloat("error",   error_steering_deg);
    Teleplot_SendFloat("out",     control.output_freq_hz);
    Teleplot_SendFloat("vel",     encoder.steering_velocity_dps);

    Teleplot_SendFloat("integ",   control.integral);
    Teleplot_SendFloat("pterm",   g_control_kp * control.error_motor_deg);
    Teleplot_SendFloat("iterm",   g_control_ki * control.integral);
}
