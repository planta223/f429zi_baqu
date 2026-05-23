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
#define TELEPLOT_PERIOD_MS         10U
#define TELEPLOT_TX_TIMEOUT_MS     2U

static uint32_t teleplot_last_tx_ms = 0U;

static void Teleplot_SendLine(const char *line)
{
    if (line == NULL) {
        return;
    }

    (void)HAL_UART_Transmit(&TELEPLOT_UART,
                            (uint8_t *)line,
                            (uint16_t)strlen(line),
                            TELEPLOT_TX_TIMEOUT_MS);
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

    if ((scaled < 0) && (integer == 0)) {
        n = snprintf(buf, sizeof(buf), "%s:-0.%03ld\r\n",
                     name,
                     (long)frac);
    } else {
        n = snprintf(buf, sizeof(buf), "%s:%ld.%03ld\r\n",
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

    control = Control_GetState();
    encoder = Encoder_GetState();

    error_steering_deg = control.error_motor_deg / STEERING_GEAR_RATIO;

    Teleplot_SendFloat("target_steering_deg",   control.target_steering_deg);
    Teleplot_SendFloat("current_steering_deg",  control.current_steering_deg);
    Teleplot_SendFloat("error_steering_deg",    error_steering_deg);
    Teleplot_SendFloat("output_freq_hz",        control.output_freq_hz);
    Teleplot_SendFloat("steering_velocity_dps", encoder.steering_velocity_dps);
}
