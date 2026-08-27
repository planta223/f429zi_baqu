/*
 * ethernet.h
 *
 *  Created on: May 23, 2026
 *      Author: kyubeom
 */

#ifndef INC_ETHERNET_H_
#define INC_ETHERNET_H_

#include <stdint.h>
#include <stdbool.h>

/* =========================================
 * Steering mode
 * ========================================= */

typedef enum {
    STEER_MODE_NONE   = 0,
    STEER_MODE_AUTO   = 1,
    STEER_MODE_MANUAL = 2,
    STEER_MODE_ESTOP  = 3
} SteerMode_t;

/* =========================================
 * Ethernet command source
 * ========================================= */

typedef enum {
    ETHERNET_SOURCE_NONE = 0,
    ETHERNET_SOURCE_ASMS = 1,
    ETHERNET_SOURCE_PC   = 2
} Ethernet_Source_t;

/* =========================================
 * Latest received packet state
 * ========================================= */

typedef struct {
    Ethernet_Source_t source;

    float steering_deg;       // 제어기에 전달할 목표 조향각 [deg]

    int16_t asms_adc_raw;     // ASMS joystick ADC raw
    int32_t  pc_steer_raw;    // PC steering raw
    uint32_t speed_raw;       // PC speed raw
    uint8_t  misc;            // PC misc field
} Ethernet_Packet_t;

/* =========================================
 * Public functions
 * ========================================= */

void Ethernet_Init(void);

bool Ethernet_IsInitialized(void);

bool Ethernet_HasNewData(void);
Ethernet_Packet_t Ethernet_GetLatestData(void);

SteerMode_t Ethernet_GetCurrentMode(void);

bool Ethernet_ConsumeEmergencyRequest(void);
uint32_t Ethernet_GetLastRxTick(void);

#endif /* INC_ETHERNET_H_ */
