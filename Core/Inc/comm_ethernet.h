/*
 * comm_ethernet.h
 *
 *  Created on: Aug 27, 2026
 *      Author: kyubeom
 */

#ifndef INC_COMM_ETHERNET_H_
#define INC_COMM_ETHERNET_H_

#include <stdint.h>
#include <stdbool.h>

/* =========================================
 * Ethernet E-Stop source mask
 * ========================================= */

#define COMM_ETHERNET_ESTOP_NONE   0x00U
#define COMM_ETHERNET_ESTOP_ASMS   (1U << 0)
#define COMM_ETHERNET_ESTOP_PC     (1U << 1)


/* ASMS packet */
typedef struct {
    uint8_t  mode_raw;
    uint16_t speed_raw;
    int16_t steer_raw;
    uint32_t rx_tick_ms;
} CommEthernet_AsmsPacket_t;


/* PC packet */
typedef struct {
    int32_t steer_raw;
    uint32_t speed_raw;
    uint8_t misc;
    uint32_t rx_tick_ms;
} CommEthernet_PcPacket_t;

void CommEthernet_Init(void);
bool CommEthernet_IsInitialized(void);

bool CommEthernet_ConsumeAsmsPacket( CommEthernet_AsmsPacket_t *packet );
bool CommEthernet_ConsumePcPacket( CommEthernet_PcPacket_t *packet );

uint8_t CommEthernet_ConsumeEstopMask(void);

uint32_t CommEthernet_GetLastAsmsRxTick(void);
uint32_t CommEthernet_GetLastPcRxTick(void);

#endif /* INC_COMM_ETHERNET_H_ */
