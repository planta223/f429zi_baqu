/*
 * comm_can.h
 *
 *  Created on: Aug 27, 2026
 *      Author: kyubeom
 */

#ifndef INC_COMM_CAN_H_
#define INC_COMM_CAN_H_

#include <stdint.h>
#include <stdbool.h>

/* Steering Request*/
typedef struct {
    int16_t steer_raw;
    uint8_t flags;
    uint32_t rx_tick_ms;
} CommCan_SteerRequest_t;


/*  Steering Status */
typedef struct
{
    int16_t actual_raw;
    int16_t target_raw;
    uint8_t flags;
} CommCan_SteerStatus_t;


void CommCan_Init(void);
bool CommCan_IsInitialized(void);

bool CommCan_ConsumeSteerRequest( CommCan_SteerRequest_t *request );
bool CommCan_ConsumeEstopRequest(void);

uint32_t CommCan_GetLastRequestRxTick(void);

bool CommCan_SendSteerStatus( const CommCan_SteerStatus_t *status );

#endif /* INC_COMM_CAN_H_ */
