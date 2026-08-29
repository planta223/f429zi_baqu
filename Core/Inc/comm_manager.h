/*
 * comm_manager.h
 *
 *  Created on: Aug 27, 2026
 *      Author: kyubeom
 */

#ifndef INC_COMM_MANAGER_H_
#define INC_COMM_MANAGER_H_

#include <stdint.h>


/* Steering mode */
typedef enum {
    COMM_STEER_MODE_NONE   = 0U,
    COMM_STEER_MODE_AUTO   = 1U,
    COMM_STEER_MODE_MANUAL = 2U,
    COMM_STEER_MODE_ESTOP  = 3U
} CommManager_SteerMode_t;


/*  현재 실제 target을 적용한 source */
typedef enum {
    COMM_SOURCE_NONE = 0U,

    COMM_SOURCE_ETHERNET_ASMS,
    COMM_SOURCE_ETHERNET_PC,
    COMM_SOURCE_CAN
} CommManager_Source_t;


/* E-Stop source */
#define COMM_ESTOP_SOURCE_ETHERNET_ASMS   (1U << 0)
#define COMM_ESTOP_SOURCE_ETHERNET_PC     (1U << 1)
#define COMM_ESTOP_SOURCE_CAN             (1U << 2)


/* Manager state */
typedef struct {
    CommManager_SteerMode_t mode;
    CommManager_Source_t active_source;

    uint32_t last_valid_ethernet_tick;
    uint32_t last_valid_can_tick;

    uint8_t ethernet_timeout;
    uint8_t can_timeout;

    uint8_t last_estop_source_mask;
} CommManager_State_t;

void CommManager_Init(void);
void CommManager_Update(void);
CommManager_SteerMode_t CommManager_GetMode(void);
CommManager_Source_t CommManager_GetActiveSource(void);
CommManager_State_t CommManager_GetState(void);

#endif /* INC_COMM_MANAGER_H_ */
