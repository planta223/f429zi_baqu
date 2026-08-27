/*
 * comm_ethernet.h
 *
 *  Created on: Aug 27, 2026
 *      Author: kyubeom
 */

#ifndef INC_COMM_ETHERNET_H_
#define INC_COMM_ETHERNET_H_

void CommEthernet_Init(void);
void CommEthernet_Process(void);

bool CommEthernet_HasAsmsPacket(void);
CommEthernet_AsmsPacket_t CommEthernet_GetAsmsPacket(void);

bool CommEthernet_HasPcPacket(void);
CommEthernet_PcPacket_t CommEthernet_GetPcPacket(void);

uint32_t CommEthernet_GetLastRxTick(void);
bool CommEthernet_IsInitialized(void);

#endif /* INC_COMM_ETHERNET_H_ */
