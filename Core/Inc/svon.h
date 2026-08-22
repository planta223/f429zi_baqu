/*
 * svon.h
 *
 *  Created on: Aug 22, 2026
 *      Author: kyubeom
 */

#ifndef INC_SVON_H_
#define INC_SVON_H_

#include <stdint.h>

void SVON_Init(void);

void SVON_Enable(void);
void SVON_Disable(void);

uint8_t SVON_IsEnabled(void);

#endif /* INC_SVON_H_ */
