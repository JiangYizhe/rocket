#ifndef __ROCKET_H
#define __ROCKET_H

#include <stdint.h>
#include "RocketState.h"

void Rocket_Init(void);
void Rocket_Task(uint32_t now_ms);
Rocket_State Rocket_GetState(void);
uint8_t Rocket_HasImuFault(void);
uint8_t Rocket_HasBarometerFault(void);

/* 保留原工程中有意义的开伞接口，供已有调用兼容。 */
void Rocket_DeployOutputInit(void);
void Rocket_DeployOutputSet(void);

#endif
