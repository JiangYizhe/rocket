#ifndef __BASE_TIME_H
#define __BASE_TIME_H

#include <stdint.h>

void BaseTime_Init(void);
uint32_t BaseTime_GetMs(void);
void BaseTime_TickIrqHandler(void);

#endif
