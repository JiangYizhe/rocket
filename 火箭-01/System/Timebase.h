#ifndef __TIMEBASE_H
#define __TIMEBASE_H

#include <stdint.h>

void Timebase_Init(void);
uint32_t Timebase_GetMs(void);
void Timebase_TickIrqHandler(void);

#endif
