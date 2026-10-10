#ifndef __DEBUG_MODE_H
#define __DEBUG_MODE_H

#include <stdint.h>

uint8_t DebugMode_IsRequested(void);
void DebugMode_Init(void);
void DebugMode_Task(uint32_t now_ms);

#endif
