#ifndef __FAULT_LED_H
#define __FAULT_LED_H

#include <stdint.h>

#define FAULT_LED_CODE_NONE                 0U
#define FAULT_LED_CODE_IMU                  1U
#define FAULT_LED_CODE_STORAGE              2U
#define FAULT_LED_CODE_BAROMETER            3U

void FaultLED_Init(void);
void FaultLED_Set(uint8_t active);
void FaultLED_Task(uint32_t now_ms, uint8_t fault_code);

#endif
