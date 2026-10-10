#ifndef __BUZZER_H
#define __BUZZER_H

#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_Task(uint32_t now_ms, uint8_t launch_ready, uint8_t launch_started);

#endif
