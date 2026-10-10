#ifndef __GROUND_LINK_H
#define __GROUND_LINK_H

#include <stdint.h>

void GroundLink_Init(void);
void GroundLink_Task(uint32_t now_ms);
uint8_t GroundLink_IsReady(void);
uint8_t GroundLink_StartLaunch(uint32_t now_ms);
uint8_t GroundLink_HasStarted(void);
uint8_t GroundLink_IsLaunchAcknowledged(void);
uint8_t GroundLink_HasLaunchFailed(void);

#endif
