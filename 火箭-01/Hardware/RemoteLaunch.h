#ifndef __REMOTE_LAUNCH_H
#define __REMOTE_LAUNCH_H

#include <stdint.h>

void RemoteLaunch_Init(void);
void RemoteLaunch_Task(uint32_t now_ms, uint8_t flight_ready);
uint8_t RemoteLaunch_ConsumeRequest(void);
uint16_t RemoteLaunch_GetLaunchAgeMs(void);
uint8_t RemoteLaunch_IsLinkQualified(void);
uint8_t RemoteLaunch_IsReady(void);
uint8_t RemoteLaunch_HasStarted(void);

#endif
