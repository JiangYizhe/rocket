#ifndef __BOARD_IO_H
#define __BOARD_IO_H

#include <stdint.h>

void BoardIO_Init(void);
void BoardIO_SetAllLeds(uint8_t active);
void BoardIO_SetStage1Ignition(uint8_t active);
uint8_t BoardIO_IsLaunchButtonPressed(void);

#endif
