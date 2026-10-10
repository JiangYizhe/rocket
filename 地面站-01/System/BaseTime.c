#include "stm32f10x.h"
#include "BaseTime.h"

static volatile uint32_t s_time_ms = 0U;

void BaseTime_Init(void)
{
	s_time_ms = 0U;
	(void)SysTick_Config(SystemCoreClock / 1000U);
}

uint32_t BaseTime_GetMs(void)
{
	return s_time_ms;
}

void BaseTime_TickIrqHandler(void)
{
	s_time_ms++;
}
