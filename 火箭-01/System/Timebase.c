#include "stm32f10x.h"
#include "RocketConfig.h"
#include "Timebase.h"

/*============================== 私有变量 ==============================*/

static volatile uint32_t s_time_ms = 0U;

/*============================== 公开函数 ==============================*/

void Timebase_Init(void)
{
	TIM_TimeBaseInitTypeDef tim_init;
	NVIC_InitTypeDef nvic_init;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	TIM_InternalClockConfig(TIM1);

	TIM_TimeBaseStructInit(&tim_init);
	tim_init.TIM_Prescaler = 72U - 1U;
	tim_init.TIM_Period = (1000U * ROCKET_TIMEBASE_PERIOD_MS) - 1U;
	tim_init.TIM_CounterMode = TIM_CounterMode_Up;
	tim_init.TIM_ClockDivision = TIM_CKD_DIV1;
	tim_init.TIM_RepetitionCounter = 0U;
	TIM_TimeBaseInit(TIM1, &tim_init);
	TIM_SelectOnePulseMode(TIM1, TIM_OPMode_Repetitive);
	TIM_SetCounter(TIM1, 0U);
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	NVIC_ClearPendingIRQ(TIM1_UP_IRQn);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	nvic_init.NVIC_IRQChannel = TIM1_UP_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 1U;
	nvic_init.NVIC_IRQChannelSubPriority = 1U;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);

	s_time_ms = 0U;
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
}

uint32_t Timebase_GetMs(void)
{
	return s_time_ms;
}

void Timebase_TickIrqHandler(void)
{
	s_time_ms += ROCKET_TIMEBASE_PERIOD_MS;
}
