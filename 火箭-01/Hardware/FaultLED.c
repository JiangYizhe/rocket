#include "stm32f10x.h"
#include "RocketConfig.h"
#include "FaultLED.h"

/*============================== 私有宏 ==============================*/

#define FAULT_LED_ON_TIME_MS                150U
#define FAULT_LED_PULSE_PERIOD_MS           300U
#define FAULT_LED_PATTERN_PERIOD_MS         1500U

/*============================== 私有变量 ==============================*/

static uint8_t s_fault_code = FAULT_LED_CODE_NONE;
static uint32_t s_pattern_start_ms = 0U;

/*============================== 公开函数 ==============================*/

void FaultLED_Init(void)
{
	GPIO_InitTypeDef gpio_init;

	RCC_APB2PeriphClockCmd(ROCKET_FAULT_LED_GPIO_CLOCK, ENABLE);

	/* 先预装低电平，再切换为输出，确保上电默认熄灭。 */
	GPIO_ResetBits(ROCKET_FAULT_LED_GPIO, ROCKET_FAULT_LED_GPIO_PIN);
	GPIO_StructInit(&gpio_init);
	gpio_init.GPIO_Pin = ROCKET_FAULT_LED_GPIO_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ROCKET_FAULT_LED_GPIO, &gpio_init);
	GPIO_ResetBits(ROCKET_FAULT_LED_GPIO, ROCKET_FAULT_LED_GPIO_PIN);
	s_fault_code = FAULT_LED_CODE_NONE;
	s_pattern_start_ms = 0U;
}

void FaultLED_Set(uint8_t active)
{
	if(active != 0U)
	{
		GPIO_SetBits(ROCKET_FAULT_LED_GPIO, ROCKET_FAULT_LED_GPIO_PIN);
	}
	else
	{
		GPIO_ResetBits(ROCKET_FAULT_LED_GPIO, ROCKET_FAULT_LED_GPIO_PIN);
	}
}

void FaultLED_Task(uint32_t now_ms, uint8_t fault_code)
{
	uint32_t phase_ms;
	uint32_t active_time_ms;

	if(fault_code > FAULT_LED_CODE_BAROMETER)
	{
		fault_code = FAULT_LED_CODE_BAROMETER;
	}
	if(fault_code != s_fault_code)
	{
		s_fault_code = fault_code;
		s_pattern_start_ms = now_ms;
	}
	if(fault_code == FAULT_LED_CODE_NONE)
	{
		FaultLED_Set(0U);
		return;
	}

	phase_ms =
		(uint32_t)(now_ms - s_pattern_start_ms) %
		FAULT_LED_PATTERN_PERIOD_MS;
	active_time_ms =
		(uint32_t)fault_code * FAULT_LED_PULSE_PERIOD_MS;
	if((phase_ms < active_time_ms) &&
	   ((phase_ms % FAULT_LED_PULSE_PERIOD_MS) <
		FAULT_LED_ON_TIME_MS))
	{
		FaultLED_Set(1U);
	}
	else
	{
		FaultLED_Set(0U);
	}
}
