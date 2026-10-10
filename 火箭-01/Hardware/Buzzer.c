#include "stm32f10x.h"
#include "RocketConfig.h"
#include "Buzzer.h"

static uint8_t s_sequence_active = 0U;
static uint32_t s_sequence_start_ms = 0U;

static void Buzzer_Set(uint8_t active)
{
	if(active != 0U)
	{
		GPIO_SetBits(ROCKET_BUZZER_GPIO, ROCKET_BUZZER_GPIO_PIN);
	}
	else
	{
		GPIO_ResetBits(ROCKET_BUZZER_GPIO, ROCKET_BUZZER_GPIO_PIN);
	}
}

void Buzzer_Init(void)
{
	GPIO_InitTypeDef gpio_init;

	RCC_APB2PeriphClockCmd(ROCKET_BUZZER_GPIO_CLOCK, ENABLE);
	GPIO_ResetBits(ROCKET_BUZZER_GPIO, ROCKET_BUZZER_GPIO_PIN);
	gpio_init.GPIO_Pin = ROCKET_BUZZER_GPIO_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(ROCKET_BUZZER_GPIO, &gpio_init);
	Buzzer_Set(0U);
	s_sequence_active = 0U;
	s_sequence_start_ms = 0U;
}

void Buzzer_Task(uint32_t now_ms, uint8_t launch_ready, uint8_t launch_started)
{
	uint32_t elapsed_ms;
	uint32_t short_cycle_ms;

	/*
	 * Once the ready indication has started, complete the three short beeps
	 * and then keep the buzzer on throughout the launched mission.
	 * A readiness loss before launch still cancels and resets the sequence.
	 */
	if((launch_ready == 0U) && (launch_started == 0U))
	{
		Buzzer_Set(0U);
		s_sequence_active = 0U;
		return;
	}

	if(s_sequence_active == 0U)
	{
		s_sequence_active = 1U;
		s_sequence_start_ms = now_ms;
	}
	elapsed_ms = (uint32_t)(now_ms - s_sequence_start_ms);
	short_cycle_ms =
		ROCKET_BUZZER_SHORT_ON_MS + ROCKET_BUZZER_SHORT_OFF_MS;
	if(elapsed_ms < (3U * short_cycle_ms))
	{
		Buzzer_Set((uint8_t)((elapsed_ms % short_cycle_ms) <
							 ROCKET_BUZZER_SHORT_ON_MS));
		return;
	}

	Buzzer_Set(1U);
}
