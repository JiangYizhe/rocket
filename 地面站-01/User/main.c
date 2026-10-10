#include "stm32f10x.h"
#include "BaseConfig.h"
#include "BaseTime.h"
#include "BoardIO.h"
#include "GroundLink.h"

static uint8_t s_button_last_raw = 0U;
static uint8_t s_button_stable = 0U;
static uint8_t s_button_armed = 0U;
static uint32_t s_button_change_ms = 0U;

static uint8_t LaunchButton_Task(uint32_t now_ms, uint8_t launch_ready)
{
	uint8_t raw_pressed;

	raw_pressed = BoardIO_IsLaunchButtonPressed();
	if(raw_pressed != s_button_last_raw)
	{
		s_button_last_raw = raw_pressed;
		s_button_change_ms = now_ms;
	}

	if((raw_pressed != s_button_stable) &&
	   ((uint32_t)(now_ms - s_button_change_ms) >=
		BASE_BUTTON_DEBOUNCE_MS))
	{
		s_button_stable = raw_pressed;
		if(s_button_stable != 0U)
		{
			/*
			 * A press is consumed even if the link is not ready.  This prevents
			 * a button held during reconnection from firing later by itself.
			 */
			if((s_button_armed != 0U) && (launch_ready != 0U))
			{
				s_button_armed = 0U;
				return 1U;
			}
			s_button_armed = 0U;
		}
	}

	if((s_button_stable == 0U) &&
	   ((uint32_t)(now_ms - s_button_change_ms) >=
		BASE_BUTTON_RELEASE_ARM_MS))
	{
		s_button_armed = 1U;
	}
	return 0U;
}

int main(void)
{
	uint32_t now_ms;
	uint32_t launch_start_ms = 0U;
	uint8_t launch_ready;
	uint8_t launch_started = 0U;

	/* PC13 is forced low before any timers, UART, or LEDs are enabled. */
	BoardIO_Init();
	GroundLink_Init();
	BaseTime_Init();

	while(1)
	{
		now_ms = BaseTime_GetMs();
		GroundLink_Task(now_ms);
		launch_ready = GroundLink_IsReady();

		if(launch_started == 0U)
		{
			if(launch_ready != 0U)
			{
				BoardIO_SetAllLeds(
					(uint8_t)(((now_ms / BASE_LED_FLASH_PERIOD_MS) & 1U) != 0U));
			}
			else
			{
				BoardIO_SetAllLeds(0U);
			}

			if(LaunchButton_Task(now_ms, launch_ready) != 0U)
			{
				if(GroundLink_StartLaunch(now_ms) != 0U)
				{
					/* Local stage 1 and the remote countdown start together. */
					BoardIO_SetStage1Ignition(1U);
					launch_started = 1U;
					launch_start_ms = now_ms;
				}
			}
			continue;
		}

		if((uint32_t)(now_ms - launch_start_ms) >=
		   BASE_STAGE1_IGNITION_PULSE_MS)
		{
			BoardIO_SetStage1Ignition(0U);
		}

		if(GroundLink_IsLaunchAcknowledged() != 0U)
		{
			BoardIO_SetAllLeds(1U);
		}
		else if(GroundLink_HasLaunchFailed() != 0U)
		{
			BoardIO_SetAllLeds(
				(uint8_t)(((now_ms / BASE_ERROR_FLASH_PERIOD_MS) & 1U) != 0U));
		}
		else
		{
			BoardIO_SetAllLeds(
				(uint8_t)(((now_ms / BASE_LED_FLASH_PERIOD_MS) & 1U) != 0U));
		}
	}
}
