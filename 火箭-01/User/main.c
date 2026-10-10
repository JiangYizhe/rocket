#include "stm32f10x.h"
#include "Ignition.h"
#include "FaultLED.h"
#include "Timebase.h"
#include "DebugMode.h"
#include "FlightLog.h"
#include "Rocket.h"

int main(void)
{
	FlightLog_Status log_status;
	uint32_t now_ms;
	uint8_t fault_code;

	/* 点火输出必须最先初始化并锁定为低电平。 */
	Ignition_Init();
	FaultLED_Init();

	if(DebugMode_IsRequested() != 0U)
	{
		Timebase_Init();
		DebugMode_Init();
		while(1)
		{
			DebugMode_Task(Timebase_GetMs());
		}
	}

	/*
	 * 正常模式先预擦除日志区域，再启动任务计时，避免Flash擦除时间
	 * 占用发射流程的时间窗口。
	 */
	log_status = FlightLog_Init();
	FaultLED_Set((log_status == FLIGHT_LOG_STATUS_FLASH_ERROR) ? 1U : 0U);
	Timebase_Init();
	Rocket_Init();

	while(1)
	{
		now_ms = Timebase_GetMs();
		Rocket_Task(now_ms);
		if(FlightLog_GetStatus() == FLIGHT_LOG_STATUS_FLASH_ERROR)
		{
			fault_code = FAULT_LED_CODE_STORAGE;
		}
		else if(Rocket_HasImuFault() != 0U)
		{
			fault_code = FAULT_LED_CODE_IMU;
		}
		else if(Rocket_HasBarometerFault() != 0U)
		{
			fault_code = FAULT_LED_CODE_BAROMETER;
		}
		else
		{
			fault_code = FAULT_LED_CODE_NONE;
		}
		FaultLED_Task(now_ms, fault_code);
	}
}
