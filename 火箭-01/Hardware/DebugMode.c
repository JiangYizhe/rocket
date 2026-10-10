#include <stdint.h>

#include "stm32f10x.h"
#include "Delay.h"
#include "DebugConfig.h"
#include "W25Q64.h"
#include "FaultLED.h"
#include "Servo.h"
#include "IMU.h"
#include "FlightControl.h"
#include "DebugMode.h"

#if (DEBUG_SERVO_CONTROL_ENABLE != 0) && \
	(DEBUG_SERVO_CONTROL_ENABLE != 1)
#error "DEBUG_SERVO_CONTROL_ENABLE must be 0 or 1"
#endif

#if (DEBUG_FLASH_BYTES_PER_LINE == 0) || \
	(DEBUG_FLASH_BYTES_PER_LINE > W25Q64_PAGE_SIZE_BYTES)
#error "DEBUG_FLASH_BYTES_PER_LINE must be in range 1..256"
#endif

#if ((DEBUG_FLASH_DUMP_START_ADDRESS + DEBUG_FLASH_DUMP_SIZE_BYTES) > \
	W25Q64_SIZE_BYTES)
#error "Debug Flash dump range exceeds W25Q64 capacity"
#endif

/*============================== 私有变量 ==============================*/

static uint8_t s_flash_ready = 0U;
static uint32_t s_flash_address = DEBUG_FLASH_DUMP_START_ADDRESS;
static uint32_t s_last_flash_retry_ms = 0U;

#if DEBUG_SERVO_CONTROL_ENABLE
static IMU_Data s_debug_imu;
static uint32_t s_last_imu_ms = 0U;
static uint32_t s_last_imu_retry_ms = 0U;
static uint8_t s_control_running = 0U;
#endif

/*============================== 私有函数 ==============================*/

static uint8_t DebugMode_USART_WriteByte(uint8_t data)
{
	uint32_t timeout = DEBUG_USART_TIMEOUT_LOOPS;

	while(USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET)
	{
		if(timeout == 0U)
		{
			return 0U;
		}
		timeout--;
	}

	USART_SendData(DEBUG_USART, data);
	return 1U;
}

static void DebugMode_USART_WriteString(const char *text)
{
	if(text == 0)
	{
		return;
	}

	while(*text != '\0')
	{
		if(DebugMode_USART_WriteByte((uint8_t)*text) == 0U)
		{
			return;
		}
		text++;
	}
}

static void DebugMode_USART_WriteHexNibble(uint8_t value)
{
	static const char hex[] = "0123456789ABCDEF";

	(void)DebugMode_USART_WriteByte((uint8_t)hex[value & 0x0FU]);
}

static void DebugMode_USART_WriteHex8(uint8_t value)
{
	DebugMode_USART_WriteHexNibble((uint8_t)(value >> 4));
	DebugMode_USART_WriteHexNibble(value);
}

static void DebugMode_USART_WriteAddress(uint32_t address)
{
	uint8_t index;
	uint8_t shift;

	for(index = 0U; index < 6U; index++)
	{
		shift = (uint8_t)((5U - index) * 4U);
		DebugMode_USART_WriteHexNibble((uint8_t)(address >> shift));
	}
}

static void DebugMode_USART_WriteResult(W25Q64_Result result)
{
	DebugMode_USART_WriteString(" result=");
	DebugMode_USART_WriteHex8((uint8_t)result);
	DebugMode_USART_WriteString("\r\n");
}

static void DebugMode_USART_Init(void)
{
	GPIO_InitTypeDef gpio_init;
	USART_InitTypeDef usart_init;

	RCC_APB2PeriphClockCmd(DEBUG_USART_GPIO_CLOCK |
						  DEBUG_USART_CLOCK,
						  ENABLE);

	GPIO_StructInit(&gpio_init);
	gpio_init.GPIO_Pin = DEBUG_USART_TX_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(DEBUG_USART_GPIO, &gpio_init);

	USART_StructInit(&usart_init);
	usart_init.USART_BaudRate = DEBUG_USART_BAUDRATE;
	usart_init.USART_WordLength = USART_WordLength_8b;
	usart_init.USART_StopBits = USART_StopBits_1;
	usart_init.USART_Parity = USART_Parity_No;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Mode = USART_Mode_Tx;
	USART_Init(DEBUG_USART, &usart_init);
	USART_Cmd(DEBUG_USART, ENABLE);
}

static void DebugMode_FlashInit(void)
{
	W25Q64_Result result;

	result = W25Q64_Init();
	if(result == W25Q64_RESULT_OK)
	{
		s_flash_ready = 1U;
		DebugMode_USART_WriteString("W25Q64 JEDEC EF4017 OK\r\n");
	}
	else
	{
		s_flash_ready = 0U;
		DebugMode_USART_WriteString("W25Q64 INIT ERROR");
		DebugMode_USART_WriteResult(result);
	}
}

static void DebugMode_DumpFlashLine(void)
{
	uint8_t buffer[DEBUG_FLASH_BYTES_PER_LINE];
	W25Q64_Result result;
	uint16_t index;

	result = W25Q64_Read(s_flash_address,
						buffer,
						DEBUG_FLASH_BYTES_PER_LINE);
	if(result != W25Q64_RESULT_OK)
	{
		s_flash_ready = 0U;
		DebugMode_USART_WriteString("W25Q64 READ ERROR");
		DebugMode_USART_WriteResult(result);
		return;
	}

	DebugMode_USART_WriteAddress(s_flash_address);
	DebugMode_USART_WriteString(":");
	for(index = 0U; index < DEBUG_FLASH_BYTES_PER_LINE; index++)
	{
		DebugMode_USART_WriteString(" ");
		DebugMode_USART_WriteHex8(buffer[index]);
	}
	DebugMode_USART_WriteString("\r\n");

	s_flash_address += DEBUG_FLASH_BYTES_PER_LINE;
	if(s_flash_address >=
	   (DEBUG_FLASH_DUMP_START_ADDRESS + DEBUG_FLASH_DUMP_SIZE_BYTES))
	{
		s_flash_address = DEBUG_FLASH_DUMP_START_ADDRESS;
		DebugMode_USART_WriteString("--- W25Q64 DUMP RESTART ---\r\n");
	}
}

#if DEBUG_SERVO_CONTROL_ENABLE
static void DebugMode_ControlInit(void)
{
	Servo_Init();
	FlightControl_Init();
	(void)IMU_Init();

	s_last_imu_ms = 0U;
	s_last_imu_retry_ms = 0U;
	s_control_running = 0U;
}

static void DebugMode_ControlTask(uint32_t now_ms)
{
	uint8_t sample_valid;

	if((IMU_IsInitialized() == 0U) &&
	   ((uint32_t)(now_ms - s_last_imu_retry_ms) >=
		DEBUG_IMU_RETRY_PERIOD_MS))
	{
		s_last_imu_retry_ms = now_ms;
		s_last_imu_ms = now_ms;
		(void)IMU_Init();
	}

	if((uint32_t)(now_ms - s_last_imu_ms) < DEBUG_IMU_PERIOD_MS)
	{
		return;
	}

	s_last_imu_ms = now_ms;
	sample_valid = IMU_Read(&s_debug_imu, now_ms);
	if(now_ms < DEBUG_SERVO_CONTROL_START_MS)
	{
		return;
	}

	if(sample_valid != 0U)
	{
		FlightControl_Update(&s_debug_imu, now_ms, 0.0f);
		s_control_running = 1U;
	}
	else if(s_control_running != 0U)
	{
		FlightControl_Stop();
		s_control_running = 0U;
	}
}
#endif

/*============================== 公开函数 ==============================*/

uint8_t DebugMode_IsRequested(void)
{
	GPIO_InitTypeDef gpio_init;

	RCC_APB2PeriphClockCmd(DEBUG_JUMPER_GPIO_CLOCK, ENABLE);

	/* PC10先预装高电平，PC11使用内部下拉；短接后PC11读取为高�?*/
	GPIO_SetBits(DEBUG_JUMPER_GPIO, DEBUG_JUMPER_OUTPUT_PIN);
	GPIO_ResetBits(DEBUG_JUMPER_GPIO, DEBUG_JUMPER_INPUT_PIN);

	GPIO_StructInit(&gpio_init);
	gpio_init.GPIO_Pin = DEBUG_JUMPER_OUTPUT_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(DEBUG_JUMPER_GPIO, &gpio_init);

	gpio_init.GPIO_Pin = DEBUG_JUMPER_INPUT_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(DEBUG_JUMPER_GPIO, &gpio_init);

	Delay_us(10U);
	return (GPIO_ReadInputDataBit(DEBUG_JUMPER_GPIO,
								  DEBUG_JUMPER_INPUT_PIN) != Bit_RESET) ?
		   1U : 0U;
}

void DebugMode_Init(void)
{
	GPIO_InitTypeDef gpio_init;

	RCC_APB2PeriphClockCmd(DEBUG_LED_GPIO_CLOCK, ENABLE);
	GPIO_SetBits(DEBUG_LED_GPIO, DEBUG_LED_PIN);
	GPIO_StructInit(&gpio_init);
	gpio_init.GPIO_Pin = DEBUG_LED_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(DEBUG_LED_GPIO, &gpio_init);

	DebugMode_USART_Init();
	DebugMode_USART_WriteString("\r\n=== PHYSICAL DEBUG MODE ===\r\n");
	DebugMode_USART_WriteString("JUMPER PC10 -> PC11\r\n");

	s_flash_address = DEBUG_FLASH_DUMP_START_ADDRESS;
	s_last_flash_retry_ms = 0U;
	DebugMode_FlashInit();

#if DEBUG_SERVO_CONTROL_ENABLE
	DebugMode_USART_WriteString("SERVO CONTROL ON, START AT 1000ms\r\n");
	DebugMode_ControlInit();
#else
	DebugMode_USART_WriteString("SERVO CONTROL OFF\r\n");
#endif
}

void DebugMode_Task(uint32_t now_ms)
{
	uint8_t fault_code;

#if DEBUG_SERVO_CONTROL_ENABLE
	DebugMode_ControlTask(now_ms);
#endif

	if(s_flash_ready == 0U)
	{
		fault_code = FAULT_LED_CODE_STORAGE;
	}
#if DEBUG_SERVO_CONTROL_ENABLE
	else if((IMU_IsInitialized() == 0U) ||
			(IMU_GetStatus() != IMU_STATUS_READY) ||
			(s_debug_imu.valid == 0U))
	{
		fault_code = FAULT_LED_CODE_IMU;
	}
#endif
	else
	{
		fault_code = FAULT_LED_CODE_NONE;
	}
	FaultLED_Task(now_ms, fault_code);

	if(s_flash_ready == 0U)
	{
		if((uint32_t)(now_ms - s_last_flash_retry_ms) >=
		   DEBUG_FLASH_RETRY_PERIOD_MS)
		{
			s_last_flash_retry_ms = now_ms;
			DebugMode_FlashInit();
		}
		return;
	}

	DebugMode_DumpFlashLine();
}
