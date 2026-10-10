#include <stdint.h>

#include "stm32f10x.h"
#include "Delay.h"
#include "RocketConfig.h"
#include "W25Q64.h"

/*============================== 私有宏 ==============================*/

#define W25Q64_CMD_WRITE_ENABLE            0x06U
#define W25Q64_CMD_READ_STATUS1            0x05U
#define W25Q64_CMD_READ_DATA               0x03U
#define W25Q64_CMD_PAGE_PROGRAM            0x02U
#define W25Q64_CMD_SECTOR_ERASE_4K         0x20U
#define W25Q64_CMD_BLOCK_ERASE_32K         0x52U
#define W25Q64_CMD_BLOCK_ERASE_64K         0xD8U
#define W25Q64_CMD_CHIP_ERASE              0xC7U
#define W25Q64_CMD_JEDEC_ID                0x9FU
#define W25Q64_CMD_RELEASE_POWER_DOWN       0xABU
#define W25Q64_CMD_ENABLE_RESET            0x66U
#define W25Q64_CMD_RESET_DEVICE            0x99U

#define W25Q64_STATUS_BUSY                 0x01U
#define W25Q64_STATUS_WEL                  0x02U

#define W25Q64_SPI_TIMEOUT_LOOPS           100000U
#define W25Q64_COMMAND_READY_TIMEOUT_MS    100U

/*
 * 下列超时均略大于数据手册最大值：
 * Page Program 3ms、4KB 400ms、32KB 1600ms、64KB 2000ms、Chip 100s。
 */
#define W25Q64_PROGRAM_TIMEOUT_MS          5U
#define W25Q64_SECTOR_ERASE_TIMEOUT_MS     500U
#define W25Q64_BLOCK32_ERASE_TIMEOUT_MS    2000U
#define W25Q64_BLOCK64_ERASE_TIMEOUT_MS    2500U
#define W25Q64_CHIP_ERASE_TIMEOUT_MS       110000U

/*============================== 私有函数 ==============================*/

static void W25Q64_CS_Low(void)
{
	GPIO_ResetBits(ROCKET_FLASH_CS_GPIO, ROCKET_FLASH_CS_PIN);
}

static void W25Q64_CS_High(void)
{
	GPIO_SetBits(ROCKET_FLASH_CS_GPIO, ROCKET_FLASH_CS_PIN);
}

static uint8_t W25Q64_SPI_WaitFlag(uint16_t flag, FlagStatus expected)
{
	uint32_t timeout = W25Q64_SPI_TIMEOUT_LOOPS;

	while(SPI_I2S_GetFlagStatus(ROCKET_FLASH_SPI, flag) != expected)
	{
		if(timeout == 0U)
		{
			return 0U;
		}
		timeout--;
	}

	return 1U;
}

static uint8_t W25Q64_SPI_TransferByte(uint8_t tx_data, uint8_t *rx_data)
{
	if(rx_data == 0)
	{
		return 0U;
	}

	if(W25Q64_SPI_WaitFlag(SPI_I2S_FLAG_TXE, SET) == 0U)
	{
		return 0U;
	}

	SPI_I2S_SendData(ROCKET_FLASH_SPI, tx_data);
	if(W25Q64_SPI_WaitFlag(SPI_I2S_FLAG_RXNE, SET) == 0U)
	{
		return 0U;
	}

	*rx_data = (uint8_t)SPI_I2S_ReceiveData(ROCKET_FLASH_SPI);
	return 1U;
}

static void W25Q64_SPI_Recover(void)
{
	volatile uint16_t discard;

	W25Q64_CS_High();
	SPI_Cmd(ROCKET_FLASH_SPI, DISABLE);
	discard = ROCKET_FLASH_SPI->DR;
	discard = ROCKET_FLASH_SPI->SR;
	(void)discard;
	SPI_Cmd(ROCKET_FLASH_SPI, ENABLE);
}

static W25Q64_Result W25Q64_EndTransaction(W25Q64_Result result)
{
	if((result == W25Q64_RESULT_OK) &&
	   (W25Q64_SPI_WaitFlag(SPI_I2S_FLAG_BSY, RESET) == 0U))
	{
		result = W25Q64_RESULT_BUS_ERROR;
	}

	if(result == W25Q64_RESULT_OK)
	{
		W25Q64_CS_High();
	}
	else
	{
		W25Q64_SPI_Recover();
	}

	return result;
}

static W25Q64_Result W25Q64_SendByte(uint8_t tx_data, uint8_t *rx_data)
{
	if(W25Q64_SPI_TransferByte(tx_data, rx_data) == 0U)
	{
		return W25Q64_RESULT_BUS_ERROR;
	}

	return W25Q64_RESULT_OK;
}

static W25Q64_Result W25Q64_SendAddress(uint32_t address)
{
	W25Q64_Result result;
	uint8_t discard;

	result = W25Q64_SendByte((uint8_t)(address >> 16), &discard);
	if(result == W25Q64_RESULT_OK)
	{
		result = W25Q64_SendByte((uint8_t)(address >> 8), &discard);
	}
	if(result == W25Q64_RESULT_OK)
	{
		result = W25Q64_SendByte((uint8_t)address, &discard);
	}

	return result;
}

static uint8_t W25Q64_IsRangeValid(uint32_t address, uint32_t length)
{
	if(address >= W25Q64_SIZE_BYTES)
	{
		return 0U;
	}
	if(length > (W25Q64_SIZE_BYTES - address))
	{
		return 0U;
	}

	return 1U;
}

static W25Q64_Result W25Q64_SendSimpleCommand(uint8_t command)
{
	W25Q64_Result result;
	uint8_t discard;

	W25Q64_CS_Low();
	result = W25Q64_SendByte(command, &discard);
	return W25Q64_EndTransaction(result);
}

static W25Q64_Result W25Q64_WriteEnable(void)
{
	W25Q64_Result result;
	uint8_t status;

	result = W25Q64_SendSimpleCommand(W25Q64_CMD_WRITE_ENABLE);
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}

	result = W25Q64_ReadStatus1(&status);
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}
	if((status & W25Q64_STATUS_WEL) == 0U)
	{
		return W25Q64_RESULT_WRITE_ENABLE_FAILED;
	}

	return W25Q64_RESULT_OK;
}

static W25Q64_Result W25Q64_EraseAddressed(uint8_t command,
										   uint32_t address,
										   uint32_t alignment,
										   uint32_t timeout_ms)
{
	W25Q64_Result result;
	uint8_t discard;

	if((address >= W25Q64_SIZE_BYTES) ||
	   ((address & (alignment - 1U)) != 0U))
	{
		return W25Q64_RESULT_INVALID_ARGUMENT;
	}

	result = W25Q64_WaitReady(W25Q64_COMMAND_READY_TIMEOUT_MS);
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}
	result = W25Q64_WriteEnable();
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}

	W25Q64_CS_Low();
	result = W25Q64_SendByte(command, &discard);
	if(result == W25Q64_RESULT_OK)
	{
		result = W25Q64_SendAddress(address);
	}
	result = W25Q64_EndTransaction(result);
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}

	return W25Q64_WaitReady(timeout_ms);
}

/*============================== 公开函数 ==============================*/

W25Q64_Result W25Q64_Init(void)
{
	GPIO_InitTypeDef gpio_init;
	SPI_InitTypeDef spi_init;
	W25Q64_Result result;
	uint8_t manufacturer_id;
	uint8_t memory_type_id;
	uint8_t capacity_id;

	RCC_APB2PeriphClockCmd(ROCKET_FLASH_SPI_GPIO_CLOCK |
						  ROCKET_FLASH_CS_GPIO_CLOCK,
						  ENABLE);
	RCC_APB1PeriphClockCmd(ROCKET_FLASH_SPI_CLOCK, ENABLE);

	/* 先预装高电平，再切换PC1为输出，避免片选初始化毛刺。 */
	W25Q64_CS_High();
	GPIO_StructInit(&gpio_init);
	gpio_init.GPIO_Pin = ROCKET_FLASH_CS_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ROCKET_FLASH_CS_GPIO, &gpio_init);

	gpio_init.GPIO_Pin = ROCKET_FLASH_SPI_SCK_PIN |
						 ROCKET_FLASH_SPI_MOSI_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ROCKET_FLASH_SPI_GPIO, &gpio_init);

	gpio_init.GPIO_Pin = ROCKET_FLASH_SPI_MISO_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(ROCKET_FLASH_SPI_GPIO, &gpio_init);

	SPI_I2S_DeInit(ROCKET_FLASH_SPI);
	SPI_StructInit(&spi_init);
	spi_init.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi_init.SPI_Mode = SPI_Mode_Master;
	spi_init.SPI_DataSize = SPI_DataSize_8b;
	spi_init.SPI_CPOL = SPI_CPOL_High;
	spi_init.SPI_CPHA = SPI_CPHA_2Edge;
	spi_init.SPI_NSS = SPI_NSS_Soft;
	spi_init.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	spi_init.SPI_FirstBit = SPI_FirstBit_MSB;
	spi_init.SPI_CRCPolynomial = 7U;
	SPI_Init(ROCKET_FLASH_SPI, &spi_init);
	SPI_NSSInternalSoftwareConfig(ROCKET_FLASH_SPI,
								  SPI_NSSInternalSoft_Set);
	SPI_Cmd(ROCKET_FLASH_SPI, ENABLE);

	/* 兼容芯片此前处于Power-down状态，tRES1最大3us。 */
	result = W25Q64_SendSimpleCommand(W25Q64_CMD_RELEASE_POWER_DOWN);
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}
	Delay_us(5U);

	result = W25Q64_ReadJedecId(&manufacturer_id,
							   &memory_type_id,
							   &capacity_id);
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}
	if((manufacturer_id != W25Q64_MANUFACTURER_ID) ||
	   (memory_type_id != W25Q64_MEMORY_TYPE_ID) ||
	   (capacity_id != W25Q64_CAPACITY_ID))
	{
		return W25Q64_RESULT_DEVICE_ID_MISMATCH;
	}

	return W25Q64_RESULT_OK;
}

W25Q64_Result W25Q64_Reset(void)
{
	W25Q64_Result result;

	result = W25Q64_SendSimpleCommand(W25Q64_CMD_ENABLE_RESET);
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}
	result = W25Q64_SendSimpleCommand(W25Q64_CMD_RESET_DEVICE);
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}

	/* 数据手册tRST最大30us，留出裕量。 */
	Delay_us(50U);
	return W25Q64_RESULT_OK;
}

W25Q64_Result W25Q64_ReadJedecId(uint8_t *manufacturer_id,
								uint8_t *memory_type_id,
								uint8_t *capacity_id)
{
	W25Q64_Result result;
	uint8_t discard;

	if((manufacturer_id == 0) ||
	   (memory_type_id == 0) ||
	   (capacity_id == 0))
	{
		return W25Q64_RESULT_INVALID_ARGUMENT;
	}

	W25Q64_CS_Low();
	result = W25Q64_SendByte(W25Q64_CMD_JEDEC_ID, &discard);
	if(result == W25Q64_RESULT_OK)
	{
		result = W25Q64_SendByte(0xFFU, manufacturer_id);
	}
	if(result == W25Q64_RESULT_OK)
	{
		result = W25Q64_SendByte(0xFFU, memory_type_id);
	}
	if(result == W25Q64_RESULT_OK)
	{
		result = W25Q64_SendByte(0xFFU, capacity_id);
	}

	return W25Q64_EndTransaction(result);
}

W25Q64_Result W25Q64_ReadStatus1(uint8_t *status)
{
	W25Q64_Result result;
	uint8_t discard;

	if(status == 0)
	{
		return W25Q64_RESULT_INVALID_ARGUMENT;
	}

	W25Q64_CS_Low();
	result = W25Q64_SendByte(W25Q64_CMD_READ_STATUS1, &discard);
	if(result == W25Q64_RESULT_OK)
	{
		result = W25Q64_SendByte(0xFFU, status);
	}

	return W25Q64_EndTransaction(result);
}

W25Q64_Result W25Q64_WaitReady(uint32_t timeout_ms)
{
	W25Q64_Result result;
	uint8_t status;

	for(;;)
	{
		result = W25Q64_ReadStatus1(&status);
		if(result != W25Q64_RESULT_OK)
		{
			return result;
		}
		if((status & W25Q64_STATUS_BUSY) == 0U)
		{
			return W25Q64_RESULT_OK;
		}
		if(timeout_ms == 0U)
		{
			return W25Q64_RESULT_TIMEOUT;
		}

		Delay_ms(1U);
		timeout_ms--;
	}
}

W25Q64_Result W25Q64_Read(uint32_t address,
						 uint8_t *data,
						 uint32_t length)
{
	W25Q64_Result result;
	uint8_t discard;
	uint32_t index;

	if(length == 0U)
	{
		return W25Q64_RESULT_OK;
	}
	if((data == 0) || (W25Q64_IsRangeValid(address, length) == 0U))
	{
		return W25Q64_RESULT_INVALID_ARGUMENT;
	}

	result = W25Q64_WaitReady(W25Q64_COMMAND_READY_TIMEOUT_MS);
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}

	W25Q64_CS_Low();
	result = W25Q64_SendByte(W25Q64_CMD_READ_DATA, &discard);
	if(result == W25Q64_RESULT_OK)
	{
		result = W25Q64_SendAddress(address);
	}
	for(index = 0U;
		(index < length) && (result == W25Q64_RESULT_OK);
		index++)
	{
		result = W25Q64_SendByte(0xFFU, &data[index]);
	}

	return W25Q64_EndTransaction(result);
}

W25Q64_Result W25Q64_PageProgram(uint32_t address,
								const uint8_t *data,
								uint16_t length)
{
	W25Q64_Result result;
	uint8_t discard;
	uint16_t index;

	if((data == 0) ||
	   (length == 0U) ||
	   (length > W25Q64_PAGE_SIZE_BYTES) ||
	   (W25Q64_IsRangeValid(address, length) == 0U) ||
	   (((address & (W25Q64_PAGE_SIZE_BYTES - 1U)) + length) >
		W25Q64_PAGE_SIZE_BYTES))
	{
		return W25Q64_RESULT_INVALID_ARGUMENT;
	}

	result = W25Q64_WaitReady(W25Q64_COMMAND_READY_TIMEOUT_MS);
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}
	result = W25Q64_WriteEnable();
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}

	W25Q64_CS_Low();
	result = W25Q64_SendByte(W25Q64_CMD_PAGE_PROGRAM, &discard);
	if(result == W25Q64_RESULT_OK)
	{
		result = W25Q64_SendAddress(address);
	}
	for(index = 0U;
		(index < length) && (result == W25Q64_RESULT_OK);
		index++)
	{
		result = W25Q64_SendByte(data[index], &discard);
	}
	result = W25Q64_EndTransaction(result);
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}

	return W25Q64_WaitReady(W25Q64_PROGRAM_TIMEOUT_MS);
}

W25Q64_Result W25Q64_Write(uint32_t address,
						  const uint8_t *data,
						  uint32_t length)
{
	W25Q64_Result result;
	uint32_t page_remaining;
	uint16_t chunk;

	if(length == 0U)
	{
		return W25Q64_RESULT_OK;
	}
	if((data == 0) || (W25Q64_IsRangeValid(address, length) == 0U))
	{
		return W25Q64_RESULT_INVALID_ARGUMENT;
	}

	while(length != 0U)
	{
		page_remaining =
			W25Q64_PAGE_SIZE_BYTES -
			(address & (W25Q64_PAGE_SIZE_BYTES - 1U));
		chunk = (uint16_t)((length < page_remaining) ?
						   length : page_remaining);

		result = W25Q64_PageProgram(address, data, chunk);
		if(result != W25Q64_RESULT_OK)
		{
			return result;
		}

		address += chunk;
		data += chunk;
		length -= chunk;
	}

	return W25Q64_RESULT_OK;
}

W25Q64_Result W25Q64_EraseSector4K(uint32_t address)
{
	return W25Q64_EraseAddressed(W25Q64_CMD_SECTOR_ERASE_4K,
								 address,
								 W25Q64_SECTOR_SIZE_BYTES,
								 W25Q64_SECTOR_ERASE_TIMEOUT_MS);
}

W25Q64_Result W25Q64_EraseBlock32K(uint32_t address)
{
	return W25Q64_EraseAddressed(W25Q64_CMD_BLOCK_ERASE_32K,
								 address,
								 W25Q64_BLOCK32_SIZE_BYTES,
								 W25Q64_BLOCK32_ERASE_TIMEOUT_MS);
}

W25Q64_Result W25Q64_EraseBlock64K(uint32_t address)
{
	return W25Q64_EraseAddressed(W25Q64_CMD_BLOCK_ERASE_64K,
								 address,
								 W25Q64_BLOCK64_SIZE_BYTES,
								 W25Q64_BLOCK64_ERASE_TIMEOUT_MS);
}

W25Q64_Result W25Q64_EraseChip(void)
{
	W25Q64_Result result;

	result = W25Q64_WaitReady(W25Q64_COMMAND_READY_TIMEOUT_MS);
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}
	result = W25Q64_WriteEnable();
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}
	result = W25Q64_SendSimpleCommand(W25Q64_CMD_CHIP_ERASE);
	if(result != W25Q64_RESULT_OK)
	{
		return result;
	}

	return W25Q64_WaitReady(W25Q64_CHIP_ERASE_TIMEOUT_MS);
}
