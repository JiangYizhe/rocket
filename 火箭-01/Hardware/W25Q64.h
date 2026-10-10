#ifndef __W25Q64_H
#define __W25Q64_H

#include <stdint.h>

#define W25Q64_MANUFACTURER_ID             0xEFU
#define W25Q64_MEMORY_TYPE_ID              0x40U
#define W25Q64_CAPACITY_ID                 0x17U

#define W25Q64_SIZE_BYTES                  0x00800000UL
#define W25Q64_PAGE_SIZE_BYTES             256U
#define W25Q64_SECTOR_SIZE_BYTES           4096UL
#define W25Q64_BLOCK32_SIZE_BYTES          32768UL
#define W25Q64_BLOCK64_SIZE_BYTES          65536UL

typedef enum
{
	W25Q64_RESULT_OK = 0,
	W25Q64_RESULT_INVALID_ARGUMENT,
	W25Q64_RESULT_BUS_ERROR,
	W25Q64_RESULT_TIMEOUT,
	W25Q64_RESULT_WRITE_ENABLE_FAILED,
	W25Q64_RESULT_DEVICE_ID_MISMATCH
} W25Q64_Result;

W25Q64_Result W25Q64_Init(void);
W25Q64_Result W25Q64_Reset(void);
W25Q64_Result W25Q64_ReadJedecId(uint8_t *manufacturer_id,
								uint8_t *memory_type_id,
								uint8_t *capacity_id);
W25Q64_Result W25Q64_ReadStatus1(uint8_t *status);
W25Q64_Result W25Q64_WaitReady(uint32_t timeout_ms);
W25Q64_Result W25Q64_Read(uint32_t address,
						 uint8_t *data,
						 uint32_t length);
/* PageProgram不得跨越256字节页边界。 */
W25Q64_Result W25Q64_PageProgram(uint32_t address,
								const uint8_t *data,
								uint16_t length);
/* Write自动拆页，但只执行1->0编程；调用前需自行擦除目标区域。 */
W25Q64_Result W25Q64_Write(uint32_t address,
						  const uint8_t *data,
						  uint32_t length);
/* 下列擦除函数为阻塞调用，地址必须按对应擦除粒度对齐。 */
W25Q64_Result W25Q64_EraseSector4K(uint32_t address);
W25Q64_Result W25Q64_EraseBlock32K(uint32_t address);
W25Q64_Result W25Q64_EraseBlock64K(uint32_t address);
W25Q64_Result W25Q64_EraseChip(void);

#endif
