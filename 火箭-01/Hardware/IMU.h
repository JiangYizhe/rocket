#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

typedef struct
{
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
	float roll_rate_dps;
	float pitch_rate_dps;
	float yaw_rate_dps;
	float accel_x_mg;
	float accel_y_mg;
	float accel_z_mg;
	uint8_t valid;
} IMU_Data;

typedef enum
{
	IMU_STATUS_NOT_INITIALIZED = 0,
	IMU_STATUS_READY,
	IMU_STATUS_BUS_ERROR,
	IMU_STATUS_DEVICE_ID_ERROR,
	IMU_STATUS_RESET_TIMEOUT,
	IMU_STATUS_CONFIG_ERROR,
	IMU_STATUS_FIFO_RESET,
	IMU_STATUS_DATA_STALE
} IMU_Status;

typedef enum
{
	IMU_FAULT_NONE = 0,
	IMU_FAULT_FIFO_STATUS_READ,
	IMU_FAULT_FIFO_OVERFLOW,
	IMU_FAULT_FIFO_RESET_FAILED,
	IMU_FAULT_FIFO_ENTRY_READ,
	IMU_FAULT_INIT_BUS_ERROR,
	IMU_FAULT_DEVICE_ID,
	IMU_FAULT_RESET_TIMEOUT,
	IMU_FAULT_CONFIG
} IMU_FaultReason;

typedef struct
{
	IMU_Status status;
	IMU_FaultReason last_fault;
	uint8_t consecutive_bus_errors;
	uint16_t fifo_level;
	uint16_t init_count;
	uint16_t total_bus_errors;
	uint16_t fifo_reset_count;
} IMU_Diagnostics;

uint8_t IMU_Init(void);
uint8_t IMU_Read(IMU_Data *data, uint32_t now_ms);
uint8_t IMU_IsInitialized(void);
IMU_Status IMU_GetStatus(void);
void IMU_GetDiagnostics(IMU_Diagnostics *diagnostics);

#endif
