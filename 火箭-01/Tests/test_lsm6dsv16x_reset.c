#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lsm6dsv16x_reg.h"

static uint8_t s_registers[256];
static uint16_t s_write_count[256];
static int16_t s_fail_read_reg = -1;

static int32_t Test_Write(void *handle,
						  uint8_t reg,
						  const uint8_t *data,
						  uint16_t length)
{
	(void)handle;
	memcpy(&s_registers[reg], data, length);
	s_write_count[reg]++;
	return 0;
}

static int32_t Test_Read(void *handle,
						 uint8_t reg,
						 uint8_t *data,
						 uint16_t length)
{
	(void)handle;
	if((int16_t)reg == s_fail_read_reg)
	{
		return -1;
	}
	memcpy(data, &s_registers[reg], length);
	return 0;
}

static void Test_ResetMock(void)
{
	memset(s_registers, 0, sizeof(s_registers));
	memset(s_write_count, 0, sizeof(s_write_count));
	s_fail_read_reg = -1;
}

static int Test_ResetRoundTrip(const stmdev_ctx_t *ctx,
							   lsm6dsv16x_reset_t expected)
{
	lsm6dsv16x_reset_t actual = LSM6DSV16X_READY;

	Test_ResetMock();
	if(lsm6dsv16x_reset_set(ctx, expected) != 0)
	{
		return 0;
	}
	if(lsm6dsv16x_reset_get(ctx, &actual) != 0)
	{
		return 0;
	}

	return actual == expected;
}

static int Test_NoWriteAfterReadFailure(const stmdev_ctx_t *ctx)
{
	lsm6dsv16x_fifo_sflp_raw_t sflp_batch;

	Test_ResetMock();
	s_fail_read_reg = LSM6DSV16X_CTRL2;
	if(lsm6dsv16x_gy_data_rate_set(
		   ctx,
		   LSM6DSV16X_ODR_AT_120Hz) == 0)
	{
		return 0;
	}
	if(s_write_count[LSM6DSV16X_CTRL2] != 0U)
	{
		return 0;
	}

	Test_ResetMock();
	sflp_batch.game_rotation = 1U;
	sflp_batch.gravity = 0U;
	sflp_batch.gbias = 0U;
	s_fail_read_reg = LSM6DSV16X_EMB_FUNC_FIFO_EN_A;
	if(lsm6dsv16x_fifo_sflp_batch_set(ctx, sflp_batch) == 0)
	{
		return 0;
	}
	if(s_write_count[LSM6DSV16X_EMB_FUNC_FIFO_EN_A] != 0U)
	{
		return 0;
	}

	return 1;
}

int main(void)
{
	stmdev_ctx_t ctx;

	ctx.write_reg = Test_Write;
	ctx.read_reg = Test_Read;
	ctx.mdelay = 0;
	ctx.handle = 0;

	if((Test_ResetRoundTrip(&ctx, LSM6DSV16X_READY) == 0) ||
	   (Test_ResetRoundTrip(&ctx, LSM6DSV16X_GLOBAL_RST) == 0) ||
	   (Test_ResetRoundTrip(&ctx, LSM6DSV16X_RESTORE_CAL_PARAM) == 0) ||
	   (Test_ResetRoundTrip(&ctx, LSM6DSV16X_RESTORE_CTRL_REGS) == 0) ||
	   (Test_NoWriteAfterReadFailure(&ctx) == 0))
	{
		printf("FAIL: LSM6DSV16X vendor regression\n");
		return 1;
	}

	printf("PASS: LSM6DSV16X vendor regressions\n");
	return 0;
}
