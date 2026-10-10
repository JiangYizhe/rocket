#include "stm32f10x.h"
#include "Delay.h"

/*============================== 公开函数 ==============================*/

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 72U * xus;				/* 设置定时器重装值。 */
	SysTick->VAL = 0x00U;					/* 清空当前计数值。 */
	SysTick->CTRL = 0x00000005U;			/* 使用HCLK并启动计数。 */
	while((SysTick->CTRL & 0x00010000U) == 0U)
	{
	}
	SysTick->CTRL = 0x00000004U;			/* 关闭计数器。 */
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000U);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000U);
	}
} 
