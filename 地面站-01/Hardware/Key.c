#include "stm32f10x.h"                  // Device header
#include "Delay.h"

/**
  * 函    数：按键初始化
  * 参    数：无
  * 返 回 值：无
  */
void Key_Init(void)
{
    /*开启时钟*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);        //开启GPIOB的时钟
    
    /*GPIO初始化 - 修改为下拉输入*/
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 初始化PB0, PB1, PB10, PB11为下拉输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  // 改为下拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * 函    数：获取按键状态（非阻塞式）
  * 参    数：无
  * 返 回 值：按键状态，每个位对应一个按键
  *           Bit0: PB0状态 (1=按下, 0=松开)
  *           Bit1: PB1状态 (1=按下, 0=松开)
  *           Bit2: PB10状态 (1=按下, 0=松开)  
  *           Bit3: PB11状态 (1=按下, 0=松开)
  */
uint8_t Key_GetState(void)
{
    uint8_t keyState = 0;
    
    // 读取PB0状态（下拉输入：按下=1，松开=0）
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 1) {  // 改为检测高电平
        keyState |= 0x01;  // PB0按下
    }
    
    // 读取PB1状态
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 1) {
        keyState |= 0x02;  // PB1按下
    }
    
    // 读取PB10状态
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 1) {
        keyState |= 0x04;  // PB10按下
    }
    
    // 读取PB11状态
    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 1) {
        keyState |= 0x08;  // PB11按下
    }
    
    return keyState;
}
