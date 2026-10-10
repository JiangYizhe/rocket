#ifndef __DEBUG_CONFIG_H
#define __DEBUG_CONFIG_H

/*============================== 调试模式开关 ==============================*/

/*
 * 0：调试模式只循环输出Flash内容，控制舵机保持未初始化。
 * 1：调试模式初始化IMU和舵机，1000ms后永久运行姿态控制环。
 */
#define DEBUG_SERVO_CONTROL_ENABLE          0

/*============================== 物理跳线 ==============================*/

#define DEBUG_JUMPER_GPIO                   GPIOC
#define DEBUG_JUMPER_GPIO_CLOCK             RCC_APB2Periph_GPIOC
#define DEBUG_JUMPER_OUTPUT_PIN             GPIO_Pin_10
#define DEBUG_JUMPER_INPUT_PIN              GPIO_Pin_11

/*============================== 调试模式指示灯 ==============================*/

#define DEBUG_LED_GPIO                      GPIOC
#define DEBUG_LED_GPIO_CLOCK                RCC_APB2Periph_GPIOC
#define DEBUG_LED_PIN                       GPIO_Pin_5

/*============================== USART1输出 ==============================*/

#define DEBUG_USART                         USART1
#define DEBUG_USART_CLOCK                   RCC_APB2Periph_USART1
#define DEBUG_USART_GPIO                    GPIOA
#define DEBUG_USART_GPIO_CLOCK              RCC_APB2Periph_GPIOA
#define DEBUG_USART_TX_PIN                  GPIO_Pin_9
#define DEBUG_USART_BAUDRATE                115200U
#define DEBUG_USART_TIMEOUT_LOOPS           100000U

/*============================== Flash循环读取 ==============================*/

#define DEBUG_FLASH_DUMP_START_ADDRESS      0x00000000UL
#define DEBUG_FLASH_DUMP_SIZE_BYTES         0x0001C000UL
#define DEBUG_FLASH_BYTES_PER_LINE          16U
#define DEBUG_FLASH_RETRY_PERIOD_MS         1000U

/*============================== 正常模式飞行日志 ==============================*/

/*
 * 日志只在未短接调试跳线的正常模式中写入。
 * 每次正常模式启动都会在计时开始前擦除本次日志区域；若需保留上次
 * 数据，必须先短接PC10-PC11并进入调试模式读取。
 */
#define DEBUG_FLIGHT_LOG_ENABLE             1
#define DEBUG_FLIGHT_LOG_FREQUENCY_HZ       50U
#define DEBUG_FLIGHT_LOG_DURATION_MS        20000UL
#define DEBUG_FLIGHT_LOG_START_ADDRESS      0x00000000UL

/*============================== 调试控制环 ==============================*/

#define DEBUG_SERVO_CONTROL_START_MS        1000U
#define DEBUG_IMU_PERIOD_MS                 10U
#define DEBUG_IMU_RETRY_PERIOD_MS           500U

#endif
