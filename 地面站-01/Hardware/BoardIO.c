#include "stm32f10x.h"
#include "BoardIO.h"

#define BOARD_LED_GPIO                       GPIOB
#define BOARD_LED_PINS                       \
	(GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5)

#define BOARD_BUTTON_GPIO                    GPIOB
#define BOARD_BUTTON_PIN                     GPIO_Pin_6

#define BOARD_IGNITION_GPIO                  GPIOC
#define BOARD_IGNITION_PIN                   GPIO_Pin_13

void BoardIO_Init(void)
{
	GPIO_InitTypeDef gpio_init;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |
						   RCC_APB2Periph_GPIOB |
						   RCC_APB2Periph_GPIOC,
						   ENABLE);
	/* PB3 and PB4 are LED outputs, so release only the JTAG pins and keep SWD. */
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIO_ResetBits(BOARD_LED_GPIO, BOARD_LED_PINS);
	gpio_init.GPIO_Pin = BOARD_LED_PINS;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(BOARD_LED_GPIO, &gpio_init);

	/*
	 * Three-wire button module: H1-1/PB6 receives OUT.  The module shown by
	 * the user drives OUT high while pressed and low while released.
	 */
	gpio_init.GPIO_Pin = BOARD_BUTTON_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(BOARD_BUTTON_GPIO, &gpio_init);

	/* Load a safe low level before PC13 becomes the ignition output. */
	GPIO_ResetBits(BOARD_IGNITION_GPIO, BOARD_IGNITION_PIN);
	gpio_init.GPIO_Pin = BOARD_IGNITION_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(BOARD_IGNITION_GPIO, &gpio_init);
	GPIO_ResetBits(BOARD_IGNITION_GPIO, BOARD_IGNITION_PIN);
}

void BoardIO_SetAllLeds(uint8_t active)
{
	if(active != 0U)
	{
		GPIO_SetBits(BOARD_LED_GPIO, BOARD_LED_PINS);
	}
	else
	{
		GPIO_ResetBits(BOARD_LED_GPIO, BOARD_LED_PINS);
	}
}

void BoardIO_SetStage1Ignition(uint8_t active)
{
	if(active != 0U)
	{
		GPIO_SetBits(BOARD_IGNITION_GPIO, BOARD_IGNITION_PIN);
	}
	else
	{
		GPIO_ResetBits(BOARD_IGNITION_GPIO, BOARD_IGNITION_PIN);
	}
}

uint8_t BoardIO_IsLaunchButtonPressed(void)
{
	return (GPIO_ReadInputDataBit(BOARD_BUTTON_GPIO, BOARD_BUTTON_PIN) != 0U) ?
		1U : 0U;
}
