#include "stm32f10x.h"
#include "RocketConfig.h"
#include "ControlMath.h"
#include "Servo.h"

/*============================== 私有变量 ==============================*/

/*
 * IMU初始化会在全部普通AFIO重映射完成后关闭JTAG；防止后续重�? * Servo_Init再次读改写MAPR并破坏SWD配置�? */
static uint8_t s_servo_initialized = 0U;

/*============================== 私有函数 ==============================*/

static uint16_t Servo_AngleToPulseUs(uint16_t angle_deg)
{
	uint32_t pulse_range;

	if(angle_deg > 180U)
	{
		angle_deg = 180U;
	}

	pulse_range = ROCKET_PARACHUTE_SERVO_MAX_US - ROCKET_PARACHUTE_SERVO_MIN_US;
	return (uint16_t)(ROCKET_PARACHUTE_SERVO_MIN_US +
					  (pulse_range * angle_deg) / 180U);
}

static void Servo_TIM_OCInit(TIM_TypeDef *timer,
							 uint8_t channel,
							 TIM_OCInitTypeDef *oc_init)
{
	switch(channel)
	{
		case 1U:
			TIM_OC1Init(timer, oc_init);
			TIM_OC1PreloadConfig(timer, TIM_OCPreload_Enable);
			break;
		case 2U:
			TIM_OC2Init(timer, oc_init);
			TIM_OC2PreloadConfig(timer, TIM_OCPreload_Enable);
			break;
		case 3U:
			TIM_OC3Init(timer, oc_init);
			TIM_OC3PreloadConfig(timer, TIM_OCPreload_Enable);
			break;
		case 4U:
			TIM_OC4Init(timer, oc_init);
			TIM_OC4PreloadConfig(timer, TIM_OCPreload_Enable);
			break;
		default:
			break;
	}
}

static void Servo_TIM_SetCompare(TIM_TypeDef *timer, uint8_t channel, uint16_t pulse_us)
{
	switch(channel)
	{
		case 1U:
			TIM_SetCompare1(timer, pulse_us);
			break;
		case 2U:
			TIM_SetCompare2(timer, pulse_us);
			break;
		case 3U:
			TIM_SetCompare3(timer, pulse_us);
			break;
		case 4U:
			TIM_SetCompare4(timer, pulse_us);
			break;
		default:
			break;
	}
}

static void Servo_TIM_TimeBaseInit(TIM_TypeDef *timer)
{
	TIM_TimeBaseInitTypeDef tim_init;

	TIM_InternalClockConfig(timer);
	TIM_TimeBaseStructInit(&tim_init);
	tim_init.TIM_Prescaler = 72U - 1U;
	tim_init.TIM_Period = ROCKET_SERVO_PWM_PERIOD_US - 1U;
	tim_init.TIM_CounterMode = TIM_CounterMode_Up;
	tim_init.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(timer, &tim_init);
}

static void Servo_TIM2Init(void)
{
	GPIO_InitTypeDef gpio_init;
	TIM_OCInitTypeDef oc_init;

	RCC_APB2PeriphClockCmd(ROCKET_TIM2_GPIO_CLOCK | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(ROCKET_TIM2_CLOCK, ENABLE);
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);

	gpio_init.GPIO_Pin = ROCKET_PARACHUTE_PWM_GPIO_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ROCKET_PARACHUTE_PWM_GPIO, &gpio_init);

	Servo_TIM_TimeBaseInit(ROCKET_PARACHUTE_PWM_TIMER);

	TIM_OCStructInit(&oc_init);
	oc_init.TIM_OCMode = TIM_OCMode_PWM1;
	oc_init.TIM_OutputState = TIM_OutputState_Enable;
	oc_init.TIM_OCPolarity = TIM_OCPolarity_High;
	oc_init.TIM_Pulse = Servo_AngleToPulseUs(ROCKET_PARACHUTE_INIT_ANGLE_DEG);
	Servo_TIM_OCInit(ROCKET_PARACHUTE_PWM_TIMER,
					 ROCKET_PARACHUTE_PWM_CHANNEL,
					 &oc_init);

	TIM_ARRPreloadConfig(ROCKET_PARACHUTE_PWM_TIMER, ENABLE);
	TIM_Cmd(ROCKET_PARACHUTE_PWM_TIMER, ENABLE);
}

static void Servo_TIM3Init(void)
{
	GPIO_InitTypeDef gpio_init;
	TIM_OCInitTypeDef oc_init;

	RCC_APB2PeriphClockCmd(ROCKET_TIM3_GPIO_CLOCK | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(ROCKET_TIM3_CLOCK, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

	gpio_init.GPIO_Pin = ROCKET_ROLL_PWM_GPIO_PIN | ROCKET_PITCH_PWM_GPIO_PIN;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ROCKET_ROLL_PWM_GPIO, &gpio_init);

	Servo_TIM_TimeBaseInit(ROCKET_ROLL_PWM_TIMER);

	TIM_OCStructInit(&oc_init);
	oc_init.TIM_OCMode = TIM_OCMode_PWM1;
	oc_init.TIM_OutputState = TIM_OutputState_Enable;
	oc_init.TIM_OCPolarity = TIM_OCPolarity_High;
	oc_init.TIM_Pulse = ROCKET_CONTROL_SERVO_CENTER_US;
	Servo_TIM_OCInit(ROCKET_ROLL_PWM_TIMER,
					 ROCKET_ROLL_PWM_CHANNEL,
					 &oc_init);
	Servo_TIM_OCInit(ROCKET_PITCH_PWM_TIMER,
					 ROCKET_PITCH_PWM_CHANNEL,
					 &oc_init);

	TIM_ARRPreloadConfig(ROCKET_ROLL_PWM_TIMER, ENABLE);
	TIM_Cmd(ROCKET_ROLL_PWM_TIMER, ENABLE);
}

/*============================== 公开函数 ==============================*/

void Servo_Init(void)
{
	if(s_servo_initialized != 0U)
	{
		return;
	}

	Servo_TIM2Init();
	Servo_TIM3Init();
	s_servo_initialized = 1U;
	Servo_CenterControl();
}

void Servo_CenterControl(void)
{
	Servo_TIM_SetCompare(ROCKET_ROLL_PWM_TIMER,
						 ROCKET_ROLL_PWM_CHANNEL,
						 ROCKET_CONTROL_SERVO_CENTER_US);
	Servo_TIM_SetCompare(ROCKET_PITCH_PWM_TIMER,
						 ROCKET_PITCH_PWM_CHANNEL,
						 ROCKET_CONTROL_SERVO_CENTER_US);
}

void Servo_SetControlOffsetUs(float roll_offset_us, float pitch_offset_us)
{
	Servo_SetMixedControlUs(roll_offset_us, pitch_offset_us, 0.0f);
}

void Servo_SetMixedControlUs(float roll_offset_us,
							 float pitch_offset_us,
							 float altitude_deflection_deg)
{
	uint16_t roll_pulse_us;
	uint16_t pitch_pulse_us;
#if ROCKET_ALTITUDE_SERVO_ENABLE
	float altitude_offset_us;

	altitude_deflection_deg =
		ControlMath_Clamp(altitude_deflection_deg,
						  0.0f,
						  ROCKET_ALTITUDE_SERVO_LIMIT_DEG);
	altitude_offset_us =
		altitude_deflection_deg * ROCKET_ALTITUDE_FIN_US_PER_DEG;
	roll_offset_us += altitude_offset_us * ROCKET_ALTITUDE_ROLL_MIX_SIGN;
	pitch_offset_us += altitude_offset_us * ROCKET_ALTITUDE_PITCH_MIX_SIGN;
#else
	(void)altitude_deflection_deg;
#endif

	roll_pulse_us = ControlMath_OffsetToPulseUs(
		roll_offset_us,
		ROCKET_ROLL_SERVO_SIGN,
		ROCKET_CONTROL_SERVO_CENTER_US,
		ROCKET_CONTROL_SERVO_MIN_US,
		ROCKET_CONTROL_SERVO_MAX_US,
		ROCKET_CONTROL_LIMIT_US);
	pitch_pulse_us = ControlMath_OffsetToPulseUs(
		pitch_offset_us,
		ROCKET_PITCH_SERVO_SIGN,
		ROCKET_CONTROL_SERVO_CENTER_US,
		ROCKET_CONTROL_SERVO_MIN_US,
		ROCKET_CONTROL_SERVO_MAX_US,
		ROCKET_CONTROL_LIMIT_US);

	Servo_TIM_SetCompare(ROCKET_ROLL_PWM_TIMER, ROCKET_ROLL_PWM_CHANNEL, roll_pulse_us);
	Servo_TIM_SetCompare(ROCKET_PITCH_PWM_TIMER, ROCKET_PITCH_PWM_CHANNEL, pitch_pulse_us);
}

void Servo_DeployParachute(void)
{
	Servo_TIM_SetCompare(ROCKET_PARACHUTE_PWM_TIMER,
						 ROCKET_PARACHUTE_PWM_CHANNEL,
						 Servo_AngleToPulseUs(ROCKET_PARACHUTE_DEPLOY_ANGLE_DEG));
}
