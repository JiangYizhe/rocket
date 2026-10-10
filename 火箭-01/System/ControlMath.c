#include "ControlMath.h"

/*============================== 公开函数 ==============================*/

float ControlMath_Clamp(float value, float min_value, float max_value)
{
	if(value < min_value)
	{
		return min_value;
	}

	if(value > max_value)
	{
		return max_value;
	}

	return value;
}

float ControlMath_AngleErrorDeg(float target_angle_deg,
								float measured_angle_deg)
{
	float error_deg = target_angle_deg - measured_angle_deg;

	while(error_deg > 180.0f)
	{
		error_deg -= 360.0f;
	}
	while(error_deg < -180.0f)
	{
		error_deg += 360.0f;
	}
	return error_deg;
}

float ControlMath_AngleToRate(float target_angle_deg,
							  float measured_angle_deg,
							  float kp,
							  float rate_limit_dps)
{
	float rate_target_dps;

	rate_target_dps =
		kp * ControlMath_AngleErrorDeg(target_angle_deg,
									 measured_angle_deg);
	return ControlMath_Clamp(rate_target_dps, -rate_limit_dps, rate_limit_dps);
}

uint16_t ControlMath_OffsetToPulseUs(float offset_us,
									float direction_sign,
									uint16_t center_us,
									uint16_t min_us,
									uint16_t max_us,
									float offset_limit_us)
{
	float pulse_us;

	offset_us = ControlMath_Clamp(offset_us, -offset_limit_us, offset_limit_us);
	pulse_us = (float)center_us + offset_us * direction_sign;
	pulse_us = ControlMath_Clamp(pulse_us, (float)min_us, (float)max_us);

	return (uint16_t)(pulse_us + 0.5f);
}
