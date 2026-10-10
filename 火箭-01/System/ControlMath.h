#ifndef __CONTROL_MATH_H
#define __CONTROL_MATH_H

#include <stdint.h>

float ControlMath_Clamp(float value, float min_value, float max_value);
float ControlMath_AngleErrorDeg(float target_angle_deg,
								float measured_angle_deg);
float ControlMath_AngleToRate(float target_angle_deg,
							  float measured_angle_deg,
							  float kp,
							  float rate_limit_dps);
uint16_t ControlMath_OffsetToPulseUs(float offset_us,
									float direction_sign,
									uint16_t center_us,
									uint16_t min_us,
									uint16_t max_us,
									float offset_limit_us);

#endif
