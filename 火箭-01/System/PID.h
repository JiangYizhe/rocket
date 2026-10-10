#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct
{
	float kp;
	float ki;
	float kd;
	float integral;
	float integral_limit;
	float output_limit;
	float previous_measurement;
	float derivative;
	float derivative_filter_alpha;
	float output;
	uint8_t initialized;
} PID_Controller;

void PID_Init(PID_Controller *pid,
			  float kp,
			  float ki,
			  float kd,
			  float integral_limit,
			  float output_limit,
			  float derivative_filter_alpha);
void PID_Reset(PID_Controller *pid);
float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt_s);

#endif
