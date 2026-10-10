#include "PID.h"

/*============================== 私有函数 ==============================*/

static float PID_Clamp(float value, float limit)
{
	if(value > limit)
	{
		return limit;
	}

	if(value < -limit)
	{
		return -limit;
	}

	return value;
}

/*============================== 公开函数 ==============================*/

void PID_Init(PID_Controller *pid,
			  float kp,
			  float ki,
			  float kd,
			  float integral_limit,
			  float output_limit,
			  float derivative_filter_alpha)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integral_limit = integral_limit;
	pid->output_limit = output_limit;
	pid->derivative_filter_alpha = derivative_filter_alpha;
	PID_Reset(pid);
}

void PID_Reset(PID_Controller *pid)
{
	pid->integral = 0.0f;
	pid->previous_measurement = 0.0f;
	pid->derivative = 0.0f;
	pid->output = 0.0f;
	pid->initialized = 0U;
}

float PID_Update(PID_Controller *pid, float setpoint, float measurement, float dt_s)
{
	float error;
	float derivative_raw;
	float integral_candidate;
	float output_candidate;
	uint8_t allow_integral;

	if(dt_s <= 0.0f)
	{
		return pid->output;
	}

	error = setpoint - measurement;

	if(pid->initialized == 0U)
	{
		pid->previous_measurement = measurement;
		pid->derivative = 0.0f;
		pid->initialized = 1U;
	}
	else
	{
		derivative_raw = (measurement - pid->previous_measurement) / dt_s;
		pid->derivative += pid->derivative_filter_alpha *
						   (derivative_raw - pid->derivative);
		pid->previous_measurement = measurement;
	}

	integral_candidate = pid->integral + pid->ki * error * dt_s;
	integral_candidate = PID_Clamp(integral_candidate, pid->integral_limit);
	output_candidate = pid->kp * error + integral_candidate - pid->kd * pid->derivative;

	allow_integral = 0U;
	if((output_candidate <= pid->output_limit) && (output_candidate >= -pid->output_limit))
	{
		allow_integral = 1U;
	}
	else if((output_candidate > pid->output_limit) && (error < 0.0f))
	{
		allow_integral = 1U;
	}
	else if((output_candidate < -pid->output_limit) && (error > 0.0f))
	{
		allow_integral = 1U;
	}

	if(allow_integral != 0U)
	{
		pid->integral = integral_candidate;
	}

	pid->output = pid->kp * error + pid->integral - pid->kd * pid->derivative;
	pid->output = PID_Clamp(pid->output, pid->output_limit);

	return pid->output;
}
