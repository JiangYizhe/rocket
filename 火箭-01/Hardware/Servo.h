#ifndef __SERVO_H
#define __SERVO_H

void Servo_Init(void);
void Servo_CenterControl(void);
void Servo_SetControlOffsetUs(float roll_offset_us, float pitch_offset_us);
void Servo_SetMixedControlUs(float roll_offset_us,
							 float pitch_offset_us,
							 float altitude_deflection_deg);
void Servo_DeployParachute(void);

#endif
