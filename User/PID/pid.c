#include "pid.h"
#include <stdio.h>
#include <stdlib.h>

PID Vel_PID[4];
int Pulse_width[4] = {1999, 1999, 1999, 1999};
int Rpm_Vel[4] = {0,0,0,0};

extern TIM_HandleTypeDef htim3;

void PID_Init(PID *PID,float kp,float ki,float kd,float max,float min,float Summax,float Summin)
{
	PID->Kp = kp;
	PID->Ki = ki;
	PID->Kd = kd;
	PID->Err = 0;
	PID->ErrSum = 0;
	PID->LastErr = 0;
	PID->Max = max;
	PID->Min = min ;
	PID->Summax = Summax;
	PID->Summin = Summin;
}
void PID_Cal(PID *PID,float goal,float now_val)
{
	static float errsum = 0;
	float pout = 0;
	float iout = 0;
	float dout = 0;
	float pidout = 0;
	float delta_err = 0;
	
	PID->Err = (int)(goal - now_val);
	delta_err = PID->Err - PID->LastErr;
	errsum += PID->Err;
	PID->ErrSum = PID_LIMIT(errsum,PID->Summax,PID->Summin);
	PID->LastErr = PID->Err;
	
	pout = PID->Kp * PID->Err;
	iout = PID->Ki * PID->ErrSum;	
	dout = PID->Kd * delta_err;
	
	pidout = pout + iout + dout;
	PID->Pid_out = PID_LIMIT(pidout,PID->Max,PID->Min);
}

void Vel_PidCal(void)
{
	PID_Cal(&Vel_PID[0], (float)V[0], Rpm_Vel[0]);
	PID_Cal(&Vel_PID[1], (float)V[1], Rpm_Vel[1]);
	PID_Cal(&Vel_PID[2], (float)V[2], Rpm_Vel[2]);
	PID_Cal(&Vel_PID[3], (float)V[3], Rpm_Vel[3]);
	
	Pulse_width[0] += (int)Vel_PID[0].Pid_out;
	Pulse_width[1] += (int)Vel_PID[1].Pid_out;
	Pulse_width[2] += (int)Vel_PID[2].Pid_out;
	Pulse_width[3] += (int)Vel_PID[3].Pid_out;
}


void Motor_SetPWM(uint8_t wheel, int pluse)
{
	int pluse_temp = 0;
	pluse_temp = PID_LIMIT(pluse, TIM_PERIOD, -TIM_PERIOD);
	if (pluse_temp < 0)
		L298N_SetDirection(wheel, Wheel_Reverse);
	else
		L298N_SetDirection(wheel, Wheel_Forward);	
	pluse_temp = pid_abs(pluse_temp);
	
	switch (wheel)
	{
	case FrontWheel_Left:	
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pluse_temp);
		break;
	case FrontWheel_Right:
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pluse_temp);
		break;
	case RearWheel_Left:
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pluse_temp); 
		break;
	case RearWheel_Right:
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pluse_temp); 
		break;
	case AllWheel:
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pluse_temp);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pluse_temp);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pluse_temp);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pluse_temp);
		break;
	default:
		break;
	}
}
