#include "Control.h"

extern int Pulse_width[4];

ToyCar McNhamCar;

void PIDInit(PID *PID, float kp, float ki, float kd, float max, float min, float Summax, float Summin)
{
	PID->Kp = kp;
	PID->Ki = ki;
	PID->Kd = kd;
	PID->Err = 0;
	PID->ErrSum = 0;
	PID->LastErr = 0;
	PID->Max = max;
	PID->Min = min;
	PID->Summax = Summax;
	PID->Summin = Summin;
}

void PIDCallFnx(PID *PID, float goal, float now_val)
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
	PID->ErrSum = PID_LIMIT(errsum, PID->Summax, PID->Summin);
	PID->LastErr = PID->Err;
	
	pout = PID->Kp * PID->Err;
	iout = PID->Ki * PID->ErrSum;	
	dout = PID->Kd * delta_err;
	
	pidout = pout + iout + dout;
	PID->Pid_out = PID_LIMIT(pidout, PID->Max, PID->Min);
}

void VelocityPidCallFnx(ToyCar * Car)
{
	PIDCallFnx(&(Car->FrontLeftWheel.VelocityPID), (float)Car->FrontLeftWheel.DesireSpeed, Car->FrontLeftWheel.CurrentSpeed);
	PIDCallFnx(&(Car->FrontRightWheel.VelocityPID), (float)Car->FrontRightWheel.DesireSpeed, Car->FrontRightWheel.CurrentSpeed);
	PIDCallFnx(&(Car->RearLeftWheel.VelocityPID), (float)Car->RearLeftWheel.DesireSpeed, Car->RearLeftWheel.CurrentSpeed);
	PIDCallFnx(&(Car->RearRightWheel.VelocityPID), (float)Car->RearRightWheel.DesireSpeed, Car->RearRightWheel.CurrentSpeed);
	
	Car->FrontLeftWheel.ControlPluse += (int)Car->FrontLeftWheel.VelocityPID.Pid_out;
	Car->FrontRightWheel.ControlPluse += (int)Car->FrontRightWheel.VelocityPID.Pid_out;
	Car->RearLeftWheel.ControlPluse += (int)Car->RearLeftWheel.VelocityPID.Pid_out;
	Car->RearRightWheel.ControlPluse += (int)Car->RearRightWheel.VelocityPID.Pid_out;
}

void CarInit(ToyCar * Car)
{
	L298N_GPIO_INIT();
	
	Car->State = CAR_STOP;
	
	Car->FrontLeftWheel.CurrentSpeed = 0;
	Car->FrontLeftWheel.DesireSpeed = 0;
	Car->FrontLeftWheel.Direction = Wheel_Stop;
	Car->FrontLeftWheel.ControlPluse = 0;
	PIDInit(&(Car->FrontLeftWheel.VelocityPID), 0.8, 0, 5, TIM_PERIOD, -TIM_PERIOD, 100, -100);
	
	Car->FrontRightWheel.CurrentSpeed = 0;
	Car->FrontRightWheel.DesireSpeed = 0;
	Car->FrontRightWheel.Direction = Wheel_Stop;
	Car->FrontRightWheel.ControlPluse = 0;
	PIDInit(&(Car->FrontRightWheel.VelocityPID), 0.8, 0, 5, TIM_PERIOD, -TIM_PERIOD, 100, -100);
	
	Car->RearLeftWheel.CurrentSpeed = 0;
	Car->RearLeftWheel.DesireSpeed = 0;
	Car->RearLeftWheel.Direction = Wheel_Stop;
	Car->RearLeftWheel.ControlPluse = 0;
	PIDInit(&(Car->RearLeftWheel.VelocityPID), 0.8, 0, 5, TIM_PERIOD, -TIM_PERIOD, 100, -100);
	
	Car->RearRightWheel.CurrentSpeed = 0;
	Car->RearRightWheel.DesireSpeed = 0;
	Car->RearRightWheel.Direction = Wheel_Stop;
	Car->RearRightWheel.ControlPluse = 0;
	PIDInit(&(Car->RearRightWheel.VelocityPID), 0.8, 0, 5, TIM_PERIOD, -TIM_PERIOD, 100, -100);
}

extern TIM_HandleTypeDef htim3;

void CarUpdate(ToyCar * car)
{
	int pluse_temp[4];
	
	car->FrontLeftWheel.ControlPluse = PID_LIMIT(car->FrontLeftWheel.ControlPluse, TIM_PERIOD, -TIM_PERIOD);
	car->FrontRightWheel.ControlPluse = PID_LIMIT(car->FrontRightWheel.ControlPluse, TIM_PERIOD, -TIM_PERIOD);
	car->RearLeftWheel.ControlPluse = PID_LIMIT(car->RearLeftWheel.ControlPluse, TIM_PERIOD, -TIM_PERIOD);
	car->RearRightWheel.ControlPluse = PID_LIMIT(car->RearRightWheel.ControlPluse, TIM_PERIOD, -TIM_PERIOD);
	
	pluse_temp[0] = pid_abs(car->FrontLeftWheel.ControlPluse);
	pluse_temp[1] = pid_abs(car->FrontRightWheel.ControlPluse);
	pluse_temp[2] = pid_abs(car->RearLeftWheel.ControlPluse);
	pluse_temp[3] = pid_abs(car->RearRightWheel.ControlPluse);
	
	if (car->FrontLeftWheel.ControlPluse < -TOLERENT)
	{
		car->FrontLeftWheel.Direction = Wheel_Reverse;
		L298N_Front_Left_IN1_LOW;
		L298N_Front_Left_IN2_HIGH;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pluse_temp[0]);
	}
	else if (car->FrontLeftWheel.ControlPluse > TOLERENT)
	{
		car->FrontLeftWheel.Direction = Wheel_Forward;
		L298N_Front_Left_IN1_HIGH;
		L298N_Front_Left_IN2_LOW;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pluse_temp[0]);
	}
	else
	{
		car->FrontLeftWheel.Direction = Wheel_Stop;
		L298N_Front_Left_IN1_LOW;
		L298N_Front_Left_IN2_LOW;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pluse_temp[0]);
	}
	
	if (car->FrontRightWheel.ControlPluse < -TOLERENT)
	{
		car->FrontRightWheel.Direction = Wheel_Reverse;
		L298N_Front_Right_IN3_LOW;
		L298N_Front_Right_IN4_HIGH;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pluse_temp[1]);
	}
	else if (car->FrontRightWheel.ControlPluse > TOLERENT)
	{
		car->FrontRightWheel.Direction = Wheel_Forward;
		L298N_Front_Right_IN3_HIGH;
		L298N_Front_Right_IN4_LOW;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pluse_temp[1]);
	}
	else
	{
		car->FrontRightWheel.Direction = Wheel_Stop;
		L298N_Front_Right_IN3_LOW;
		L298N_Front_Right_IN4_LOW;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pluse_temp[1]);
	}
	
	if (car->RearLeftWheel.ControlPluse < -TOLERENT)
	{
		car->RearLeftWheel.Direction = Wheel_Reverse;
		L298N_Rear_Left_IN3_LOW;
		L298N_Rear_Left_IN4_HIGH;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pluse_temp[2]);
	}
	else if (car->RearLeftWheel.ControlPluse > TOLERENT)
	{
		car->RearLeftWheel.Direction = Wheel_Forward;
		L298N_Rear_Left_IN3_HIGH;
		L298N_Rear_Left_IN4_LOW;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pluse_temp[2]);
	}
	else
	{
		car->RearLeftWheel.Direction = Wheel_Stop;
		L298N_Rear_Left_IN3_LOW;
		L298N_Rear_Left_IN4_LOW;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pluse_temp[2]);
	}
	
	if (car->RearRightWheel.ControlPluse < -TOLERENT)
	{
		car->RearRightWheel.Direction = Wheel_Reverse;
		L298N_Rear_Right_IN1_LOW;
		L298N_Rear_Right_IN2_HIGH;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pluse_temp[3]);
	}
	else if (car->RearRightWheel.ControlPluse > TOLERENT)
	{
		car->RearRightWheel.Direction = Wheel_Forward;
		L298N_Rear_Right_IN1_HIGH;
		L298N_Rear_Right_IN2_LOW;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pluse_temp[3]);
	}
	else
	{
		car->RearRightWheel.Direction = Wheel_Stop;
		L298N_Rear_Right_IN1_LOW;
		L298N_Rear_Right_IN2_LOW;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pluse_temp[3]);
	}
	
}