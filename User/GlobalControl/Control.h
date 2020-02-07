#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "L298N.h"
#include "stm32f7xx_hal.h"

#define CAR_STOP 0x00
#define CAR_MOVE 0x01

#define PID_LIMIT(x,max,min)	((x>=max)?max:((x<=min)?min:x))
#define pid_abs(x)			((x>=0)?x:(-x))
#define TIM_PERIOD	9999

#define TOLERENT 800

typedef struct 
{
	float Kp;
	float Ki;
	float Kd;
	
	int Err;
	float LastErr;
	float ErrSum;
	float Summax;
	float Summin;
	
	float Max;
	float Min;
	
	float Pid_out;
}PID;

typedef struct
{
	uint8_t  Direction;
	int16_t CurrentSpeed;
	int16_t DesireSpeed;
	PID VelocityPID;
	int32_t ControlPluse;
}WHEEL;

typedef struct
{
	uint16_t State;
	WHEEL FrontLeftWheel;
	WHEEL FrontRightWheel;
	WHEEL RearLeftWheel;
	WHEEL RearRightWheel;
}ToyCar;

#define PID_LIMIT(x,max,min)	((x>=max)?max:((x<=min)?min:x))
#define pid_abs(x)			((x>=0)?x:(-x))



void CarInit(ToyCar * Car);
void VelocityPidCallFnx(ToyCar * Car);
void CarUpdate(ToyCar * car);
#endif // !_CONTROL_H_
