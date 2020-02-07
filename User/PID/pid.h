#ifndef	PID_H_
#define PID_H_

#include "stm32f7xx_hal.h"
#include "L298N.h"

#define PID_LIMIT(x,max,min)	((x>=max)?max:((x<=min)?min:x))
#define pid_abs(x)			((x>=0)?x:(-x))

#define ENCODER_WIRE		500										//编码器线程
#define DEC_RAYIO			34										//电机减速比

#define PI					(3.14159265358979323846f)

#define rad2deg(rad)		((rad) / PI * 180)						//弧度制到角度制的转化
#define deg2rad(deg)		((deg) * PI / 180)

#define TIM_PERIOD	9999

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

extern int V[4];

extern PID Vel_PID[4];
extern int Pulse_width[4];
extern int Rpm_Vel[4];

void PID_Init(PID *PID,float kp,float ki,float kd,float max,float min,float Summax,float Summin);
void PID_Cal(PID *PID,float goal,float now_val);

void Vel_PidCal(void);
void Motor_SetPWM(uint8_t wheel, int pluse);

#endif


