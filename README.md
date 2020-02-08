# 麦克纳姆轮小车说明

## 小车底盘

![overview](\figure\overview.png)
 ![overview](\figure\WheelAssemble.png)

# 代码说明

## 控制部分

主要的代码放置于Control.c之中，定义了PID的结构体:
```C
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
```
定义轮子相应的结构体WHEEL，成员变量的说明:
- **uint8_t  Direction**    当前轮子的方向，通过函数 **void CarUpdate(ToyCar * car)** 进行更新
- **int16_t CurrentSpeed**  当前轮子的速度，正向为正值，反向为负值，由编码器值赋值，最大值2199
- **int16_t DesireSpeed**   设定的目标速度
- **PID VelocityPID**       该轮子的控制PID结构体
- **int32_t ControlPluse**  控制该轮子PWM占空比的数值，最大值9999
```C
typedef struct
{
	uint8_t  Direction;
	int16_t CurrentSpeed;
	int16_t DesireSpeed;
	PID VelocityPID;
	int32_t ControlPluse;
}WHEEL;
```
定义小车整体相应的结构体WHEEL，成员变量的说明:
- **uint16_t State**    小车目前的状态，可取的值为**Wheel_Reverse Wheel_Forward Wheel_Stop**
- **WHEEL xxxxxWheel**  某轮子的成员
```C
typedef struct
{
	uint16_t State;
	WHEEL FrontLeftWheel;
	WHEEL FrontRightWheel;
	WHEEL RearLeftWheel;
	WHEEL RearRightWheel;
}ToyCar;
```
控制的初始化只需创建变量**ToyCar McNhamCar** 并且调用函数**void CarInit(ToyCar * Car)**，同时需要定时调用函数 **void VelocityPidCallFnx(ToyCar * Car)** 如：将其加入定时器事件中断的回到函数中，可以得到PWM占空比的控制值，再调用**void CarUpdate(ToyCar * car)** 就可控制小车的运动。

通过设定各个轮子的**DesireSpeed** 即可对小车实现控制。



