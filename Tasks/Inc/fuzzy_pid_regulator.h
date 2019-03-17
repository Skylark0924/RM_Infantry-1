/**
  ******************************************************************************
  * File Name          : pid_regulator_fuzzy.h
  * Description        : PID函数
  ******************************************************************************
  *
  * Copyright (c) 2019 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 两套函数
	* fuzzy_前缀为模糊PID专用
	* 无前缀为官方PID，实现封装在RMLib.lib
  ******************************************************************************
  */
#ifndef _FUZZY_PID_REGULATOR_H_
#define _FUZZY_PID_REGULATOR_H_
#include "stm32f4xx.h"

#define FUZZY_PID_I_CNT 20
#define MINMAX(value, min, max) value = (value < min) ? min : (value > max ? max : value)
#define fuzzy_PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax) { \
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,\
	Kp, Ki, Kd, 0.0, 0.0, 0.0, \
	KpMax, KiMax, KdMax, 0.0, \
	OutputMax, \
	{0.0}, \
	&fuzzy_PID_Calc, &fuzzy_PID_Reset \
}
typedef __packed struct fuzzy_PID_Regulator_t
{
	float target;
	float feedback;
	float errorCurr;
	float errorSum;
	uint16_t SumCount;
	float errorLast;
	float kp;
	float ki;
	float kd;
	float componentKp;
	float componentKi;
	float componentKd;
	float componentKpMax;
	float componentKiMax;
	float componentKdMax;
	float output;
	float outputMax;
	float err[FUZZY_PID_I_CNT];
	
	void (*Calc)(struct fuzzy_PID_Regulator_t *pid);
	void (*Reset)(struct fuzzy_PID_Regulator_t *pid);
}fuzzy_PID_Regulator_t;

void fuzzy_PID_Reset(fuzzy_PID_Regulator_t *pid);
void fuzzy_PID_Calc(fuzzy_PID_Regulator_t *pid);


int16_t Fuzzy_PID_PROCESS_Double(fuzzy_PID_Regulator_t* pid_position, fuzzy_PID_Regulator_t* pid_speed,
                            float target, float position_feedback, float velocity_feedback);
#endif

