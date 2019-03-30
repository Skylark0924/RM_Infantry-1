/**
  ******************************************************************************
  * File Name       : pid_regulator_fuzzy.c
  * Description     : Fuzzy-PID函数
  * Author			：JunjiaLiu
  ******************************************************************************
  *
  * Copyright (c) 2019 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * C语言Fuzzy_PID函数实现
  ******************************************************************************
  */
#include "includes.h"


#define IS_Kp 1
#define IS_Ki 2
#define IS_Kd 3
 
#define NL   -3
#define NM	 -2
#define NS	 -1
#define ZE	 0
#define PS	 1
#define PM	 2
#define PL	 3

#define N 3
 
 
 
static const float fuzzyRuleKp[7][7]={
	PL,	PL,	PM,	PM,	PS,	PS,	ZE,
	PL,	PL,	PM,	PM,	PS,	ZE,	ZE,
	PM,	PM,	PM,	PS,	ZE,	NS,	NM,
	PM,	PS,	PS,	ZE,	NS,	NM,	NM,
	PS,	PS,	ZE,	NS,	NS,	NM,	NM,
	ZE,	ZE,	NS,	NM,	NM,	NM,	NL,
	ZE,	NS,	NS,	NM,	NM,	NL,	NL
};
 
static const float fuzzyRuleKi[7][7]={
	NL,	NL,	NL,	NM,	NM,	ZE,	ZE,
	NL,	NL,	NM,	NM,	NS,	ZE,	ZE,
	NM,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NS,	NS,	ZE,	PS,	PS,	PM,
	NS,	NS,	ZE,	PS,	PS,	PM,	PM,
	ZE,	ZE,	PS,	PM,	PM,	PL,	PL,
	ZE,	ZE,	PS,	PM,	PL,	PL,	PL
};
 
static const float fuzzyRuleKd[7][7]={
	PS,	PS,	ZE,	ZE,	ZE,	PL,	PL,
	NS,	NS,	NS,	NS,	ZE,	NS,	PM,
	NL,	NL,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NS,	NS,	NS,	ZE,	PS,	PS,
	PS,	ZE,	ZE,	ZE,	ZE,	PL,	PL
};
 
typedef struct {
	float Kp;
	float Ki;
	float Kd;
}fuzzyPID;


fuzzyPID fuzzy(float e,float ec)
{
 
     float etemp,ectemp;
     float eLefttemp,ecLefttemp;
     float eRighttemp ,ecRighttemp;
  
     int eLeftIndex,ecLeftIndex;
     int eRightIndex,ecRightIndex;
     fuzzyPID   fuzzy_PID;
     etemp = e > 3.0f ? 0.0f : (e < - 3.0f ? 0.0f : (e >= 0.0f ? (e >= 2.0f ? 2.5f: (e >= 1.0f ? 1.5f : 0.5f)) : (e >= -1.0f ? -0.5f : (e >= -2.0f ? -1.5f : (e >= -3.0f ? -2.5f : 0.0f) ))));
 
     eLeftIndex = (int)e;
     eRightIndex = eLeftIndex;
     eLeftIndex = (int)((etemp-0.5f) + 3);        //[-3,3] -> [0,6]
     eRightIndex = (int)((etemp+0.5f) + 3);
 
     eLefttemp = etemp == 0.0f ? 0.0f:((etemp+0.5f)-e);
     eRighttemp= etemp == 0.0f ? 0.0f:( e-(etemp-0.5f));
 
     ectemp = ec > 3.0f ? 0.0f : (ec < - 3.0f ? 0.0f : (ec >= 0.0f ? (ec >= 2.0f ? 2.5f: (ec >= 1.0f ? 1.5f : 0.5f)) : (ec >= -1.0f ? -0.5f : (ec >= -2.0f ? -1.5f : (ec >= -3.0f ? -2.5f : 0.0f) ))));
 
     ecLeftIndex = (int)((ectemp-0.5f) + 3);        //[-3,3] -> [0,6]
     ecRightIndex = (int)((ectemp+0.5f) + 3);
 
     ecLefttemp =ectemp == 0.0f ? 0.0f:((ectemp+0.5f)-ec);
     ecRighttemp=ectemp == 0.0f ? 0.0f:( ec-(ectemp-0.5f));
 
/*************************************反模糊*************************************/
 
 
 
 
	fuzzy_PID.Kp = (eLefttemp * ecLefttemp *  fuzzyRuleKp[ecLeftIndex][eLeftIndex]                    
					+ eLefttemp * ecRighttemp * fuzzyRuleKp[ecRightIndex][eLeftIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKp[ecLeftIndex][eRightIndex]
					+ eRighttemp * ecRighttemp * fuzzyRuleKp[ecRightIndex][eRightIndex]);
 
	fuzzy_PID.Ki =   (eLefttemp * ecLefttemp * fuzzyRuleKi[ecLeftIndex][eLeftIndex]
					+ eLefttemp * ecRighttemp * fuzzyRuleKi[ecRightIndex][eLeftIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKi[ecLeftIndex][eRightIndex]
					+ eRighttemp * ecRighttemp * fuzzyRuleKi[ecRightIndex][eRightIndex]);
 
	fuzzy_PID.Kd = (eLefttemp * ecLefttemp *    fuzzyRuleKd[ecLeftIndex][eLeftIndex]
					+ eLefttemp * ecRighttemp * fuzzyRuleKd[ecRightIndex][eLeftIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKd[ecLeftIndex][eRightIndex]
					+ eRighttemp * ecRighttemp * fuzzyRuleKd[ecRightIndex][eRightIndex]);
	return fuzzy_PID;
 }


void fuzzy_PID_Reset(fuzzy_PID_Regulator_t *pid){
	pid->errorCurr = 0;
	pid->componentKd = 0;
	pid->componentKi = 0;
	pid->componentKp = 0;
	pid->errorLast = 0;
	pid->errorSum = 0;
	pid->feedback = 0;
	pid->output = 0;
	pid->SumCount = 0;
	pid->target = 0;
	for(int i=0;i<PID_I_CNT;i++) pid->err[i] = 0;
}

void fuzzy_PID_Calc(fuzzy_PID_Regulator_t *pid){
//	pid->errorCurr = pid->target - pid->feedback;
//	pid->errorSum += pid->errorCurr - pid->err[pid->SumCount];
//	pid->err[pid->SumCount] = pid->errorCurr;
//	pid->SumCount = (pid->SumCount + 1) % PID_I_CNT;
//	
//	pid->componentKp = pid->kp * pid->errorCurr;
//	MINMAX(pid->componentKp, -pid->componentKpMax, pid->componentKpMax);
//	pid->componentKi = pid->ki * pid->errorSum;
//	MINMAX(pid->componentKi, -pid->componentKiMax, pid->componentKiMax);
//	pid->componentKd = pid->kd * (pid->errorCurr - pid->errorLast);
//	MINMAX(pid->componentKd, -pid->componentKdMax, pid->componentKdMax);
//	
//	pid->errorLast = pid->errorCurr;
//	
//	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
//	MINMAX(pid->output, -pid->outputMax, pid->outputMax);
	fuzzyPID OUT = {0, 0, 0};
	float e=0,ec=0;
	float ke, kec;
	float ku_p, ku_i, ku_d;
	float emax=20, ecmax=14;
	
	ke=N/emax;
	kec=N/ecmax;
	
	pid->errorCurr = pid->target - pid->feedback;
	printf("%f",pid->errorCurr);
	e=ke*(pid->target - pid->feedback);
	ec=kec*(pid->errorCurr - pid->errorLast);
	pid->errorSum += pid->errorCurr - pid->err[pid->SumCount];
	pid->err[pid->SumCount] = pid->errorCurr;
	pid->SumCount = (pid->SumCount + 1) % PID_I_CNT;
	
	ku_p=pid->componentKpMax/N;
	ku_i=pid->componentKiMax/N;
	ku_d=pid->componentKdMax/N;

	OUT=fuzzy(e, ec);
	
	pid->componentKp = (pid->kp+ku_p*OUT.Kp) * pid->errorCurr;
	MINMAX(pid->componentKp, -pid->componentKpMax, pid->componentKpMax);
	pid->componentKi = (pid->ki+ku_i*OUT.Ki) * pid->errorSum;
	MINMAX(pid->componentKi, -pid->componentKiMax, pid->componentKiMax);
	pid->componentKd = (pid->kd+ku_d*OUT.Kd) * (pid->errorCurr - pid->errorLast);
	MINMAX(pid->componentKd, -pid->componentKdMax, pid->componentKdMax);
	
	pid->errorLast = pid->errorCurr;
	
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	MINMAX(pid->output, -pid->outputMax, pid->outputMax);
}


int16_t Fuzzy_PID_PROCESS_Double(fuzzy_PID_Regulator_t* pid_position, fuzzy_PID_Regulator_t* pid_speed,float target, float position_feedback, float velocity_feedback)
{
	//position		
	pid_position->target = target;
	pid_position->feedback = position_feedback;
	pid_position->Calc(pid_position);
	//speed
	pid_speed->target = pid_position->output;
	pid_speed->feedback = velocity_feedback;
	pid_speed->Calc(pid_speed);
	return pid_speed->output;
}
