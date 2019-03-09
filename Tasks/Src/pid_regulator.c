/**
  ******************************************************************************
  * File Name       : pid_regulator.c
  * Description     : PID函数
  * Author			：林炳辉
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * C语言PID函数实现
  ******************************************************************************
  */
#include "includes.h"

void fw_PID_Reset(fw_PID_Regulator_t *pid){
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

void fw_PID_Calc(fw_PID_Regulator_t *pid){
	pid->errorCurr = pid->target - pid->feedback;
	pid->errorSum += pid->errorCurr - pid->err[pid->SumCount];
	pid->err[pid->SumCount] = pid->errorCurr;
	pid->SumCount = (pid->SumCount + 1) % PID_I_CNT;
	
	pid->componentKp = pid->kp * pid->errorCurr;
	MINMAX(pid->componentKp, -pid->componentKpMax, pid->componentKpMax);
	pid->componentKi = pid->ki * pid->errorSum;
	MINMAX(pid->componentKi, -pid->componentKiMax, pid->componentKiMax);
	pid->componentKd = pid->kd * (pid->errorCurr - pid->errorLast);
	MINMAX(pid->componentKd, -pid->componentKdMax, pid->componentKdMax);
	
	pid->errorLast = pid->errorCurr;
	
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	MINMAX(pid->output, -pid->outputMax, pid->outputMax);
}


int16_t PID_PROCESS_Double(fw_PID_Regulator_t* pid_position,fw_PID_Regulator_t* pid_speed,float target, float position_feedback, float velocity_feedback)
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


/**
	* @brief  Position PID parameter initialization and calculate
	* @author Created by DJI, Migrated by Xinyu Jian
*/

void abs_limit(float *a, float ABS_MAX)
{
  if (*a > ABS_MAX)
    *a = ABS_MAX;
  if (*a < -ABS_MAX)
    *a = -ABS_MAX;
}

static void pid_param_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float    kp,
    float    ki,
    float    kd)
{

  pid->integral_limit = intergral_limit;
  pid->max_out        = maxout;
  pid->pid_mode       = mode;

  pid->p = kp;
  pid->i = ki;
  pid->d = kd;

}
/**
  * @brief     modify pid parameter when code running
  * @param[in] pid: control pid struct
  * @param[in] p/i/d: pid parameter
  * @retval    none
  */
static void pid_reset(pid_t *pid, float kp, float ki, float kd)
{
  pid->p = kp;
  pid->i = ki;
  pid->d = kd;
  
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out  = 0;
  
}

/**
  * @brief     calculate delta PID and position PID
  * @param[in] pid: control pid struct
  * @param[in] get: measure feedback value
  * @param[in] set: target value
  * @retval    pid calculate output 
  */
float pid_calc(pid_t *pid, float get, float set)
{
  pid->get = get;
  pid->set = set;
  pid->err[NOW] = set - get;

  if ((pid->input_max_err != 0) && (fabs(pid->err[NOW]) > pid->input_max_err))
      return 0;

  if (pid->pid_mode == POSITION_PID) //position PID
  {
      pid->pout = pid->p * pid->err[NOW];
      pid->iout += pid->i * pid->err[NOW];
      pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST]);
    
      abs_limit(&(pid->iout), pid->integral_limit);
      pid->out = pid->pout + pid->iout + pid->dout;
      abs_limit(&(pid->out), pid->max_out);
  }
  else if (pid->pid_mode == DELTA_PID) //delta PID
  {
      pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
      pid->iout = pid->i * pid->err[NOW];
      pid->dout = pid->d * (pid->err[NOW] - 2 * pid->err[LAST] + pid->err[LLAST]);

      pid->out += pid->pout + pid->iout + pid->dout;
      abs_limit(&(pid->out), pid->max_out);
  }

  pid->err[LLAST] = pid->err[LAST];
  pid->err[LAST]  = pid->err[NOW];
  
  
  if ((pid->output_deadband != 0) && (fabs(pid->out) < pid->output_deadband))
    return 0;
  else
    return pid->out;

}
/**
  * @brief     initialize pid parameter
  * @retval    none
  */
void PID_struct_init(
    pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd)
{
  pid->f_param_init = pid_param_init;
  pid->f_pid_reset  = pid_reset;

  pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
  pid->f_pid_reset(pid, kp, ki, kd);
}
