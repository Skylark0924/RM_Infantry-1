/**
  ******************************************************************************
  * File Name      	: PowerLimitationTask.c
  * Description    	: 底盘功率限制算法实现
  * Author			:
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#include "math.h"

float SpeedAttenuation = 1.0f;
float LimitFactor = 1.0f;
uint8_t flag = 1;
pid PowerLimitationPID = {0};

//底盘功率限制
void PowerLimitation(void)
{
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = chassis_t->CMFL.Intensity;
	int32_t CMFRIntensity = chassis_t->CMFR.Intensity;
	int32_t CMBLIntensity = chassis_t->CMBL.Intensity;
	int32_t CMBRIntensity = chassis_t->CMBR.Intensity;
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
		//CM_current_max = CM_current_MAX;
	}
	
	//仿桂电策略
	else if((PowerHeatData.chassisPowerBuffer-((PowerHeatData.chassisPower-80)>0?(PowerHeatData.chassisPower-80):0)*1.0f < 20.0f))
	{
		//CM_current_max = 2730;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		float realPowerBuffer = PowerHeatData.chassisPowerBuffer;
		float realPower = PowerHeatData.chassisPower;
		pid_calculate(&PowerLimitationPID, realPower, 75);
		CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	else if (Cap_Get_Cap_State() != CAP_STATE_RELEASE && rlease_flag == 1 )
	{
		//PowerLimitationPID.Reset(&PowerLimitationPID);
		//LimitFactor = 1.0f;
		CM_current_max = 2730; //10000
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+0.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+0.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+0.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+0.0f))*CM_current_max;
		}
	}
	chassis_t->CMFL.Intensity = CMFLIntensity;
	chassis_t->CMFR.Intensity = CMFRIntensity;
	chassis_t->CMBL.Intensity = CMBLIntensity;
	chassis_t->CMBR.Intensity = CMBRIntensity;
	rlease_flag = 0;
}

//用于常态的基于自检测功率的功率限制
void CurBased_PowerLimitation(void)
{
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = chassis_t->CMFL.Intensity;
	int32_t CMFRIntensity = chassis_t->CMFR.Intensity;
	int32_t CMBLIntensity = chassis_t->CMBL.Intensity;
	int32_t CMBRIntensity = chassis_t->CMBR.Intensity;
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
		//CM_current_max = CM_current_MAX;
	}
	
	//仿桂电策略
	else if((PowerHeatData.chassisPowerBuffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-80)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-80):0)*1.0f < 20.0f))
	{
		//CM_current_max = 2730;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		float realPowerBuffer = PowerHeatData.chassisPowerBuffer;
		float realPower = PowerHeatData.chassisPower;
		pid_calculate(&PowerLimitationPID, realPower, 75);
		CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	chassis_t->CMFL.Intensity = CMFLIntensity;
	chassis_t->CMFR.Intensity = CMFRIntensity;
	chassis_t->CMBL.Intensity = CMBLIntensity;
	chassis_t->CMBR.Intensity = CMBRIntensity;
	rlease_flag = 0;
}

//用于放电模式下的基于自检测功率的功率限制
void CapBased_PowerLimitation(void)
{
	int32_t sum = 0;
	int32_t CM_current_max;
	int32_t CMFLIntensity = chassis_t->CMFL.Intensity;
	int32_t CMFRIntensity = chassis_t->CMFR.Intensity;
	int32_t CMBLIntensity = chassis_t->CMBL.Intensity;
	int32_t CMBRIntensity = chassis_t->CMBR.Intensity;
	//离线模式
	if (JUDGE_State == OFFLINE)
	{
		CM_current_max = 4000;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		if(sum > CM_current_max)
		{
			CMFLIntensity = (CMFLIntensity/(sum+1.0f))*CM_current_max;
			CMFRIntensity = (CMFRIntensity/(sum+1.0f))*CM_current_max;
			CMBLIntensity = (CMBLIntensity/(sum+1.0f))*CM_current_max;
			CMBRIntensity = (CMBRIntensity/(sum+1.0f))*CM_current_max;
		}
		//CM_current_max = CM_current_MAX;
	}
	else if(Cap_Get_Cap_Voltage() < 8 && (PowerHeatData.chassisPowerBuffer-((Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70)>0?(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()-70):0)*1.0f < 50.0f))
	{
		//CM_current_max = 2730;
		sum = __fabs(CMFLIntensity) + __fabs(CMFRIntensity) + __fabs(CMBLIntensity) + __fabs(CMBRIntensity);
		float realPowerBuffer = PowerHeatData.chassisPowerBuffer;
		float realPower = PowerHeatData.chassisPower;
		pid_calculate(&PowerLimitationPID, realPower, 75);
		CM_current_max = PowerLimitationPID.output;
		//LimitFactor += PowerLimitationPID.output/sum;
		//if(CM_current_max > 0.5) CM_current_max = 0;
		//if(CM_current_max < -sum) CM_current_max = -sum;
		if(realPowerBuffer < 0) realPowerBuffer = 0;
		//LimitFactor = 50/(((realPower-80)>0?(realPower-80):0)+1) * pow((realPowerBuffer),2);
		LimitFactor = 2500 + 192*pow((realPowerBuffer),1);
		if(LimitFactor > sum) LimitFactor = sum;
		CMFLIntensity *= LimitFactor/sum;
		CMFRIntensity *= LimitFactor/sum;
		CMBLIntensity *= LimitFactor/sum;
		CMBRIntensity *= LimitFactor/sum;
	}
	chassis_t->CMFL.Intensity = CMFLIntensity;
	chassis_t->CMFR.Intensity = CMFRIntensity;
	chassis_t->CMBL.Intensity = CMBLIntensity;
	chassis_t->CMBR.Intensity = CMBRIntensity;
	rlease_flag = 0;
}

//float realPowerBuffer;
//float delta;
//新功率控制算法，根据裁判系统反馈的底盘功率和buffer值通过PID算法计算SpeedAttenuation，实现功率闭环控制
//void SpeedLimitation(void)
//{
//	getRealSpeed();
//	if(__fabs(ChassisSpeedRef.forward_back_ref - RealChassisSpeed.forward_back_ref) > 50)
//	{
//		ChassisSpeedRef.forward_back_ref = (ChassisSpeedRef.forward_back_ref/10 + RealChassisSpeed.forward_back_ref/10*9);
//	}
//	if(__fabs(ChassisSpeedRef.left_right_ref - RealChassisSpeed.left_right_ref) > 50)
//	{
//		ChassisSpeedRef.left_right_ref = (ChassisSpeedRef.left_right_ref/10 + RealChassisSpeed.left_right_ref/10*9);
//	}
//	if(__fabs(ChassisSpeedRef.rotate_ref - RealChassisSpeed.rotate_ref) > 100)
//	{
//		ChassisSpeedRef.rotate_ref = (ChassisSpeedRef.rotate_ref + RealChassisSpeed.rotate_ref) / 2;
//	}
//	//离线模式
//	if (JUDGE_State == OFFLINE)
//	{
//		SpeedAttenuation = 1.0f;
//	}
//	
//	else if(PowerHeatData.chassisPowerBuffer-((PowerHeatData.chassisPower-80)>0?(PowerHeatData.chassisPower-80):0)*1.0f < 7.0f)
//	{
//		//SpeedAttenuation = 1.0;
//		realPowerBuffer = PowerHeatData.chassisPowerBuffer;
//		if(realPowerBuffer < 0) realPowerBuffer = 0;
//		float realPower = PowerHeatData.chassisPower;
//		PowerLimitationPID.feedback = realPower;
//		PowerLimitationPID.target = 75;
//		PowerLimitationPID.Calc(&PowerLimitationPID);
//		float tmp = SpeedAttenuation * (1 + PowerLimitationPID.output / (__fabs(ChassisSpeedRef.forward_back_ref) + __fabs(ChassisSpeedRef.left_right_ref) + __fabs(ChassisSpeedRef.rotate_ref)));
//		if(__fabs(chassis_t->CMFL.speedPID.target * tmp) > __fabs(chassis_t->CMFL.speedPID.feedback) && __fabs(CMFR.speedPID.target * tmp) > __fabs(CMFR.speedPID.feedback) && __fabs(chassis_t->CMBL.speedPID.target * tmp) > __fabs(chassis_t->CMBL.speedPID.feedback) && __fabs(chassis_t->CMBR.speedPID.target * tmp) > __fabs(chassis_t->CMBR.speedPID.feedback))
//		//SpeedAttenuation *= (1 + PowerLimitationPID.output / (__fabs(ChassisSpeedRef.forward_back_ref) + __fabs(ChassisSpeedRef.left_right_ref) + __fabs(ChassisSpeedRef.rotate_ref)));
//			{SpeedAttenuation = tmp;
//				flag = 1;
//			}
//		else	//可考虑相乘再开方
//		{ReduceSpeed(0);
//			flag = 0;
//			ChassisSpeedRef.forward_back_ref = RealChassisSpeed.forward_back_ref;
//			ChassisSpeedRef.left_right_ref = RealChassisSpeed.left_right_ref;
//			ChassisSpeedRef.rotate_ref = RealChassisSpeed.rotate_ref;
//		}//SpeedAttenuation *=  ReduceSpeed(PowerLimitationPID.output);
//		//float Attenuation = ReduceSpeed(PowerLimitationPID.output);
//		//ChassisSpeedRef.forward_back_ref *= Attenuation;
//		//ChassisSpeedRef.left_right_ref *= Attenuation;
//		//ChassisSpeedRef.rotate_ref *= Attenuation;
//		//delta = PowerLimitationPID.output / 20000.0f * (1.0f - 1.0f * realPowerBuffer / 60.0f);
//		//if(delta > 0) delta = 0;
//		//if(delta < -1.0) delta = -1.0;
//		//if(PowerHeatData.chassisPower > 80) SpeedAttenuation = 1.0f + delta;
//	}
//	else if(PowerHeatData.chassisPower < 80 && PowerHeatData.chassisPowerBuffer == 60)
//	{
//		//if()
//		PowerLimitationPID.Reset(&PowerLimitationPID);
//		SpeedAttenuation = 1.0;
//	}
//	return;
//}

//void getRealSpeed(void)
//{
//	float RealSpeed_CM1 = chassis_t->CMFL.offical_speedPID.fdb;
//	float RealSpeed_CM2 = CMFR.offical_speedPID.fdb;
//	float RealSpeed_CM3 = chassis_t->CMBL.offical_speedPID.fdb;
//	float RealSpeed_CM4 = chassis_t->CMBR.offical_speedPID.fdb;
//	RealChassisSpeed.forward_back_ref = (RealSpeed_CM1 - RealSpeed_CM2 + RealSpeed_CM3 - RealSpeed_CM4) / 4.0f / 12.0f;
//	RealChassisSpeed.left_right_ref = (RealSpeed_CM1 - RealSpeed_CM3 + RealSpeed_CM2 - RealSpeed_CM4) / 4.0f / 12.0f;
//	RealChassisSpeed.rotate_ref = (RealSpeed_CM1 + RealSpeed_CM3 + RealSpeed_CM2 + RealSpeed_CM4) / 4.0f / 12.0f;
//	float delta_forward_back = RealChassisSpeed.forward_back_ref - ChassisSpeedRef.forward_back_ref;
//	float delta_left_right = RealChassisSpeed.left_right_ref - ChassisSpeedRef.left_right_ref;
//	float delta_rotate = RealChassisSpeed.rotate_ref - ChassisSpeedRef.rotate_ref;
//	float tmp = __fabs(delta_forward_back);
//	uint8_t flag = 0;
//	if(tmp < __fabs(delta_left_right))
//	{
//		tmp = __fabs(delta_left_right);
//		flag = 1;
//	}
//	if(tmp < __fabs(delta_rotate))
//	{
//		tmp = __fabs(delta_rotate);
//		flag = 2;
//	}
//	switch (flag)
//	{
//		case 0:
//			if(ChassisSpeedRef.forward_back_ref > 0) 
//				return (ChassisSpeedRef.forward_back_ref + delta) / ChassisSpeedRef.forward_back_ref;
//			else
//				return (ChassisSpeedRef.forward_back_ref - delta) / ChassisSpeedRef.forward_back_ref;
//		case 1:
//			if(ChassisSpeedRef.left_right_ref > 0) 
//				return (ChassisSpeedRef.left_right_ref + delta) / ChassisSpeedRef.left_right_ref;
//			else
//				return (ChassisSpeedRef.left_right_ref - delta) / ChassisSpeedRef.left_right_ref;
//		case 2:
//			if(ChassisSpeedRef.rotate_ref > 0) 
//				return (ChassisSpeedRef.rotate_ref + delta) / ChassisSpeedRef.rotate_ref;
//			else
//				return (ChassisSpeedRef.rotate_ref - delta) / ChassisSpeedRef.rotate_ref;
//		default:
//			return 0.0f;
//	}
//}


