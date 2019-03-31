/**
  ******************************************************************************
  * File Name          : MotorTask.c
  * Description        : 电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JDragon-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

void ControlNM(MotorINFO *id);
void ControlSTIR(MotorINFO *id);
void ControlCM(MotorINFO *id);
void ControlGMY(MotorINFO *id);
void ControlGMP(MotorINFO *id);
static int16_t gimbal_get_ecd_angle(int16_t raw_ecd, int16_t center_offset);

uint8_t GMYReseted = 0;
uint8_t GMPReseted = 0;


//**********************************************************************
//					pid(kp,ki,kd,kprM,kirM,kdrM,rM)
//						kprM:kp result Max
//**********************************************************************

//**********************************************************************
//				Chassis_MOTORINFO_Init(func,spid)
//**********************************************************************
MotorINFO CMFL = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMFR = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBL = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBR = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICL = Chassis_MOTORINFO_Init(&ControlCM,FRIC_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICR = Chassis_MOTORINFO_Init(&ControlCM,FRIC_MOTOR_SPEED_PID_DEFAULT);
//************************************************************************
//		     Gimbal_MOTORINFO_Init(rdc,func,ppid,spid)
//************************************************************************
//使用云台电机时，请务必确定校准过零点
#ifdef INFANTRY3
MotorINFO GMP  = Gimbal_MOTORINFO_Init(1.0,&ControlGMP,
									   fw_PID_INIT(0.5,0,0.3, 	100.0, 100.0, 100.0, 10.0),
									   fw_PID_INIT(1000,80,0, 10000.0, 10000.0, 10000.0, 5000.0));
MotorINFO GMY  = Gimbal_MOTORINFO_Init(1.0,&ControlGMY,
									   fw_PID_INIT(0.3,0,0.1, 10.0, 10.0, 10.0, 10.0),
									   fw_PID_INIT(3000,20,30, 5000.0, 15000.0, 15000.0, 6000.0));
#elif defined GM_TEST 
MotorINFO GMP  = Normal_MOTORINFO_Init(1.0,&ControlGMP,
									   fw_PID_INIT(2.0,0.1,0.3, 3.0, 10.0, 10.0, 10.0),
//									   fw_PID_INIT(5000.0,80.0,300.0, 50000.0, 50000.0, 50000.0, 30000.0));
											fw_PID_INIT(3000.0,10.0,0, 50000.0, 50000.0, 50000.0, 30000.0));
MotorINFO GMY  = Normal_MOTORINFO_Init(1.0,&ControlGMY,
									   fw_PID_INIT(1.0,0.1,0.2,0.0, 10.0, 10.0, 10.0),
									   fw_PID_INIT(4000.0,10.0,20.0, 50000.0, 50000.0, 50000.0, 30000.0));			
//MotorINFO GMY  = Normal_MOTORINFO_Init(1.0,&ControlGMY,
//									   fw_PID_INIT(1,0.1,0.001,10.0, 10.0, 10.0, 10.0),
//									   fw_PID_INIT(5000.0,1.0,0, 50000.0, 50000.0, 50000.0, 30000.0));											 
#endif

//MotorINFO GMY  = Gimbal_MOTORINFO_Init(1.0,&ControlGMY,
//									   fw_PID_INIT(3.29,2.39,1.13, 10.0, 10.0, 10.0, 10.0),
//									   fw_PID_INIT(22.1,27.43,0.022, 5000.0, 15000.0, 15000.0, 6000.0));
//*************************************************************************
//			Normal_MOTORINFO_Init(rdc,func,ppid,spid)
//*************************************************************************
MotorINFO STIR = Normal_MOTORINFO_Init(36.0,&ControlSTIR,
								fw_PID_INIT(100.0, 2.0, 0.6, 	15000.0, 15000.0, 15000.0, 15000.0),
								fw_PID_INIT(1.0, 0.0, 0.0, 		15000.0, 15000.0, 15000.0, 15000.0));
								

MotorINFO* can1[8]={&FRICL,&FRICR,0,0,&GMY,&GMP,&STIR,0};
MotorINFO* can2[8]={&CMFL,&CMFR,&CMBL,&CMBR,0,0,0,0};

void ControlNM(MotorINFO* id)
{
	if(id==0) return;
	if(id->s_count == 1)
	{		
		uint16_t 	ThisAngle;	
		double 		ThisSpeed;	
		ThisAngle = id->RxMsgC6x0.angle;				
		if(id->FirstEnter==1) {id->lastRead = ThisAngle;id->FirstEnter = 0;return;}
		if(ThisAngle<=id->lastRead)
		{
			if((id->lastRead-ThisAngle)>4000)
				id->RealAngle = id->RealAngle + (ThisAngle+8192-id->lastRead) * 360 / 8192.0 / id->ReductionRate;
			else//姝ｅ父
				id->RealAngle = id->RealAngle - (id->lastRead - ThisAngle) * 360 / 8192.0 / id->ReductionRate;
		}
		else
		{
			if((ThisAngle-id->lastRead)>4000)
				id->RealAngle = id->RealAngle - (id->lastRead+8192-ThisAngle) *360 / 8192.0 / id->ReductionRate;
			else//姝ｅ父
				id->RealAngle = id->RealAngle + (ThisAngle - id->lastRead) * 360 / 8192.0 / id->ReductionRate;
		}
		ThisSpeed = id->RxMsgC6x0.RotateSpeed * 6;		
		
		id->Intensity = PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);
		
		id->s_count = 0;
		id->lastRead = ThisAngle;
		
	}
	else
	{
		id->s_count++;
	}		
}

void ControlSTIR(MotorINFO* id)
{
	if(id==0) return;
	if(id->s_count == 1)
	{		
		uint16_t 	ThisAngle;	
		double 		ThisSpeed;	
		ThisAngle = id->RxMsgC6x0.angle;				
		if(id->FirstEnter==1) {id->lastRead = ThisAngle;id->FirstEnter = 0;return;}
		if(ThisAngle<=id->lastRead)
		{
			if((id->lastRead-ThisAngle)>4000)
				id->RealAngle = id->RealAngle + (ThisAngle+8192-id->lastRead) * 360 / 8192.0 / id->ReductionRate;
			else
				id->RealAngle = id->RealAngle - (id->lastRead - ThisAngle) * 360 / 8192.0 / id->ReductionRate;
		}
		else
		{
			if((ThisAngle-id->lastRead)>4000)
				id->RealAngle = id->RealAngle - (id->lastRead+8192-ThisAngle) *360 / 8192.0 / id->ReductionRate;
			else
				id->RealAngle = id->RealAngle + (ThisAngle - id->lastRead) * 360 / 8192.0 / id->ReductionRate;
		}
		ThisSpeed = id->RxMsgC6x0.RotateSpeed;	
		
		id->Intensity = PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);
		
		id->s_count = 1;
		id->lastRead = ThisAngle;
	}
	else
	{
		id->s_count++;
	}		
}

void ControlCM(MotorINFO* id)
{
	//TargetAngle 
	if(id==0) return;
	id->offical_speedPID.ref = (float)(id->TargetAngle);
	id->offical_speedPID.fdb = id->RxMsgC6x0.RotateSpeed;
	id->offical_speedPID.Calc(&(id->offical_speedPID));
	id->Intensity=(1.30f)*id->offical_speedPID.output;
}

//Forward, Yaw turn-left angle&angle speed are positive, turn-right angle&angle speed are negative
void ControlGMY(MotorINFO* id)
{
	if(id==0) return;
	
	#ifdef INFANTRY3
	id->EncoderAngle = (id->RxMsg6623.angle - GM_YAW_ZERO)/8192.0*360.0;
	#elif defined GM_TEST
	id->EncoderAngle = (id->RxMsgC6x0.angle - GM_YAW_ZERO)/8192.0*360.0;
	#endif
	NORMALIZE_ANGLE180(id->EncoderAngle);
	
	gimbal_yaw_gyro_update(id, gyroZAngle+id->EncoderAngle);
//	gimbal_set_yaw_gyro_angle(id, id->TargetAngle);
		
	float   center_offset;
	float 	yaw;
	float 	ThisAngle = id->sensor.gyro_angle.yaw;
	float 	Speed = imu.wz;		
	
	yaw=id->TargetAngle;
	center_offset = ThisAngle - id->EncoderAngle;

			
	//Initialize as encoder
	if(id->FirstEnter==1) {
		//id->lastRead = ThisAngle;
		id->lastRead = id->EncoderAngle;
		//if(GMYReseted) id->FirstEnter = 0;
		id->RealAngle = id->EncoderAngle;
		id->FirstEnter = 0;
		return;
	}
	
	//迖讏詨0-360突窑驭m
	if(ThisAngle <= id->lastRead)
	{
		if((id->lastRead-ThisAngle) > 180)
			 id->RealAngle += (ThisAngle + 360 - id->lastRead);
		else
			 id->RealAngle -= (id->lastRead - ThisAngle);
	}
	else
	{
		if((ThisAngle-id->lastRead) > 180)
			 id->RealAngle -= (id->lastRead + 360 - ThisAngle);
		else
			 id->RealAngle += (ThisAngle - id->lastRead);
	}
	id->lastRead = ThisAngle;

	
//	//缘始郫时写謺覡毛欠卮位
//	if(id->FirstEnter==1) {
//		id->RealAngle = 0;
//		//if(GMYReseted) id->FirstEnter = 0;
//	  id->FirstEnter = 0;
//		return;
//	}
	
	//A bug is hiding here. Because yaw angle is accumulated, after running for a while, TargetAngle may be more than 360. If now change it to encoder angle, boom! 
	#ifdef SHOOT_TEST
	id->RealAngle = id->EncoderAngle;;
	#endif
	
	//Angle Limitation, from -45 to 45 degree
	MINMAX(id->TargetAngle, id->RealAngle - id->EncoderAngle - 45.0f, id->RealAngle - id->EncoderAngle + 45.0f);
	MINMAX(yaw, center_offset - 45.0f, center_offset + 45.0f);
	
	
	//For initializing slowly
	if(abs(id->RealAngle-id->TargetAngle)<2) GMYReseted = 1;
	if(GMYReseted==0) id->positionPID.outputMax = 0.5;
	else id->positionPID.outputMax = 10;
	
	#ifdef INFANTRY3
	id->Intensity = -PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,Speed);
	#elif defined GM_TEST
	id->Intensity = PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,Speed);
	#endif
	//id->Intensity=0;

}



//Pitch, bending up id positive, benging down is negative 
void ControlGMP(MotorINFO* id)
{
	if(id==0) return;
	#ifdef INFANTRY3
		id->EncoderAngle = (id->RxMsg6623.angle - GM_PITCH_ZERO)/ENCODER_ANGLE_RATIO;
	#elif defined GM_TEST
		id->EncoderAngle = -(id->RxMsgC6x0.angle - GM_PITCH_ZERO)/ENCODER_ANGLE_RATIO;
	#endif
	NORMALIZE_ANGLE180(id->EncoderAngle);//编码器0-8191突变处理
	
	#ifdef INFANTRY3
		id->RealAngle = -imu.pit;
		float Speed = imu.wy;
	#elif defined GM_TEST
		id->RealAngle = id->EncoderAngle;
		float Speed = -imu.wx;
	#endif
	
	#ifdef SHOOT_TEST
		id->RealAngle = id->EncoderAngle;
	#endif
	
	//限位，俯角8度，仰角30度
	MINMAX(id->TargetAngle, id->RealAngle - id->EncoderAngle - 8.0f, id->RealAngle - id->EncoderAngle + 30.0f);
	
	///For initializing slowly
	if(abs(id->RealAngle-id->TargetAngle)<3) GMPReseted = 1;
	if(GMPReseted==0) id->positionPID.outputMax = 1.6;
	else id->positionPID.outputMax = 10.0;
	
	id->Intensity = GM_PITCH_GRAVITY_COMPENSATION - PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,Speed);

	//id->Intensity=0;
}

//CAN
void setCAN11()
{
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x200;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can1[i]==0) {
			hcan1.pTxMsg->Data[i*2]   = 0;
			hcan1.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan1.pTxMsg->Data[i*2]   = (uint8_t)(can1[i]->Intensity >> 8);
			hcan1.pTxMsg->Data[i*2+1] = (uint8_t)can1[i]->Intensity;
		}
	}

	if(can1_update == 1 && can1_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		#ifdef CAN12
		can1_type = 2;
		#endif
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
	}
}
void setCAN12()
{
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x1ff;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can1[i+4]==0) {
			hcan1.pTxMsg->Data[i*2]   = 0;
			hcan1.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan1.pTxMsg->Data[i*2]   = (uint8_t)(can1[i+4]->Intensity >> 8);
			hcan1.pTxMsg->Data[i*2+1] = (uint8_t)can1[i+4]->Intensity;
		}
	}

	if(can1_update == 1 && can1_type == 2)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		#ifdef CAN11
		can1_type = 1;
		#endif 
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}
void setCAN21()
{
	CanTxMsgTypeDef pData;
	hcan2.pTxMsg = &pData;
	
	hcan2.pTxMsg->StdId = 0x200;
	hcan2.pTxMsg->ExtId = 0;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can2[i]==0) {
			hcan2.pTxMsg->Data[i*2]   = 0;
			hcan2.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan2.pTxMsg->Data[i*2]   = (uint8_t)(can2[i]->Intensity >> 8);
			hcan2.pTxMsg->Data[i*2+1] = (uint8_t)can2[i]->Intensity;
		}
	}

	if(can2_update == 1 && can2_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
		{
			Error_Handler();
		}
		can2_update = 0;
		#ifdef CAN22
		can2_type = 2;
		#endif
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}
void setCAN22()
{
	CanTxMsgTypeDef pData;
	hcan2.pTxMsg = &pData;
	
	hcan2.pTxMsg->StdId = 0x1ff;
	hcan2.pTxMsg->ExtId = 0;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can2[i+4]==0) {
			hcan2.pTxMsg->Data[i*2]   = 0;
			hcan2.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan2.pTxMsg->Data[i*2]   = (uint8_t)(can2[i+4]->Intensity >> 8);
			hcan2.pTxMsg->Data[i*2+1] = (uint8_t)can2[i+4]->Intensity;
		}
	}

	if(can2_update == 1 && can2_type == 2)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
		{
			Error_Handler();
		}
		can2_update = 0;
		can2_type = 1;
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
  }
}

void InitMotor(MotorINFO *id)
{
	if(id==0) return;
	id->FirstEnter=1;
	id->lastRead=0;
	id->RealAngle=0;
	id->TargetAngle=0;
	id->offical_speedPID.Reset(&(id->offical_speedPID));
	(id->Handle)(id);
	id->Intensity=0;
}

void Motor_ID_Setting()
{
	for(int i=0;i<4;i++)
	{
		if(can1[i]!=0) 
		{
			can1[i]->CAN_TYPE=&hcan1;
			can1[i]->RXID = 0x201+i;
			can1[i]->TXID = 0x200;
		}
		if(can2[i]!=0) 
		{
			can2[i]->CAN_TYPE=&hcan2;
			can2[i]->RXID = 0x201+i;
			can2[i]->TXID = 0x200;
		}
	}
	for(int i=4;i<8;i++)
	{
		if(can1[i]!=0) 
		{
			can1[i]->CAN_TYPE=&hcan1;
			can1[i]->RXID = 0x201+i;
			can1[i]->TXID = 0x1ff;
		}
		if(can2[i]!=0) 
		{
			can2[i]->CAN_TYPE=&hcan2;
			can2[i]->RXID = 0x201+i;
			can2[i]->TXID = 0x1ff;
		}
	}
}
static int16_t gimbal_get_ecd_angle(int16_t raw_ecd, int16_t center_offset)
{
	int16_t tmp = 0;
  if (center_offset >= 4096)
  {
    if (raw_ecd > center_offset - 4096)
      tmp = raw_ecd - center_offset;
    else
      tmp = raw_ecd + 8192 - center_offset;
  }
  else
  {
    if (raw_ecd > center_offset + 4096)
      tmp = raw_ecd - 8192 - center_offset;
    else
      tmp = raw_ecd - center_offset;
  }
  return tmp;
}


void gimbal_set_yaw_gyro_angle(MotorINFO* id, float yaw)
{
  float yaw_offset, yaw_now, yaw_target;

  ANGLE_LIMIT_360(yaw_target, yaw);
  ANGLE_LIMIT_360(yaw_now, id->sensor.gyro_angle.yaw);

  yaw_offset = yaw_target - yaw_now;
    if (yaw_offset > 180)
    {
      yaw_offset = yaw_offset - 360;
    }
    else if (yaw_offset < -180)
    {
      yaw_offset = yaw_offset + 360;
    }
   id->TargetAngle= id->sensor.gyro_angle.yaw + yaw_offset;
}
void gimbal_yaw_gyro_update(MotorINFO* id , float yaw)
{
  id->sensor.gyro_angle.yaw = yaw;
}
