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

//void ControlNM(MotorINFO *id);
//void ControlSTIR(MotorINFO *id);
//void ControlCM(MotorINFO *id);
//void ControlGMY(MotorINFO *id);
//void ControlGMP(MotorINFO *id);
//static int16_t gimbal_get_ecd_angle(int16_t raw_ecd, int16_t center_offset);
MotorINFO *can1[8], *can2[8];
gimbal *gimbal_t;
chassis *chassis_t;
shoot *shoot_t;



uint8_t GMYReseted = 0;
uint8_t GMPReseted = 0;

//**********************************************************************
//				pid_struct_init(*pid,maxout,inte_limit,kp,ki,kd)
//**********************************************************************
void chassis_pid_register()
{
	pid_struct_init(&chassis_t->CMFL.speedPID,15000, 500,12.0f,0.17f,2.0f);
	pid_struct_init(&chassis_t->CMFR.speedPID,15000, 500,12.0f,0.17f,2.0f);
	pid_struct_init(&chassis_t->CMBL.speedPID,15000, 500,12.0f,0.17f,2.0f);
	pid_struct_init(&chassis_t->CMBR.speedPID,15000, 500,12.0f,0.17f,2.0f);
}

void shoot_pid_register()
{
	pid_struct_init(&shoot_t->FRICL.speedPID, 30000, 10000,8.5f,0.0f,7.3f);
	pid_struct_init(&shoot_t->FRICR.speedPID, 30000, 10000,8.5f,0.0f,7.3f);
	pid_struct_init(&shoot_t->STIR.positionPID, 15000.0, 100, 100.0, 2.0, 0.6);
	pid_struct_init(&shoot_t->STIR.speedPID, 15000.0, 0, 1.0, 0.0, 0.0);
}
//使用云台电机时，请务必确定校准过零点
void gimbal_pid_register()
{
	#ifdef INFANTRY3
	pid_struct_init(&gimbal_t->GMY.positionPID, 2000, 10, 0.3, 0, 0.1;
	pid_struct_init(&gimbal_t->GMY.speedPID, 30000, 3000, 3000, 20, 30);
	
	pid_struct_init(&gimbal_t->GMP.positionPID, 2000, 10, 0.5, 0, 0.3);
  pid_struct_init(&gimbal_t->GMP.speedPID, 30000, 3000, 1000, 80, 0);	
	#elif defined GM_TEST
	pid_struct_init(&gimbal_t->GMY.positionPID, 2000, 10, 1.0, 0.1, 0.2);
	pid_struct_init(&gimbal_t->GMY.speedPID, 30000, 3000, 5000.0, 10.0, 20.0);
	
	pid_struct_init(&gimbal_t->GMP.positionPID, 2000, 10, 2.0, 0.1, 0.3);
	pid_struct_init(&gimbal_t->GMP.speedPID, 30000, 3000, 3000.0, 10.0, 0);	 
	#endif
}

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
		
		id->Intensity = PID_PROCESS_Double(id->positionPID, id->speedPID, id->TargetAngle,id->RealAngle,ThisSpeed);
		
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
		
		id->Intensity = PID_PROCESS_Double(id->positionPID, id->speedPID,id->TargetAngle,id->RealAngle,ThisSpeed);
		
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
	pid_calculate(&(id->speedPID), id->RxMsgC6x0.RotateSpeed, (float)(id->TargetAngle));
	id->Intensity=(1.30f)*id->speedPID.output;
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
	
	#ifdef USE_CHASSIS_IMU
	gimbal_yaw_gyro_update(id, gyroZAngle+id->EncoderAngle);
	gimbal_set_yaw_gyro_angle(id, id->TargetAngle);
		
	float   center_offset;
	float 	yaw;
	float 	ThisAngle = id->sensor.gyro_angle.yaw+id->EncoderAngle;
	float 	Speed = imu.wz;		
	
	yaw=id->TargetAngle;
	center_offset = ThisAngle - id->EncoderAngle;
	//Initialize as encoder
	if(id->FirstEnter==1) {
		//id->lastRead = ThisAngle;
		id->lastRead = id->EncoderAngle;
		id->RealAngle = id->EncoderAngle;
		id->FirstEnter = 0;
		return;
	}

	id->RealAngle = ThisAngle;
	
	//A bug is hiding here. Because yaw angle is accumulated, after running for a while, TargetAngle may be more than 360. If now change it to encoder angle, boom! 
	#ifdef SHOOT_TEST
	id->RealAngle = id->EncoderAngle;;
	#endif
	
	//Angle Limitation, from -45 to 45 degree
//	MINMAX(id->TargetAngle, id->RealAngle - id->EncoderAngle - 45.0f, id->RealAngle - id->EncoderAngle + 45.0f);
	MINMAX(yaw, center_offset - 45.0f, center_offset + 45.0f);
	MINMAX(id->EncoderAngle,  - 45.0f,  45.0f);
	
	#else
	
	float 	ThisAngle = -imu.yaw;
	float 	Speed = imu.wz;		

			
	//Initialize as encoder
	if(id->FirstEnter==1) {
		//id->lastRead = ThisAngle;
		id->lastRead = id->EncoderAngle;
		//if(GMYReseted) id->FirstEnter = 0;
		id->RealAngle = id->EncoderAngle;
		id->FirstEnter = 0;
		return;
	}
	
	//Deal with that imu angle changes from 0 to 360 and accumulative angle 
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
	
	//A bug is hiding here. Because yaw angle is accumulated, after running for a while, TargetAngle may be more than 360. If now change it to encoder angle, boom! 
	#ifdef SHOOT_TEST
	id->RealAngle = id->EncoderAngle;;
	#endif
	
	//Angle Limitation, from -45 to 45 degree
	MINMAX(id->TargetAngle, id->RealAngle - id->EncoderAngle - 45.0f, id->RealAngle - id->EncoderAngle + 45.0f);
	
	#endif
	//For initializing slowly
	if(abs(id->RealAngle-id->TargetAngle)<2) GMYReseted = 1;
	if(GMYReseted==0) id->positionPID.param.max_out = 0.5;
	else id->positionPID.param.max_out = 10;
	
	#ifdef INFANTRY3
	id->Intensity = -PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,Speed);
	#elif defined GM_TEST
	id->Intensity = PID_PROCESS_Double(id->positionPID, id->speedPID, id->TargetAngle,id->RealAngle,Speed);
	#endif
	//id->Intensity=0;
}

//Pitch, bending up id positive, benging down is negative 
void ControlGMP(MotorINFO* id)
{
	if(id==0) return;
	#ifdef INFANTRY3
		id->EncoderAngle = (id->RxMsg6623.angle - GM_PITCH_ZERO)/8192.0*360.0;
	#elif defined GM_TEST
		id->EncoderAngle = (id->RxMsgC6x0.angle - GM_PITCH_ZERO)/8192.0*360.0;
	#endif
	NORMALIZE_ANGLE180(id->EncoderAngle);//Deal with that encoder changes from 0 to 8191 
	
	#ifdef INFANTRY3
		id->RealAngle = -imu.pit;
		float Speed = imu.wy;
	#elif defined GM_TEST
		id->RealAngle = -imu.pit;
		float Speed = imu.wy;
//		id->RealAngle = -imu.rol;
//		float Speed = -imu.wx;
	#endif
	
	#ifdef SHOOT_TEST
		id->RealAngle = id->EncoderAngle;
	#endif
	
	//Limit Angle, -8 to 30
	MINMAX(id->TargetAngle, id->RealAngle - id->EncoderAngle - 8.0f, id->RealAngle - id->EncoderAngle + 30.0f);
	
	///For initializing slowly
	if(abs(id->RealAngle-id->TargetAngle)<3) GMPReseted = 1;
	if(GMPReseted==0) id->positionPID.param.max_out = 1.6;
	else id->positionPID.param.max_out = 10.0;
	
	id->Intensity = GM_PITCH_GRAVITY_COMPENSATION + PID_PROCESS_Double(id->positionPID,id->speedPID,id->TargetAngle,id->RealAngle,Speed);

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
