/**
  ******************************************************************************
  *FileName				: AutoAimTask.c
  *Description		: �������
  *Author					: ���׺�
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
*/

#include "includes.h"

#ifndef DEBUG_MODE
#ifdef	USE_AUTOAIM

#define USE_AUTOAIM_ANGLE

//*****************************************��������******************************************//

GMAngle_t aim,aim_rcd;																				//Ŀ��Ƕ�
GMAngle_t adjust;																							//У׼�������
Coordinate_t enemy_gun,enemy_scope,scope_gun;									//����
uint8_t Enemy_INFO[8],Tx_INFO[8];															//����
uint8_t find_enemy=0,aim_mode=0,upper_mode;										//aim_mode����ѡ����׼ģʽ��0Ϊ�ֶ���׼��1Ϊ�������飬2Ϊ�����3���ޣ����䣿��
uint16_t aim_cnt=0;																						//�����Ƶ��ʱ����
uint16_t auto_counter_fps = 1000;															//���֡��
int16_t current_yaw=0,current_pitch=0;												//��ǰ��̨�Ƕ�
int16_t receive_cnt=0,receive_rcd=0;													//�����λ���ź�֡��
int8_t track_cnt=60;																						//׷�ٱ���

//********************************************************************************************//


//********************************�����ʼ��********************************//

void InitAutoAim()
{
	//����AUTO_AIM_UART��DMA����
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8)!= HAL_OK)
	{
		Error_Handler();
	}

	//���������ʼ��������Ҫ�޸ģ�
	enemy_scope.x=0;	enemy_scope.y=0;	enemy_scope.z=200;
	enemy_gun.x=0;		enemy_gun.y=0;		enemy_gun.z=200;
	
	//�Ƕȱ�����ʼ��������Ҫ�޸ģ�
	aim.yaw=0;				aim.pitch=0;
	adjust.yaw=0.0f;			adjust.pitch=-0.3f;
	
	//���������ʼֵ�����ݲ�ͬ��װ���������3��������
	scope_gun.x=0;		scope_gun.y=-10;		scope_gun.z=0;
}

//**************************************************************************//


//*******************************UART�ص�����********************************//

void AutoAimUartRxCpltCallback()
{
	#ifndef USE_AUTOAIM_ANGLE
	//�������ݽ���
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e')
	{
		enemy_scope.x=(float)((RX_ENEMY_X1<<8)|RX_ENEMY_X2)*k_coordinate;
		enemy_scope.y=(float)((RX_ENEMY_Y1<<8)|RX_ENEMY_Y2)*k_coordinate;
//		enemy_scope.z=(float)((RX_ENEMY_Z1<<8)|RX_ENEMY_Z2)*k_distance;
		enemy_scope.z=350;
		enemy_scope.x=(enemy_scope.x>coordinate_max)?(enemy_scope.x-2*coordinate_max):enemy_scope.x;
		enemy_scope.y=(enemy_scope.y>coordinate_max)?(enemy_scope.y-2*coordinate_max):enemy_scope.y;
		find_enemy=1;
		receive_cnt++;
	}
	#else
	if(RX_ENEMY_START=='s'&&RX_ENEMY_END=='e')
	{
		aim.yaw=-(float)( (((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)>0x7fff) ? (((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)-0xffff) : (RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2 )*k_angle;
		aim.pitch=(float)( (((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)>0x7fff) ? (((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)-0xffff) : (RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2 )*k_angle;
		aim.yaw-=adjust.yaw;
		aim.pitch+=adjust.pitch;
		MINMAX(aim.yaw,-15.0f,15.0f);
		MINMAX(aim.pitch,-10.0f,10.0f);
		//enemy_scope.z=350;
		find_enemy=1;
		receive_cnt++;
	}
	#endif
	
	HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&Enemy_INFO,8);
}
//***************************************************************************//


//*************************************CAN�����ذ��ͨ��*************************************//

//����Ӣ��˫��̨
extern RC_Ctl_t RC_CtrlData;
void CANTxINFO()
{
	extern int16_t channelrcol;
	uint8_t stir_state= (channelrcol>0) ? (channelrcol * 7 / 661) : 0 ;
	
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x300;	//��ͷΪ0x300��ע�����Ƿ���Լ����޳�ͻ���
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	switch(WorkState)
	{
		case STOP_STATE: hcan1.pTxMsg->Data[0] = 0xff; break;
		case PREPARE_STATE: hcan1.pTxMsg->Data[0] = 0x00; break;
		case NORMAL_STATE: hcan1.pTxMsg->Data[0] = 0x01; break;
		case ADDITIONAL_STATE_ONE: hcan1.pTxMsg->Data[0] = 0x02; break;
		case ADDITIONAL_STATE_TWO: hcan1.pTxMsg->Data[0] = 0x03; break;
	}
	switch(inputmode)
	{
		case REMOTE_INPUT: hcan1.pTxMsg->Data[1] = 0x01; break;
		case KEY_MOUSE_INPUT: hcan1.pTxMsg->Data[1] = 0x02; break;
		case STOP: hcan1.pTxMsg->Data[1] = 0x03; break;
	}
	hcan1.pTxMsg->Data[2] = (uint8_t)(stir_state);
	hcan1.pTxMsg->Data[3] = (uint8_t)RC_CtrlData.mouse.press_l;
	hcan1.pTxMsg->Data[4] = (uint8_t)RC_CtrlData.mouse.press_r;
	hcan1.pTxMsg->Data[5] = (uint8_t)(RC_CtrlData.key.v & 0xff);
	hcan1.pTxMsg->Data[6] = (uint8_t)((RC_CtrlData.key.v>>8) & 0xff);
	hcan1.pTxMsg->Data[7] = 0;

	if(can1_update == 1 && can1_type == 3)
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
		#else
		#ifdef CAN12
			can1_type = 2;
		#endif
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

//*******************************************************************************************//


//****************************************����Ƕ�ת������*************************************//

//��ʱ���ж��з�Ƶ����øú���
void EnemyINFOProcess()
{
	#ifndef USE_AUTOAIM_ANGLE
	//����ת��
	enemy_gun.x=enemy_scope.x+scope_gun.x;
	enemy_gun.y=enemy_scope.y+scope_gun.y;
	enemy_gun.z=enemy_scope.z+scope_gun.z;
	
	//�Ƕȼ��㣨���������ڴ�϶࣬���ܷ���2ms���µ�ʱ���ж���ִ�У�
	aim.yaw=atan(enemy_gun.x/(enemy_gun.z*cos(GMP_ANGLE)-enemy_gun.y*sin(GMP_ANGLE)))/const_pi*180.0-adjust.yaw;
	aim.pitch=atan(enemy_gun.y/enemy_gun.z)/const_pi*180.0+adjust.pitch;
	#endif
	
	//׷��
	if(((aim.yaw>0 && aim_rcd.yaw>0) || (aim.yaw<0 && aim_rcd.yaw<0)))
	{
		track_cnt--;
		MINMAX(track_cnt,0,60);
	}
	else
	{
		track_cnt=60;
	}
}

//*********************************************************************************************//


//**************************��ͨģʽ������ƺ���****************************//

void AutoAimNormal()
{
	MINMAX(aim.yaw,-15.0f,15.0f);
	MINMAX(aim.pitch,-10.0f,10.0f);
	if(find_enemy)
	{
//		if(aim_cnt<1)
//		{
	//		GMY.TargetAngle+=(aim.yaw)/8;
			gimbal_t->GMY.TargetAngle+=(aim.yaw)/5*(0.5f+0.012f*track_cnt) - 0.02f;
			gimbal_t->GMP.TargetAngle+=(aim.pitch+aim_rcd.pitch)/6 - 0.17f;
//			aim_cnt++;
//		}
//		else
//		{
			find_enemy=0;
//			aim_cnt=0;
//			aim_rcd.yaw=aim.yaw;
//			aim_rcd.pitch=aim.pitch;
		}
	}


//**************************************************************************//


//**************************���ģʽ������ƺ���****************************//

void AutoAimBuff()
{
	if(find_enemy)
	{
		//��̨ת��Ŀ�귽��
		if(aim_cnt==0)
		{
			aim_rcd.yaw=aim.pitch;
			gimbal_t->GMY.TargetAngle+=aim_rcd.yaw/20;
			gimbal_t->GMP.TargetAngle+=aim_rcd.pitch/20;
			aim_cnt++;
		}
		else if(aim_cnt>=1 && aim_cnt<20)
		{
			gimbal_t->GMY.TargetAngle+=aim_rcd.yaw/20;
			gimbal_t->GMP.TargetAngle+=aim_rcd.pitch/20;
			aim_cnt++;
		}
		//�ȴ���̨�ȶ�
		else if(aim_cnt>=20 && aim_cnt<30)
		{
			aim_cnt++;
		}
		//����
		else if(aim_cnt==30)
		{
			//ShootOneBullet();
			aim_cnt++;
		}
		//�ȴ���̨�ȶ�
		else if(aim_cnt>30 && aim_cnt<50)
		{
			aim_cnt++;
		}
		//����ָ������
		else if(aim_cnt>=50 && aim_cnt<70)
		{
			gimbal_t->GMY.TargetAngle-=aim_rcd.yaw/20;
			gimbal_t->GMP.TargetAngle-=aim_rcd.pitch/20;
			aim_cnt++;
		}
		else
		{
			aim_cnt=0;
			find_enemy=0;
		}
	}
}

//**************************************************************************//


//***************************��λ������ģʽ�л�*****************************//

void UpperStateFSM()
{
	if(upper_mode != aim_mode && upper_mode != 0)
	{
		Tx_INFO[0] = 'c';
		Tx_INFO[1] = aim_mode;
		Tx_INFO[2] = 'e';
		HAL_UART_Transmit(&AUTOAIM_UART,Tx_INFO,3,0xff);
		
		upper_mode = aim_mode;
	}
}

//**************************************************************************//


//***********************************�������*******************************//

void AutoAimGMCTRL()
{
	UpperStateFSM();
	switch(aim_mode)
	{
		case 1: 														//����
		{
			AutoAimNormal();
			break;
		}
		case 2:															//���
		{
			AutoAimBuff();
			break;
		}
		default: break;
	}
	
	/************���֡��*************/
	if(receive_cnt == 0)
		auto_counter_fps = 1000;
	if(auto_counter_fps == 0)
	{
		receive_rcd = receive_cnt;
		receive_cnt = 0;
		auto_counter_fps = 1000;
	}
	/*********************************/
}

//**************************************************************************//


#endif /*USE_AUTOAIM*/
#endif /*DEBUG_MODE*/
