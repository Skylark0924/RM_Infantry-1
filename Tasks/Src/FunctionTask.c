/**
  ******************************************************************************
  * File Name          : FunctionTask.c
  * Description        : 用于记录机器人独有的功能
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#define  STIR_STEP_ANGLE 45
KeyboardMode_e KeyboardMode = NO_CHANGE;
MouseMode_e MouseLMode = NO_CLICK;
MouseMode_e MouseRMode = NO_CLICK;
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//斜坡函数
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
ChassisSpeed_Ref_t ChassisSpeedRef; 
uint8_t auto_init_f;

int ChassisTwistGapAngle = 0;

int32_t auto_counter=0;		//用于准确延时的完成某事件

int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;
int16_t testIntensity = 0;
uint8_t SuperCTestMode = 0;
uint8_t ShootState = 0;
uint8_t cdflag0 = 0;
uint8_t burst = 0;
uint16_t allowBullet0 = 0;
uint8_t ChassisTwistState = 0;
int16_t FrictionLSpeedLow = -5000;
int16_t FrictionLSpeedMid = -6500;
int16_t FrictionLSpeedHigh = -8000;
int8_t aimcount=0, chassiscount=0, servocount=0;
extern uint8_t startUp;

//初始化
void FunctionTaskInit()
{
	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	
	KeyboardMode=NO_CHANGE;
}

void OptionalFunction()
{
	PowerLimitation();
}

void Limit_and_Synchronization()
{
	//demo
	//MINMAX(UD1.TargetAngle,-900,270);//limit
	//UD2.TargetAngle=-UD1.TargetAngle;//sychronization
	//demo end
}
//******************
//遥控器模式功能编写
//******************
void RemoteControlProcess(Remote *rc)
{
	static WorkState_e LastState = NORMAL_STATE;
	if(WorkState <= 0) return;
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	if(WorkState == NORMAL_STATE)
	{	
		//for debug SuperC

		if(LastState != WorkState){
			Cap_State_Switch(CAP_STATE_RECHARGE);
		}

		ChassisSpeedRef.forward_back_ref = channelrcol* RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF;
		#ifdef USE_CHASSIS_FOLLOW
			GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
			GMY.TargetAngle -= channellrow * RC_GIMBAL_SPEED_REF;
		#else
		ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
		#endif
		
		ChassisTwistState = 0;
		
		ShootState = 0;
		FRICL.TargetAngle = 0;
		FRICR.TargetAngle = 0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
		
		#ifdef USE_AUTOAIM
		aim_mode=0;
		AutoAimGMCTRL();
		#endif /*USE_AUTOAIM*/
		
		/*遥控器左侧轴掰到最下，开启舱盖*/
		if(rc->ch3 == 0x16C)
		{
			//twist_state = 1;
			int servo_id = 0, pwm = 1800, time = 0;
			char ServoMes[15];
			sprintf(ServoMes, "#%03dP%04dT%04d!", servo_id, pwm, time);
			HAL_UART_Transmit(&SERVO_UART,(uint8_t *)&ServoMes, 15, 0xFFFF);
		}
		/*否则关闭舱盖*/
		else 
		{
//			twist_state = 0;
			int servo_id = 0, pwm = 500, time = 0;
			char ServoMes[15];
			sprintf(ServoMes, "#%03dP%04dT%04d!", servo_id, pwm, time);
			HAL_UART_Transmit(&SERVO_UART,(uint8_t *)&ServoMes, 15, 0xFFFF);
		}
		
	}
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		//for debug SuperC
		if(SuperCTestMode==1)
		{
			if(LastState != WorkState){
				Cap_State_Switch(CAP_STATE_RELEASE);
			}
		}
		else
		{
				Cap_State_Switch(CAP_STATE_STOP);
		}
		if(Cap_Get_Cap_Voltage()>12 && SuperCTestMode==1){
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF * 2;
			ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF ;
		}
		else
		{
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
			ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF ;
		}
		#ifdef USE_CHASSIS_FOLLOW
		GMY.TargetAngle -= channellrow * RC_GIMBAL_SPEED_REF;
		GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#else
		ChassisSpeedRef.rotate_ref = channellrow * RC_ROTATE_SPEED_REF;
		#endif
		
		ChassisTwistState = 0;
		
		if(SuperCTestMode==0)
		{
			ShootState = 1;
			FRICL.TargetAngle = FrictionLSpeedLow;
			FRICR.TargetAngle = -FrictionLSpeedLow;
		}
		else
		{
			//ShootState = 0;
			//FRICL.TargetAngle = 0;
			//FRICR.TargetAngle = 0;
			//ChassisTwistState = 1;
			//ShootState = 1;
			//FRICL.TargetAngle = -8000;
			//FRICR.TargetAngle = 8000;
		}		
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
		
		#ifdef USE_AUTOAIM
		aim_mode=1;
		AutoAimGMCTRL();
		#endif /*USE_AUTOAIM*/
	}
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF;
		#ifdef USE_CHASSIS_FOLLOW
		GMY.TargetAngle -= channellrow * RC_GIMBAL_SPEED_REF;
		//GMP.TargetAngle += channellcol * RC_GIMBAL_SPEED_REF;
		if(startUp)GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		else if(!startUp)GMP.TargetAngle -= channellcol * RC_GIMBAL_SPEED_REF;
		#else
		ChassisSpeedRef.rotate_ref = channellrow * RC_ROTATE_SPEED_REF;
		#endif
		
		ChassisTwistState = 0;
		
		if(SuperCTestMode==0)
		{
			ShootState = 1;
			FRICL.TargetAngle = FrictionLSpeedLow;
			FRICR.TargetAngle = -FrictionLSpeedLow;
			//Delay(10,{STIR.TargetAngle-=60;});
			if(STIR.TargetAngle-STIR.RealAngle>-100)
			{Delay(5,{STIR.TargetAngle-=60;});}
			else
			{//STIR.TargetAngle+=60;
				// 
				STIR.TargetAngle=STIR.RealAngle+100;
				//
			Delay(5,{STIR.TargetAngle-=60;});}
		}
		else
		{
			//ShootState = 0;
			//FRICL.TargetAngle = 0;
			//FRICR.TargetAngle = 0;
				ShootState = 1;
			FRICL.TargetAngle = FrictionLSpeedLow;
			FRICR.TargetAngle = -FrictionLSpeedLow;
			//Delay(10,{STIR.TargetAngle-=60;});
			if(STIR.TargetAngle-STIR.RealAngle>-100)
			{Delay(5,{STIR.TargetAngle-=60;});}
			else
			{//STIR.TargetAngle+=60;
				// 
				STIR.TargetAngle=STIR.RealAngle+100;
				//
			Delay(5,{STIR.TargetAngle-=60;});}
			ChassisTwistState = 1;
		}
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
		
		#ifdef USE_AUTOAIM
		aim_mode=1;
		AutoAimGMCTRL();
		#endif /*USE_AUTOAIM*/
	}
	FreshSuperCState();
	if(ChassisTwistState)
	{
		LJHTwist();
	}
	else ChassisDeTwist();
	//Limit_and_Synchronization();
	LastState = WorkState;
}


uint16_t KM_FORWORD_BACK_SPEED 	= NORMAL_FORWARD_BACK_SPEED;
uint16_t KM_LEFT_RIGHT_SPEED  	= NORMAL_LEFT_RIGHT_SPEED;
void KeyboardModeFSM(Key *key);
void MouseModeFSM(Mouse *mouse);

//****************
//键鼠模式功能编写
//****************
void ShootOneBullet()
{
	#ifdef USE_HEAT_LIMIT_INFANTRY
	if(JUDGE_State == ONLINE && fakeHeat0 > (maxHeat0 - realBulletSpeed0) && !burst)cdflag0 = 1;
	else cdflag0 = 0;
	if((STIR.TargetAngle - STIR.RealAngle <= 50) && burst)
	{
		if(((!cdflag0) || JUDGE_State == OFFLINE))
		{
			if(maxHeat0>fakeHeat0)allowBullet0 = (maxHeat0-fakeHeat0)/realBulletSpeed0;
			else allowBullet0 = 0;
			if(allowBullet0 >= 6)allowBullet0 = 6;
			for(int i=0;i<allowBullet0;i++)
			{
				if(fakeHeat0 < (maxHeat0 - 1*realBulletSpeed0))
				{
					STIR.TargetAngle -= STIR_STEP_ANGLE;
					fakeHeat0 += realBulletSpeed0;
				}
				else 
				{
//					if(STIR.RealAngle - STIR.TargetAngle <= 0)STIR.TargetAngle = -STIR_STEP_ANGLE * floor(-STIR.RealAngle/STIR_STEP_ANGLE);
					if(STIR.RealAngle - STIR.TargetAngle <= 0)STIR.TargetAngle = STIR.RealAngle;
				}
			}
			allowBullet0 = 0;
		}
	}
	else if((STIR.RealAngle - STIR.TargetAngle <= 1))
	{
		if(((!cdflag0) || JUDGE_State == OFFLINE) && fakeHeat0 < (maxHeat0 - realBulletSpeed0))
		{
			if(fakeHeat0 < (maxHeat0 - 1*realBulletSpeed0))
			{
				{
					STIR.TargetAngle -= STIR_STEP_ANGLE;
					fakeHeat0 += realBulletSpeed0;
				}
			}
//			else if(STIR.RealAngle - STIR.TargetAngle <= 0)STIR.TargetAngle = -STIR_STEP_ANGLE * floor(-STIR.RealAngle/STIR_STEP_ANGLE);
			if(STIR.RealAngle - STIR.TargetAngle <= 0)STIR.TargetAngle = STIR.RealAngle;
		}
	}
	#else
	//STIR.TargetAngle -= STIR_STEP_ANGLE;
	#endif
}

void MouseKeyControlProcess(Mouse *mouse, Key *key)
{	
	if(WorkState <= 0) return;
	
	MINMAX(mouse->x, -150, 150); 
	MINMAX(mouse->y, -150, 150); 
	
	#ifdef USE_CHASSIS_FOLLOW
	//GMY.TargetAngle -= mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT;
	//GMP.TargetAngle -= mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
	if(startUp)
	{
		GMP.TargetAngle += mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
		GMY.TargetAngle -= mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT;
	}
	else if(!startUp)
	{
		GMP.TargetAngle += mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
		GMY.TargetAngle -= mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT;
	}
	#else
	ChassisSpeedRef.rotate_ref = -mouse->x * RC_ROTATE_SPEED_REF;
	#endif
	
	MouseModeFSM(mouse);
	
	switch(MouseRMode)
	{
		case SHORT_CLICK:
		{
			ShootState = 1;
			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
			FRICL.TargetAngle = FrictionLSpeedLow;
			FRICR.TargetAngle = -FrictionLSpeedLow;
		}break;
		case LONG_CLICK:
		{
			if(ShootState)
			{
				ShootState = 0;
				HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
				FRICL.TargetAngle = 0;
				FRICR.TargetAngle = 0;
			}
		}break;
		default: break;
	}
	
	switch (MouseLMode)
	{
		case SHORT_CLICK:
		{
			ShootState=1;
			if(ShootState && fabs(STIR.TargetAngle-STIR.RealAngle)<5.0) {ShootOneBullet();}
			//if(ShootState) Delay(20,{STIR.TargetAngle-=60;});
		}break;
		case LONG_CLICK:
		{
			ShootState=1;
			if(ShootState)
					ShootOneBullet();//fakeHeat0=fakeHeat0+realBulletSpeed0;
		}
		default: break;
	}
	
	if (Cap_Get_Cap_State() != CAP_STATE_STOP )
	{
	    Cap_State_Switch(CAP_STATE_STOP);
	}

	KeyboardModeFSM(key);
	
	switch (KeyboardMode)
	{
		case SHIFT_CTRL:		//State control
		{
			
			break;
		}
		case CTRL:				//slow
		{
			
		}//DO NOT NEED TO BREAK
		case SHIFT:				//quick
		{
		
		}//DO NOT NEED TO BREAK
		case NO_CHANGE:			//normal
		{
			//CM Movement Process
			if(key->v & KEY_W)  		//key: w
				ChassisSpeedRef.forward_back_ref =  KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
			else if(key->v & KEY_S) 	//key: s
				ChassisSpeedRef.forward_back_ref = -KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
			else
			{
				ChassisSpeedRef.forward_back_ref = 0;
				FBSpeedRamp.ResetCounter(&FBSpeedRamp);
			}
			if(key->v & KEY_D)  		//key: d
				ChassisSpeedRef.left_right_ref =  KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
			else if(key->v & KEY_A) 	//key: a
				ChassisSpeedRef.left_right_ref = -KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
			else
			{
				ChassisSpeedRef.left_right_ref = 0;
				LRSpeedRamp.ResetCounter(&LRSpeedRamp);
			}
			
			if(key->v & KEY_Z)  		//key: Z 12m/s
			{
				ShootState = 1;
				FRICL.TargetAngle = FrictionLSpeedLow;
				FRICR.TargetAngle = -FrictionLSpeedLow;
				HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
			}else if(key->v & KEY_X) 	//key: X 20m/s
			{
				ShootState = 1;
				FRICL.TargetAngle = FrictionLSpeedMid;
				FRICR.TargetAngle = -FrictionLSpeedMid;
				HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
			}
			else if(key->v & KEY_C) 	//key: C 28m/s
			{
				ShootState = 1;
				FRICL.TargetAngle = FrictionLSpeedHigh;
				FRICR.TargetAngle = -FrictionLSpeedHigh;
				HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
			}
			else if(key->v & KEY_F)
			{
				aimcount++;
				if (aimcount%2==0)
					aim_mode=0;
				else
					aim_mode=1;
			}
			else if (key->v & KEY_Q)
			{
				chassiscount++;
				if (chassiscount%2==0)
					ChassisTwistState=0;
				else 
					ChassisTwistState=1;
			}
					
			else if(key->v & KEY_G)
			{
					/*按住G控制舵机，开启舱盖*/
					int id = 0, pwm = 1800, time = 0;
					char ServoMes[15];
					sprintf(ServoMes, "#%03dP%04dT%04d!", id, pwm, time);
					HAL_UART_Transmit(&SERVO_UART,(uint8_t *)&ServoMes, 15, 0xFFFF);	
					/********************/
			}
			/*不按G，关闭舱盖*/
			int id = 0, pwm = 500, time = 0;
			char ServoMes[15];
			sprintf(ServoMes, "#%03dP%04dT%04d!", id, pwm, time);
			HAL_UART_Transmit(&SERVO_UART,(uint8_t *)&ServoMes, 15, 0xFFFF);
					/***************/
			
			if(ChassisTwistState)
					ChassisTwist();
			else
				ChassisDeTwist();
			if(aim_mode)
					AutoAimGMCTRL();
		}
	}
	//Limit_and_Synchronization();
}

void KeyboardModeFSM(Key *key)
{
	if((key->v & 0x30) == 0x30)//Shift_Ctrl
	{
		KM_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
		KeyboardMode=SHIFT_CTRL;
		burst = 1;
	}
	else if(key->v & KEY_SHIFT)//Shift
	{
		if (Cap_Get_Cap_State() != CAP_STATE_RELEASE )
	  {
			Cap_State_Switch(CAP_STATE_RELEASE);
		}
		if(Cap_Get_Cap_Voltage()>12)
		{
			KM_FORWORD_BACK_SPEED=  HIGH_FORWARD_BACK_SPEED;
			KM_LEFT_RIGHT_SPEED = HIGH_LEFT_RIGHT_SPEED;
		}
		else
		{
			KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
			KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
		}
		
		KeyboardMode=SHIFT;
	}
	else if(key->v & KEY_CTRL)//Ctrl
	{
		KM_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
		KeyboardMode=CTRL;
		burst = 1;
	}
	else
	{
		burst = 0;
		KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
		KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
		KeyboardMode=NO_CHANGE;
	}	
}

void MouseModeFSM(Mouse *mouse)
{
	static uint8_t counterl = 0;
	static uint8_t counterr = 0;
	switch (MouseLMode)
	{
		case SHORT_CLICK:
		{
			counterl++;
			if(mouse->press_l == 0)
			{
				MouseLMode = NO_CLICK;
				counterl = 0;
			}
			else if(counterl>=100)
			{
				MouseLMode = LONG_CLICK;
				counterl = 0;
			}
			else
			{
				MouseLMode = SHORT_CLICK;
			}
		}break;
		case LONG_CLICK:
		{
			if(mouse->press_l==0)
			{
				MouseLMode = NO_CLICK;
			}
			else
			{
				MouseLMode = LONG_CLICK;
			}
		}break;
		case NO_CLICK:
		{
			if(mouse->press_l)
			{
				MouseLMode = SHORT_CLICK;
				//if(ShootState && abs(STIR.TargetAngle-STIR.RealAngle)<5.0) {ShootOneBullet();}
			}
		}break;
	}
	
	switch (MouseRMode)
	{
		case SHORT_CLICK:
		{
			counterr++;
			if(mouse->press_r == 0)
			{
				MouseRMode = NO_CLICK;
				counterr = 0;
			}
			else if(counterr>=50)
			{
				MouseRMode = LONG_CLICK;
				counterr = 0;
			}
			else
			{
				MouseRMode = SHORT_CLICK;
			}
		}break;
		case LONG_CLICK:
		{
			if(mouse->press_r==0)
			{
				MouseRMode = NO_CLICK;
			}
			else
			{
				MouseRMode = LONG_CLICK;
			}
		}break;
		case NO_CLICK:
		{
			if(mouse->press_r)
			{
				MouseRMode = SHORT_CLICK;
			}
		}break;
	}
}

//用于遥控器模式下超级电容测试模式的控制
void FreshSuperCState(void)
{
	static uint8_t counter = 0;
	if(HAL_GPIO_ReadPin(BUTTON_GPIO_Port,BUTTON_Pin))
	{
		counter++;
		if(counter==40)
		{
			SuperCTestMode = (SuperCTestMode==1)?0:1;
		}
	}
	else
	{
		counter = 0;
	}
	if(SuperCTestMode==1)
	{
		HAL_GPIO_WritePin(GPIOF, LED_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF, LED_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin 
                          |LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin 
                          |LED1_Pin, GPIO_PIN_RESET);
	if(Control_SuperCap.C_voltage<1100)
	{
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin|LED1_Pin, GPIO_PIN_SET);
	}
	else if(Control_SuperCap.C_voltage<1300)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<1500)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin|LED4_Pin|LED3_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<1700)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin|LED4_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<1800)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin|LED5_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<1900)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin|LED6_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<2000)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin|LED7_Pin, GPIO_PIN_SET);
	else if(Control_SuperCap.C_voltage<2100)
		HAL_GPIO_WritePin(GPIOG, LED8_Pin, GPIO_PIN_SET);
}

void ChassisTwist(void)
{
	switch (ChassisTwistGapAngle)
	{
		case 0:
		{ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;}break;
		case CHASSIS_TWIST_ANGLE_LIMIT:
		{
			if(fabs((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 360 / 8192.0f - ChassisTwistGapAngle)<3)
			{ChassisTwistGapAngle = -CHASSIS_TWIST_ANGLE_LIMIT;}break;
		}
		case -CHASSIS_TWIST_ANGLE_LIMIT:
		{
			if(fabs((GMY.RxMsg6623.angle - GM_YAW_ZERO) * 360 / 8192.0f - ChassisTwistGapAngle)<3)
			{ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;}break;
		}
	}
}

void ChassisDeTwist(void)
{
	ChassisTwistGapAngle = 0;
}

void LJHTwist(void)
{
	ChassisTwist();
}


//获取云台编码器角度
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
/*****云台初始化回中*****/
void InitGimbalState(void)
{
//	float center_offset_pit, center_offset_yaw;
//	
//	
//	
//	if (auto_init_f == 0)
//  {
//		//gimbal_set_pitch_angle(pgimbal, pgimbal->ecd_angle.pitch * (1 - ramp_calculate(&pitch_ramp)));
//    {
//      if (fabs(pgimbal->ecd_angle.pitch) < 1.5f)
//      {
//        gimbal_yaw_enable(pgimbal);
//        gimbal_set_yaw_angle(pgimbal, pgimbal->ecd_angle.yaw * (1 - ramp_calculate(&yaw_ramp)), 0);
//        if (fabs(pgimbal->ecd_angle.yaw) < 1.5f)
//        {
//          auto_init_f = 1;
//        }
//      }
//    }
//  }
}