/**
  ******************************************************************************
  * File Name          : CANMotor.h
  * Description        : CAN线电机控制
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __CANMOTOR_H
#define __CANMOTOR_H

#include "includes.h"

#ifdef INFANTRY3
#define GM_PITCH_GRAVITY_COMPENSATION 600//1400
#define GM_PITCH_ZERO 	4976
#define GM_YAW_ZERO 	3536
#endif

#ifdef GM_TEST
#define GM_PITCH_GRAVITY_COMPENSATION 0
#define GM_PITCH_ZERO 	6910
#define GM_YAW_ZERO 	115
#endif

/************************** gimbal parameter *****************************/
/* the ratio of motor encoder value translate to degree */
#ifndef ENCODER_ANGLE_RATIO
  #define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)
#endif 

#define ANGLE_LIMIT_360(val, angle) \
  do                                \
  {                                 \
    (val) = (angle) - (int)(angle); \
    (val) += (int)(angle) % 360;    \
  } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
  do                                \
  {                                 \
    if((val)>180)                   \
      (val) -= 360;                 \
  } while (0)


#define CHASSIS_SPEED_ATTENUATION   (1.30f)
#define NORMALIZE_ANGLE180(angle) angle = ((angle) > 180) ? ((angle) - 360) : (((angle) < -180) ? (angle) + 360 : angle)
#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,0,{0,0},\
	2.0f,0.0f,0.5f,/*p i d*/\
	0,0,0,\
	20,20,20,\
	0,50,0,0,0,\
	&PID_Calc,&PID_Reset,\
} 

typedef enum
{
	ESC_C6x0=0,
	ESC_6623=1
}ESCtype_e;

typedef struct gimbal_p_y
{
  /* unit: degree */
  float yaw;
  float pitch;  
}gimbal_p_y;

typedef struct gimbal_rate
{
  /* unit: degree/s */
  float yaw_rate;
  float pitch_rate;  
}gimbal_rate;

typedef struct gimbal_sensor
{
  gimbal_p_y gyro_angle;
  gimbal_rate rate;
}gimbal_sensor;

typedef struct MotorINFO
{
	ESCtype_e			ESCtype;
	CAN_HandleTypeDef* 	CAN_TYPE;
	uint16_t 			TXID;
	uint16_t			RXID;
	float 				ReductionRate;
	ESCC6x0RxMsg_t		RxMsgC6x0;
	ESC6623RxMsg_t		RxMsg6623;
	double 				TargetAngle;
	uint8_t				s_count;
	uint8_t 			FirstEnter;
	uint8_t				gyroFirstEnter;
	double 				lastRead;
	double 				RealAngle;
	void (*Handle)(struct MotorINFO* id);
	pid_t 	positionPID;
	pid_t 	speedPID;
	int16_t				Intensity;
	float					EncoderAngle;
	float					EncoderLastAngle;
	gimbal_sensor sensor; 
}MotorINFO;

typedef struct gimbal
{
	MotorINFO GMY;
	MotorINFO GMP;
}gimbal;

typedef struct chassis
{
	MotorINFO CMFL;
	MotorINFO CMFR;
	MotorINFO CMBR;
	MotorINFO CMBL;
}chassis;

typedef struct shoot
{
	MotorINFO FRICL;
	MotorINFO FRICR;
	MotorINFO STIR;
}shoot;


extern gimbal *gimbal_t;
extern chassis *chassis_t;
extern shoot *shoot_t;
extern MotorINFO *can1[8],*can2[8];

void InitMotor(MotorINFO *id);
void Motor_ID_Setting(void);

void setCAN11(void);
void setCAN12(void);
void setCAN21(void);
void setCAN22(void);

static int16_t gimbal_get_ecd_angle(int16_t raw_ecd, int16_t center_offset);
void gimbal_set_yaw_gyro_angle(MotorINFO* id, float yaw);
void gimbal_yaw_gyro_update(MotorINFO* id , float yaw);

#endif /*__ CANMOTOR_H */
