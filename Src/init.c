/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "includes.h"


void Init_Motor_Task()
{
//	chassis_pid_register();
//	shoot_pid_register();
//	gimbal_pid_register();
	
	pid_struct_init(&chassis_t->CMFL.speedPID,15000, 500,12.0f,0.17f,2.0f);
	pid_struct_init(&chassis_t->CMFR.speedPID,15000, 500,12.0f,0.17f,2.0f);
	pid_struct_init(&chassis_t->CMBL.speedPID,15000, 500,12.0f,0.17f,2.0f);
	pid_struct_init(&chassis_t->CMBR.speedPID,15000, 500,12.0f,0.17f,2.0f); 
	
	pid_struct_init(&shoot_t->FRICL.speedPID, 30000, 10000,8.5f,0.0f,7.3f);
	pid_struct_init(&shoot_t->FRICR.speedPID, 30000, 10000,8.5f,0.0f,7.3f);
	pid_struct_init(&shoot_t->STIR.positionPID, 15000.0, 100, 100.0, 2.0, 0.6);
	pid_struct_init(&shoot_t->STIR.speedPID, 15000.0, 0, 1.0, 0.0, 0.0);
	
	pid_struct_init(&gimbal_t->GMY.positionPID, 2000, 10, 1.0, 0.1, 0.2);
	pid_struct_init(&gimbal_t->GMY.speedPID, 30000, 3000, 5000.0, 10.0, 20.0);
	
	pid_struct_init(&gimbal_t->GMP.positionPID, 2000, 10, 2.0, 0.1, 0.3);
	pid_struct_init(&gimbal_t->GMP.speedPID, 30000, 3000, 3000.0, 10.0, 0);	 
	
	MotorINFO *canlist1[8]={&shoot_t->FRICL,&shoot_t->FRICR,0,0,&gimbal_t->GMY,&gimbal_t->GMP,&shoot_t->STIR,0};
  MotorINFO *canlist2[8]={&chassis_t->CMFL,&chassis_t->CMFR,&chassis_t->CMBL,&chassis_t->CMBR,0,0,0,0};
	
	for(int i=0; i<8; i++)
	{
		can1[i]=canlist1[i];
		can2[i]=canlist2[i];
	}
	
	// gimbal task init
	ControlGMY(&gimbal_t->GMY);
	ControlGMP(&gimbal_t->GMP);
	
	//chassis task init
	ControlCM(&chassis_t->CMBL);
	ControlCM(&chassis_t->CMBR);
	ControlCM(&chassis_t->CMFL);
	ControlCM(&chassis_t->CMFR);
	
	//shoot task init
	ControlCM(&shoot_t->FRICL);
	ControlCM(&shoot_t->FRICR);
	ControlSTIR(&shoot_t->STIR);	
}
