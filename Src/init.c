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

#include "init.h"
#include "MotorTask.h"


void Init_Motor_Task(void)
{
	chassis_pid_register();
	shoot_pid_register();
	gimbal_pid_register();
	MotorINFO *canlist1[8]={&shoot_t->FRICL,&shoot_t->FRICR,0,0,&gimbal_t->GMY,&gimbal_t->GMP,&shoot_t->STIR,0};
  MotorINFO *canliat2[8]={&chassis_t->CMFL,&chassis_t->CMFR,&chassis_t->CMBL,&chassis_t->CMBR,0,0,0,0};
	
	for(int i=0; i<8; i++)
	{
		can1[i]=canlist1[i];
		can2[i]=canliat2[i];
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
