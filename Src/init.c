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



void Init_Motor_Task(gimbal *gimbal, chassis *chassis, shoot *shoot)
{
	// gimbal task init
	ControlGMY(&gimbal->GMY);
	ControlGMP(&gimbal->GMP);
	
	//chassis task init
	ControlCM(&chassis->CMBL);
	ControlCM(&chassis->CMBR);
	ControlCM(&chassis->CMFL);
	ControlCM(&chassis->CMFR);
	
	//shoot task init
	ControlCM(&shoot->FRICL);
	ControlCM(&shoot->FRICR);
	ControlSTIR(&shoot->STIR);	
}
