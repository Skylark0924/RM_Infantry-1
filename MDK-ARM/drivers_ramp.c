/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of?
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.? See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "includes.h"

void RampInit(RampGen_t *ramp, int32_t XSCALE)
{
  ramp->count = 0;
  ramp->XSCALE = XSCALE;
}

float RampCalc(RampGen_t *ramp)
{
  if (ramp->XSCALE <= 0)
    return 0;
  
  if (ramp->count++ >= ramp->XSCALE)
    ramp->count = ramp->XSCALE;
  
  ramp->out = ramp->count / ((float)ramp->XSCALE);
  return ramp->out;
}