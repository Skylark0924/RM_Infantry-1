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

#ifndef __PID_H__
#define __PID_H__

#ifdef PID_H_GLOBAL
#define PID_H_EXTERN
#else
#define PID_H_EXTERN extern
#endif

#include "includes.h"


typedef struct pid_param
{
  float p;
  float i;
  float d;
  float input_max_err;

  float max_out;
  float inte_limit;
}pid_param;

typedef struct pid
{
  pid_param param;

  float target;
  float feedback;

  float err;
  float last_err;

  float pout;
  float iout;
  float dout;
  float output;
	
  void (*f_param_init)(struct pid *pid,
                       float max_output,
                       float inte_limit,
                       float p,
                       float i,
                       float d);
  void (*f_pid_reset)(struct pid *pid, float p, float i, float d);
}pid_t;

void pid_struct_init(
    struct pid *pid,
    float maxout,
    float intergral_limit,

    float kp,
    float ki,
    float kd);

float pid_calculate(struct pid *pid, float fdb, float ref);
int16_t PID_PROCESS_Double(struct pid pid_position, struct pid pid_speed, 
							float target, float position_feedback, float speed_feedback);

#endif // __PID_H__
