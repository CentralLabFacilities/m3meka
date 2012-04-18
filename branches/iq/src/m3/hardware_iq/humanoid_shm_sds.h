/* 
M3 -- Meka Robotics Robot Components
Copyright (c) 2010 Meka Robotics
Author: edsinger@mekabot.com (Aaron Edsinger)

M3 is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

M3 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with M3.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef M3BOT_SHM_SDS_H
#define M3BOT_SHM_SDS_H

#include <m3rt/base/m3rt_def.h>
#include <m3rt/base/m3ec_def.h>
#include "m3/chains/joint_array_mode.pb.h"

#define MAX_NDOF 12  // per limb


typedef struct 
{
  JOINT_ARRAY_MODE	ctrl_mode[MAX_NDOF];
  mReal			tq_desired[MAX_NDOF];
  mReal			q_desired[MAX_NDOF];
  mReal			slew_rate_q_desired[MAX_NDOF];
  mReal			q_stiffness[MAX_NDOF];
}M3JntArrayShmSdsCommand;


typedef struct
{    
    mReal			theta[MAX_NDOF];			
    mReal			thetadot[MAX_NDOF];			
    mReal			torque[MAX_NDOF];    
}M3JntArrayShmSdsStatus;

typedef struct
{
    mReal 			wrench[6];
}M3LoadX6ShmSdsStatus;

typedef struct
{
  int64_t	timestamp;
  M3JntArrayShmSdsStatus right_arm;    
  M3JntArrayShmSdsStatus right_hand;
}M3BotShmSdsStatus;

typedef struct
{
  int64_t	timestamp;
  M3JntArrayShmSdsCommand right_arm;    
  M3JntArrayShmSdsCommand right_hand;
  M3LoadX6ShmSdsStatus right_loadx6;
}M3BotShmSdsCommand;

#endif