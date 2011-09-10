/* 
M3 -- Meka Robotics Real-Time Control System
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

#ifndef M3EC_PDO_V3_H
#define M3EC_PDO_V3_H

#ifdef __KERNEL__
#define int16_t short
#define int32_t int
#else
#ifndef EMBEDDED
#include <sys/types.h>
#include <stdint.h>
#endif
#endif


typedef struct 
{
	int16_t		current_ma;		//Calibrated value
	int16_t		debug;			//Reserved
	int16_t		adc_torque;		//Torque input 
	int16_t		adc_motor_temp;		//Motor temp
	int16_t		adc_amp_temp;		//Amplifier temp 
	int16_t		adc_current_a;		//Motor current leg A 
	int16_t		adc_current_b;		//Motor current leg B 
	int16_t		pwm_cmd;		//Current PWM command to motor
	int16_t		qei_on;			//Encoder ticks
	int16_t		qei_period;		//Encoder ticks
	int16_t		qei_rollover;		//Encoder rollover counts (directional) -1|0|1 ...
	int16_t		flags;			//Reserved
}M3ActPdoV3Status;

///////////////////////////////  M3ACT_PDO_V1 Multi-Channel /////////////////////////////////////////////////////

typedef struct
{
	uint64_t    timestamp;		    //Time in us
	M3ActPdoV3Status 	status[1];
} M3ActX1PdoV3Status;





#endif
