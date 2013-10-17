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

#ifndef M3EC_PDO_VM_H
#define M3EC_PDO_VM_H

#ifdef __KERNEL__
#define int16_t short
#define int32_t int
#define uint8_t unsigned char
#else
#ifndef EMBEDDED
#include <sys/types.h>
#include <stdint.h>
#endif
#endif

///////////////////////////////  From M3ACT_PDO_V4 /////////////////////////////////////////////////////

//V4 is for IQ version of actuator (current commanded)

typedef struct 
{
	int32_t		mode;					//Reserved
	uint8_t[28]	rt_control_command;   			//P gain, torque control
	uint8_t[28]	rt_control_command_2;			//I gain, torque control
	int32_t		toggle;					//D gain, torque control
	uint8_t[28]	rpc_packet;				//Shift scalar, torque control
} M3ActPdoVMCmd;



typedef struct 
{
	int32_t		motor_torque;		//Calibrated value
	int32_t		motor_iq;			//Reserved [vertx err cnts]
	int32_t		motor_vq_avg;			//Torque input 
	int32_t		analog_1;		//Err
	int32_t		analog_1_dot;		//Motor temp
	int32_t		analog_2;		//Amplifier temp 
	int32_t		analog_2_dot;		//Motor current leg A 
	int32_t		analog_diff;		//Motor current leg B 
	int32_t		analog_diff_dot;		//PWM command to motor
	int32_t		quadrature_1;			//Encoder ticks
	int32_t		quadrature_1_dot;		//Encoder ticks
	int32_t		quadrature_2;		//Encoder rollover counts (directional) -1|0|1 ...
	int32_t		quadrature_2_dot;		//Err
	int32_t		ssi;			//Reserved
        int32_t         ssi_dot;              // calculated from the hall effect sensors
	int32_t		fault_flags;		//Encoder rollover counts (directional) -1|0|1 ...
	int32_t		bus_v;		//Err
	int32_t		debug;			//Reserved
        uint8_t[28]     rpa_packet;              // calculated from the hall effect sensors

}M3ActPdoVMStatus;



#endif
