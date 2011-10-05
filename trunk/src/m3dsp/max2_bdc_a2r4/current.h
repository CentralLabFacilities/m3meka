/* 
M3 -- Meka Robotics Real-Time Control System
Copyright (c) 2011 Meka Robotics
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

#ifndef __CURRENT_H__
#define __CURRENT_H__ 

#ifdef USE_CURRENT

void setup_current();
void step_current();
//int step_current_pid();
int get_current_state();
int get_current_ma();
long get_current_rms_mom_sq_ma();
long get_current_rms_cont_sq_ma();
int current_fault_mom_flag();
int current_fault_cont_flag();
void reset_current_buf();

enum {
  CURRENT_STARTUP,
  CURRENT_READY,
  CURRENT_FAULT_MOM,
  CURRENT_FAULT_CONT};
  
// OP-Amp scale: G=0.6428 (5V to 3V3 using 1.5k/2.7k divider)
// Ticks per mV: S = 4096/3300 = 1.2412...
// ACS714-30: 1000/ (66.0 mv/A *G * S ) = 18.990 mA/tick
// ACS714-20: 1000/ (100.0 mv/A *G * S )= 12.523 mA/tick
// ACS714-5:  1000/ (185.0 mv/A *G * S ) = 6.775 mA/tick
#define ADC_CURRENT_MA_PER_TICK 13 //12.523 WAS float
#define MAX_MOM_CURRENT	20000	//20A
#define MAX_CONT_CURRENT 10000	//10A
#define CURRENT_MAX_MOM_RMS_SQ  400000000	// MAX_MOM_CURRENT*MAX_MOM_CURRENT		//ToDo Calibrate!
#define CURRENT_MAX_CONT_RMS_SQ 100000000	// MAX_CONT_CURRENT*MAX_CONT_CURRENT	//ToDo Calibrate!

#endif
#endif
