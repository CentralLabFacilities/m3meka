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

#ifndef CURRENT2_H
#define CURRENT2_H

// for Q6 format, multiplier is .4578 mA/tick
// 32768*.4578 = 15001
//#define CURRENT_MA_MULT     15001
//#define CURRENT_MA_SHIFT    15

// for Q5 format, multiplier is .9155 mA/tick
// .9155 = 3.3/1024 * 1000/.11 * 1/2^5
// 32768*.9155 =
#define CURRENT_MA_MULT     30000
#define CURRENT_MA_SHIFT    15
// other direction
#define CURRENT_ADC_MULT    35791


int current_control();
int get_current_ma();
void set_current_command_ma(int current_desired_ma);
void set_current_ab(int i_a, int i_b);
int adc2ma(int adc);
int ma2adc(int ma);
int get_max_current_ma();

#endif

