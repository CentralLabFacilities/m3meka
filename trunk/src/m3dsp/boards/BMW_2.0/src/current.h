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


// sensitivity = 185 mV/A
// for Q5 format, multiplier is .5444 mA/tick
// .5444 = 3.3/1024 * 1000/.185 * 1/2^5
// 2^16*.5444 =
#define CURRENT_MA_MULT     35676
#define CURRENT_MA_SHIFT    16
// other direction 2^15*1/.5444
#define CURRENT_ADC_MULT    60191
#define CURRENT_ADC_SHIFT   15


int current_control();
int get_current_ma();
void set_current_command_ma(int current_desired_ma);
void set_current_ab(int i_a, int i_b);
int adc2ma(int adc);
int ma2adc(int ma);
int get_max_current_ma();

#endif

