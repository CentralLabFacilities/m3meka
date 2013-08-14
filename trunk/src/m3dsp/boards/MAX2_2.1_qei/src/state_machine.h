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

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

enum dsp_state {
    DSP_OFF,
    DSP_STARTUP,
    DSP_PWM,
    DSP_CURRENT,
    DSP_BRAKE,
    DSP_ERROR
};



void step_state();
enum dsp_state get_dsp_state();
int limit_check(int item, int compare, int* count, int count_max );

#endif
