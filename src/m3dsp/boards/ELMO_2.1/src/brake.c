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

#ifdef USE_BRAKE

#include "brake.h"
#include "setup.h"

int frc_brake_on;
void step_brake(int off)
{

 if (off && !frc_brake_on)
 {
	 SetBrakeOff; 
 }
 else 
 {
	 SetBrakeOn;
}
}

void force_brake_on()
{
	frc_brake_on=1;
}
void reset_force_brake_on()
{
	frc_brake_on=0;
}

void setup_brake() 
{
	frc_brake_on=0;
}
#endif
