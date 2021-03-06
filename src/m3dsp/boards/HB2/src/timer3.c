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

#ifdef USE_TIMER3

#include "timer3.h"
#include "setup.h"

int tmp_cnt;
int irq_cnt;

void setup_timer3(void)
{
	// ToDo: Setup timer3 for Ethercat distributed clock jitter calculation
	// The ECAT_TIMER (16 bit timer) runs from 0 to 0xFFFF round and round, 
	// the ECAT_CAPTURE_REG captures the ECAT_TIMER_REG when the ESC interrupt goes active
	irq_cnt=0;
	T3CON = 0;
	TMR3 = 0x0000;
	T3CONbits.TCKPS=T3_TCKPS;					
	PR3 = (unsigned int)T3_PR3;					
	_T3IF = 0;									//Clear interrupt
	T3CONbits.TON = 1;							//Start Timer 3
	_T3IE = 1;									//Enable T3 ints
	return;
}

//Every X us
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void) {
	_T3IF = 0;


//Latch encoder timestamp on Rising edge.
#ifdef USE_TIMESTAMP_DC
	SetTimestampLatch;
	ClrTimestampLatch;
#endif

//ToggleHeartbeatLED();

#if defined HB2_H2R1_J0J1 || defined HB2_H2R1_J2J3J4  ||defined HB2_H2R2_J0J1 || \
	defined HB2_H2R2_J2J3J4 || defined HB2_0_2_H2R3_J0J1 || defined HB2_0_2_H2R3_J2J3J4
#ifdef USE_CONTROL	
	step_control();
#endif
	irq_cnt++;
#endif
}

#endif
