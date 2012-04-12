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

#ifndef __ADC_H__
#define __ADC_H__ 
#include "setup.h"

#ifdef USE_ADC

//#define ADC_NUM_SMOOTH	32 		//Must be even
//#define ADC_SHIFT_SMOOTH 5 		//2^ADC_SHIFT_SMOOTH

#define ADC_NUM_SMOOTH	8 		//Must be even
#define ADC_SHIFT_SMOOTH 0 		//2^ADC_SHIFT_SMOOTH

#define ADC_NUM_SMOOTH_FAST	16 	//Must be even
#define ADC_SHIFT_SMOOTH_FAST 4 //2^ADC_SHIFT_SMOOTH_FAST
#define ADC_NUM_CH 4 
#define ADC_CURRENT_A 0
#define ADC_CURRENT_B 1
#define ADC_AMP_TEMP 2
#define ADC_MOTOR_TEMP 3
#define ADC_EXT 3			//SEAX2-1.2 is configured to take load-cell on the motor temp connector
#define DMA_BUF_DEPTH 			4 //32//128		//
#define DMA_FILTER_SHIFT		5		//128 = 2^7 : 2^DMA_FILTER_SHIFT = DMA_BUF_DEPTH

void setup_adc(void);
void setup_dma1(void);
unsigned int get_avg_adc(int idx);
extern unsigned int adc_raw[];
unsigned int get_avg_adc_torque();
unsigned int * current_dma_buf();
void adc_filter(int *y, int x );
int current_control(int current_command, int current_reading);

#endif
#endif
