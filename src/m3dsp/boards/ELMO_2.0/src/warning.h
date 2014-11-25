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

#ifndef DEF_WARNING_H	
#define DEF_WARNING_H	

//Compilation warnings:
#ifdef MAX2_BDC_0_3_A2R2
	#warning "Project: MAX2_BDC_0_3_A2R2"
#elif defined MAX2_BLDC_0_3_A2R2
	#warning "Project: MAX2_BLDC_0_3_A2R2"
#elif defined MAX2_BDC_0_3_T2R2
	#warning "Project: MAX2_BDC_0_3_T2R2"
#elif defined MAX2_BLDC_0_3_T2R2
	#warning "Project: MAX2_BLDC_0_3_T2R2"
#elif defined MAX2_BDC_0_2_A2R3
	#warning "Project: MAX2_BDC_0_2_A2R3"
#elif defined MAX2_BLDC_0_2_A2R3
	#warning "Project: MAX2_BLDC_0_2_A2R3"
#elif defined MAX2_BDC_0_2_T2R3
	#warning "Project: MAX2_BDC_0_2_T2R3"
#elif defined MAX2_BLDC_0_2_T2R3
	#warning "Project: MAX2_BLDC_0_2_T2R3"
#else
	#warning "How do you expect it to work if you don't specify what robot?"
#endif

//MAX2 version reminder
#ifndef USE_MAX2_0_2 
#ifndef USE_MAX2_0_3
#warning "Please specify your MAX2 version!"
#endif
#endif

#endif
