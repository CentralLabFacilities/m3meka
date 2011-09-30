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

#ifdef USE_CURRENT

#include "setup.h"
#include "current.h"
extern int test;


//Assuming step is called at 2kHz
#define I_RMS_MOM_BUF_SZ 16
#define I_RMS_MOM_BUF_SHIFT 4
#define I_RMS_MOM_DS 248 //2000Hz to 8.06Hz, Buff of 16 gives ~2s window size.

//Assuming step is called at 2kHz
#define I_RMS_CONT_BUF_SZ 32
#define I_RMS_CONT_BUF_SHIFT 5
#define I_RMS_CONT_DS 624 //2000Hz to 3.2Hz. Buff of 32 gives ~10S window size.

volatile int i_mA;
volatile unsigned int i_zero_a, i_zero_b;
volatile int i_zero_cnt;
volatile unsigned long i_rms_mom_buf[I_RMS_MOM_BUF_SZ];
volatile unsigned long i_rms_cont_buf[I_RMS_CONT_BUF_SZ];
volatile unsigned int i_zero_sum_a, i_zero_sum_b;
volatile long i_rms_cont_sq, i_rms_mom_sq, i_fault_val; //squared value
volatile int i_rms_mom_ds,i_rms_cont_ds;
volatile unsigned long i_rms_mom_sum, i_rms_cont_sum;
volatile int i_rms_mom_idx,i_rms_cont_idx;
volatile int i_state,i_fault_cont,i_fault_mom;


int current_fault_mom_flag()
{
   if (i_fault_mom)
     return M3ACT_FLAG_I_FAULT_MOM;
   else
     return 0;
}

int current_fault_cont_flag()
{
     if (i_fault_cont)
     return M3ACT_FLAG_I_FAULT_CONT;
   else
     return 0;
}

int get_current_ma()
{
  return i_mA;
}

long get_current_rms_mom_sq_ma()
{
  return i_rms_mom_sq;
}
long get_current_rms_cont_sq_ma()
{
  return i_rms_cont_sq;
}
int get_current_state()
{
  return i_state;
}

void reset_current_buf()
{
  	i_mA=0;
 	i_rms_cont_idx=0;
 	i_rms_cont_sum=0;
 	i_rms_cont_sq=0;	
 	i_rms_cont_ds=0;
	i_rms_mom_idx=0;
	i_rms_mom_sum=0;
	i_rms_mom_sq=0;	
	i_rms_mom_ds=0;
  	memset((unsigned long *)i_rms_mom_buf,0,sizeof(unsigned long)*I_RMS_MOM_BUF_SZ);		//WAS unsigned char, all 4
  	memset((unsigned long *)i_rms_cont_buf,0,sizeof(unsigned long)*I_RMS_CONT_BUF_SZ);
}

void step_current()
{
  //Measure zero sensor reading at startup
  if (i_zero_cnt<5 && i_zero_cnt>0)
  {
#if defined M3_MAX2_BDC_A2R4
      i_zero_sum_a = i_zero_sum_a + get_avg_adc(ADC_CURRENT_A);
      i_zero_sum_b = i_zero_sum_b + get_avg_adc(ADC_CURRENT_B);
      if (i_zero_cnt ==1 )
      {
		i_zero_a = (i_zero_sum_a >> 2);
		i_zero_b = (i_zero_sum_b >> 2);
		i_state = CURRENT_READY;
      }
#endif
  }

  i_zero_cnt=MAX(0,i_zero_cnt-1);

if (i_state != CURRENT_STARTUP)
{
	//Compute 'instantaneous' current
	#if defined M3_MAX2_BDC_A2R4
	  int x =(int)get_avg_adc(ADC_CURRENT_A) - (int)i_zero_a;
	  //i_mA=(int)((float)x * (float)ADC_CURRENT_MA_PER_TICK);
	i_mA = (x * ADC_CURRENT_MA_PER_TICK);	//WAS: New version, int
	#endif

/*
	  //Now compute the momentary RMS value
	  i_rms_mom_ds=INC_MOD(i_rms_mom_ds,I_RMS_MOM_DS);
	  if (i_rms_mom_ds==0)
	  {
	    i_rms_mom_sum=i_rms_mom_sum-i_rms_mom_buf[i_rms_mom_idx];
	    i_rms_mom_buf[i_rms_mom_idx]=(long)i_mA*(long)i_mA;
	    i_rms_mom_sum=i_rms_mom_sum+i_rms_mom_buf[i_rms_mom_idx];
	    i_rms_mom_sq=i_rms_mom_sum>>I_RMS_MOM_BUF_SHIFT;
	    i_rms_mom_idx=INC_MOD(i_rms_mom_idx,I_RMS_MOM_BUF_SZ);
	  }

	   	if (!i_fault_mom && i_rms_mom_sq>CURRENT_MAX_MOM_RMS_SQ)
		{
			//if (i_state==CURRENT_READY)
			//	i_fault_val=i_rms_mom_sq;
			i_state=CURRENT_FAULT_MOM;
			i_fault_mom=1;
			reset_current_buf();//power turns off
		}

		 //Now compute the continuous RMS value
		 i_rms_cont_ds=INC_MOD(i_rms_cont_ds,I_RMS_CONT_DS);
		  if (i_rms_cont_ds==0)
		  {
			
		    i_rms_cont_sum-=i_rms_cont_buf[i_rms_cont_idx];
		    i_rms_cont_buf[i_rms_cont_idx]=i_rms_mom_sq;
		    i_rms_cont_sum+=i_rms_mom_sq;
			i_rms_cont_sq=i_rms_cont_sum>>I_RMS_CONT_BUF_SHIFT;
		    i_rms_cont_idx=INC_MOD(i_rms_cont_idx,I_RMS_CONT_BUF_SZ);
		  }
 		if (!i_fault_cont && i_rms_cont_sq>CURRENT_MAX_CONT_RMS_SQ)
		{
		//	if (i_state==CURRENT_READY)
			//	i_fault_val=i_rms_cont_sq;
			i_state=CURRENT_FAULT_CONT;
			i_fault_cont=1;
			reset_current_buf();//power turns off

		}
*/
	}

}

void setup_current()
{
  	i_mA=0;
  	i_zero_a=0;
  	i_zero_b=0;
  	i_zero_cnt=200;
  	i_zero_sum_a=0;
  	i_zero_sum_b=0;
 	i_rms_cont_idx=0;
 	i_rms_cont_sum=0;
 	i_rms_cont_sq=0;	
 	i_rms_cont_ds=0;
	i_fault_cont=0;
	i_fault_mom=0;
  	i_rms_mom_idx=0;
  	i_rms_mom_sum=0;
  	i_rms_mom_sq=0;	
  	i_rms_mom_ds=0;
  	i_state=CURRENT_STARTUP;
  	memset((unsigned long *)i_rms_mom_buf,0,sizeof(unsigned long)*I_RMS_MOM_BUF_SZ);
  	memset((unsigned long *)i_rms_cont_buf,0,sizeof(unsigned long)*I_RMS_CONT_BUF_SZ);
}

#endif //USE_CURRENT
