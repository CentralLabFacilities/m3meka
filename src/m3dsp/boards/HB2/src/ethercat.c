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

#ifdef USE_ETHERCAT

#include "setup.h"
#include "ethercat.h"
#include "ethercat_def.h"
#include "ethercat_slave_fsm.h"

void isr_update_outputs(void);
void isr_update_inputs(void);
void isr_update_input_pdo(void);


void ISR_EscWriteAccess( UINT8 *pData, UINT16 Address, UINT16 Len );
void ISR_EscReadAccess( UINT8 *pData, UINT16 Address, UINT16 Len );

ec_cmd_t  ec_cmd;
ec_stat_t   ec_stat;

#if defined USE_ACTX1_PDO_V2
ec_cmd_in_t  ec_cmd_in;
ec_stat_out_t   ec_stat_out;
ec_stat_ext_t	ec_stat_ext;
ec_cmd_ext_t	ec_cmd_ext;
int stat_ext_idx[NUM_DBG_CH];
#endif

unsigned char pdo_cmd[PDO_COMMAND_SIZE];
unsigned char pdo_stat[PDO_STATUS_SIZE];

int eeprom_loaded(void){return EEPROM_LOADED;}

int ec_wd_expired;
long ec_wd_timestamp;
int ec_active;

int ec_debug[NUM_DBG_CH];
int ec_flags[NUM_DBG_CH];

//This should be done in a more general way. Brute force manual copy for now...
//Multiplex non time critical data in EtherCAT packets to reduce packet size.
void unpack_command_in()
{
  
#if defined USE_ACTX1_PDO_V2
      int ch;
      for (ch=0;ch<NUM_DBG_CH;ch++)
      {
	  ec_cmd.command[ch].config=ec_cmd_in.command[ch].config;
	  ec_cmd.command[ch].t_desire=ec_cmd_in.command[ch].t_desire;
	  ec_cmd.command[ch].mode=ec_cmd_in.command[ch].mode;
	  ec_cmd.command[ch].config=ec_cmd_in.command[ch].config;
	  
	  //Pass in EXT data
	  int ext_sz = sizeof(M3ActPdoV2CmdExt);
	  int num_copy=MIN(M3ACT_PDO_V2_CMD_EXT_SZ,ext_sz); //max num to copy 2
	  int start = ec_cmd_in.command[ch].ext_start_idx; 
	  int end = XINC_MOD(start,ext_sz,num_copy-1); 
	  if (end<start) //handle roll-over
	  {
	    int nc1=ext_sz-start; 
	    int nc2=num_copy-nc1; 
	    memcpy(((unsigned char *)&ec_cmd_ext)+start,(unsigned char *)ec_cmd_in.command[ch].ext,nc1);
	    memcpy((unsigned char *)&ec_cmd_ext,(unsigned char *) ec_cmd_in.command[ch].ext+nc1,nc2);
	  }
	  else
	    memcpy(((unsigned char *)&ec_cmd_ext)+start,(unsigned char *) ec_cmd_in.command[ch].ext,num_copy);
	  
	  ec_cmd.command[ch].k_p=ec_cmd_ext.k_p;
	  ec_cmd.command[ch].k_i=ec_cmd_ext.k_i;
	  ec_cmd.command[ch].k_d=ec_cmd_ext.k_d;
	  ec_cmd.command[ch].k_p_shift=ec_cmd_ext.k_p_shift;
	  ec_cmd.command[ch].k_i_shift=ec_cmd_ext.k_i_shift;
	  ec_cmd.command[ch].k_d_shift=ec_cmd_ext.k_d_shift;
	  ec_cmd.command[ch].k_i_limit=ec_cmd_ext.k_i_limit;
	  ec_cmd.command[ch].t_max=ec_cmd_ext.t_max;
	  ec_cmd.command[ch].t_min=ec_cmd_ext.t_min;
	  ec_cmd.command[ch].pwm_max=ec_cmd_ext.pwm_max;
	  ec_cmd.command[ch].qei_max=ec_cmd_ext.qei_max;
	  ec_cmd.command[ch].qei_min=ec_cmd_ext.qei_min;
	  ec_cmd.command[ch].k_ff_zero=ec_cmd_ext.k_ff_zero;
	  ec_cmd.command[ch].k_ff_shift=ec_cmd_ext.k_ff_shift;
	  ec_cmd.command[ch].k_ff=ec_cmd_ext.k_ff;
	  ec_cmd.command[ch].pwm_db=ec_cmd_ext.pwm_db;
      }
#endif
  
}

void pack_status_out()
{
#if defined USE_ACTX1_PDO_V2
      ec_stat_out.timestamp=ec_stat.timestamp;
      int ch;
      for (ch=0;ch<NUM_DBG_CH;ch++)
      {
		  int ext_sz = sizeof(M3ActPdoV2StatusExt);
	int num_copy=MIN(M3ACT_PDO_V2_STATUS_EXT_SZ,ext_sz);
	ec_stat_out.status[ch].qei_period=ec_stat.status[ch].qei_period;
	ec_stat_out.status[ch].qei_on=ec_stat.status[ch].qei_on;
	ec_stat_out.status[ch].qei_rollover=ec_stat.status[ch].qei_rollover;
	ec_stat_out.status[ch].adc_torque=ec_stat.status[ch].adc_torque;
	ec_stat_out.status[ch].pwm_cmd=ec_stat.status[ch].pwm_cmd;
	ec_stat_out.status[ch].flags=ec_stat.status[ch].flags;
	ec_stat_out.status[ch].ext_start_idx=stat_ext_idx[ch]; //copy pointer

	ec_stat_ext.debug=ec_stat.status[ch].debug;
	ec_stat_ext.adc_motor_temp=ec_stat.status[ch].adc_motor_temp;
	ec_stat_ext.adc_ext_a=ec_stat.status[ch].adc_ext_a;
	ec_stat_ext.adc_ext_b=ec_stat.status[ch].adc_ext_b;
	ec_stat_ext.adc_amp_temp=ec_stat.status[ch].adc_amp_temp;
	ec_stat_ext.adc_current_a=ec_stat.status[ch].adc_current_a;
	ec_stat_ext.adc_current_b=ec_stat.status[ch].adc_current_b;
	//Copy portion of EXT struct to ext buffer
	int start = stat_ext_idx[ch];
	int end=XINC_MOD(start,ext_sz,num_copy-1);
	if (end<start) //rollover
	{
	  int nc1=ext_sz-start; 
	  int nc2=num_copy-nc1; 
	  memcpy(ec_stat_out.status[ch].ext,((unsigned char *)&ec_stat_ext)+start,nc1);
	  memcpy(ec_stat_out.status[ch].ext+nc1,((unsigned char *)&ec_stat_ext),nc2);
	}
	else
	  memcpy(ec_stat_out.status[ch].ext,((unsigned char *)&ec_stat_ext)+start,num_copy);
	//Advance the copy pointer.*/
	stat_ext_idx[ch]=XINC_MOD(stat_ext_idx[ch],ext_sz,num_copy);
      }
#endif  
}


/////////////////////////////////////////////////////////////
int step_ethercat(void)
{
	ECAT_Main();		/* check if masked interrupts were received */
	return 0;
}

void isr_reset_outputs(void)
{
}

/////////////////////////////////////////////////////////////
void isr_update_outputs(void)
{
	if ( EscAlEvent.Byte[1] & (PROCESS_OUTPUT_EVENT>>8) )	/* check if the watchdog should be reset */
	{
		bEcatFirstOutputsReceived = 1;						/* reset watchdog */
#ifdef EC_USE_WATCHDOG
		ec_wd_timestamp = 0;//ECAT_TIMER_REG;
		ec_wd=0;
#endif
		ISR_EscReadAccess( (unsigned char *) pdo_cmd, nEscAddrOutputData, nPdOutputSize );
		
#if defined USE_ACTX1_PDO_V2
	memcpy((unsigned char *)&ec_cmd_in,pdo_cmd,sizeof(ec_cmd_in_t));
	unpack_command_in();
#else
		memcpy((unsigned char *)&ec_cmd,pdo_cmd,sizeof(ec_cmd_t));
#endif

//#if defined USE_LED_MDRV
//	load_led_mdrv();
//#endif
	}
}
/////////////////////////////////////////////////////////////

void isr_update_inputs(void)
{
	unsigned long long dc_timestamp;
#ifndef USE_SYNC0
	isr_update_input_pdo();
#endif

#ifdef USE_TIMESTAMP_DC
	ISR_EscReadAccess((unsigned char*)&dc_timestamp, (unsigned int)EC_LATCH1_POS_EDG_ADDR, (unsigned int)8 );
	ec_stat.timestamp=dc_timestamp;
#else
	ec_stat.timestamp=0;
#endif
#if defined USE_ACTX1_PDO_V2
	pack_status_out();
	memcpy(pdo_stat,(unsigned char *)&ec_stat_out,sizeof(ec_stat_out_t));
#else
	memcpy(pdo_stat,(unsigned char *)&ec_stat,sizeof(ec_stat_t));
#endif
	ISR_EscWriteAccess( (UINT8 *) pdo_stat, nEscAddrInputData, nPdInputSize );
	return;
}

void isr_update_input_pdo(void)
{
#if defined HB2_H2R1_J0J1 \
	|| defined HB2_H2R1_J2J3J4 || defined HB2_H2R2_J0J1 || defined HB2_H2R2_J2J3J4 \
	|| defined HB2_0_2_H2R3_J0J1 || defined HB2_0_2_H2R3_J2J3J4

#ifdef USE_ENCODER_MA3
	ec_stat.status[0].qei_period=ma3_period(0);
	ec_stat.status[1].qei_period=ma3_period(1);
	ec_stat.status[0].qei_on=ma3_on(0);
	ec_stat.status[1].qei_on=ma3_on(1);
	ec_stat.status[0].qei_rollover=ma3_rollover(0);
	ec_stat.status[1].qei_rollover=ma3_rollover(1);
#if defined HB2_H2R1_J2J3J4 || defined HB2_H2R2_J2J3J4 || defined HB2_0_2_H2R3_J2J3J4
	ec_stat.status[2].qei_period=ma3_period(2);
	ec_stat.status[2].qei_on=ma3_on(2);
	ec_stat.status[2].qei_rollover=ma3_rollover(2);
#endif
#endif

#if defined USE_ADC
#if defined HB2_H2R1_J0J1 
	ec_stat.status[0].adc_torque=get_avg_adc(ADC_SEAS_A);
	ec_stat.status[1].adc_torque=get_avg_adc(ADC_SEAS_B);
	ec_stat.status[0].adc_motor_temp=get_avg_adc(ADC_MOTOR_TEMP_A);
	ec_stat.status[1].adc_motor_temp=get_avg_adc(ADC_MOTOR_TEMP_B);
	ec_stat.status[0].adc_amp_temp=get_avg_adc(ADC_AMP_TEMP_A);
	ec_stat.status[1].adc_amp_temp=get_avg_adc(ADC_AMP_TEMP_B);
#endif
#if defined HB2_H2R2_J0J1  || defined HB2_0_2_H2R3_J0J1 
	ec_stat.status[0].adc_torque=get_avg_adc(ADC_SEAS_A);
	ec_stat.status[1].adc_torque=get_avg_adc(ADC_SEAS_B);
	ec_stat.status[0].adc_amp_temp=get_avg_adc(ADC_AMP_TEMP_A);
	ec_stat.status[1].adc_amp_temp=get_avg_adc(ADC_AMP_TEMP_B);
	ec_stat.status[0].adc_motor_temp=ec_stat.status[0].adc_amp_temp;//shared, approximate as ambient temp
	ec_stat.status[1].adc_motor_temp=ec_stat.status[1].adc_amp_temp;//shared, approximate as ambient temp
	ec_stat.status[0].adc_current_a=get_avg_adc(ADC_CURRENT_A);
	ec_stat.status[1].adc_current_a=get_avg_adc(ADC_CURRENT_B);
#endif
#if defined HB2_H2R1_J2J3J4 
	ec_stat.status[0].adc_torque=get_avg_adc(ADC_SEAS_A);
	ec_stat.status[1].adc_torque=get_avg_adc(ADC_SEAS_B);
	ec_stat.status[2].adc_torque=get_avg_adc(ADC_SEAS_C);
	ec_stat.status[0].adc_motor_temp=get_avg_adc(ADC_MOTOR_TEMP_A);
	ec_stat.status[1].adc_motor_temp=get_avg_adc(ADC_MOTOR_TEMP_B);
	ec_stat.status[2].adc_motor_temp=get_avg_adc(ADC_MOTOR_TEMP_C);
	ec_stat.status[0].adc_amp_temp=get_avg_adc(ADC_AMP_TEMP_A);
	ec_stat.status[1].adc_amp_temp=get_avg_adc(ADC_AMP_TEMP_B);
	ec_stat.status[2].adc_amp_temp=ec_stat.status[1].adc_amp_temp;//shared
#endif
#if defined HB2_H2R2_J2J3J4 || defined HB2_0_2_H2R3_J2J3J4
	ec_stat.status[0].adc_torque=get_avg_adc(ADC_SEAS_A);
	ec_stat.status[1].adc_torque=get_avg_adc(ADC_SEAS_B);
	ec_stat.status[2].adc_torque=get_avg_adc(ADC_SEAS_C);
	ec_stat.status[0].adc_amp_temp=get_avg_adc(ADC_AMP_TEMP_A);
	ec_stat.status[1].adc_amp_temp=ec_stat.status[0].adc_amp_temp;//shared
	ec_stat.status[2].adc_amp_temp=ec_stat.status[0].adc_amp_temp;//shared
	ec_stat.status[0].adc_motor_temp=ec_stat.status[0].adc_amp_temp;//shared
	ec_stat.status[1].adc_motor_temp=ec_stat.status[0].adc_amp_temp;//shared
	ec_stat.status[2].adc_motor_temp=ec_stat.status[0].adc_amp_temp;//shared
	ec_stat.status[0].adc_current_a=get_avg_adc(ADC_CURRENT_A);
	ec_stat.status[1].adc_current_a=get_avg_adc(ADC_CURRENT_B);
	ec_stat.status[2].adc_current_a=get_avg_adc(ADC_CURRENT_C);
#endif
#endif

#ifdef USE_PWM
	ec_stat.status[0].pwm_cmd=pwm_cmd(0);
	ec_stat.status[1].pwm_cmd=pwm_cmd(1);
#endif
	ec_stat.status[0].debug=ec_debug[0];
	ec_stat.status[1].debug=ec_debug[1];
	ec_stat.status[0].flags=ec_flags[0] | M3ACT_FLAG_QEI_CALIBRATED;
	ec_stat.status[1].flags=ec_flags[1] | M3ACT_FLAG_QEI_CALIBRATED;

#if defined HB2_H2R1_J2J3J4 || defined HB2_H2R2_J2J3J4 || defined HB2_0_2_H2R3_J2J3J4
#if defined USE_PWM
	ec_stat.status[2].pwm_cmd=pwm_cmd(2);
#endif
	ec_stat.status[2].debug=ec_debug[2];
	ec_stat.status[2].flags=ec_flags[2] | M3ACT_FLAG_QEI_CALIBRATED;
#endif

#endif
}

/////////////////////////////////////////////////////////////
void __attribute__((__interrupt__, no_auto_psv)) _INT0Interrupt(void)
{
	//Interrupt service routine for the interrupts from the EtherCAT Slave Controller
	//Do the processing of PDOs here

	/* INTERRUPT_PROTECT_ENABLE acts as an atomic lock on the data. This can protect
	against mode change blips, etc. However it causes priority issues, in particular it affects
	the TIMER3 timing with VertX. Disabling it doesn't seem to affect performance.*/

	SPI_SEL = SPI_DEACTIVE;				/* SPI should be deactivated to interrupt a possible transmission */
	ACK_ESC_INT;						/* reset the interrupt flag */
	
	ISR_GetInterruptRegister();			/* get the AL event in EscAlEvent */
#if DC_SUPPORTED 
	if ( bDcSyncActive )				/* read the sync 0 status to acknowledge the SYNC interrupt */
		ISR_EscReadAccess( dummy, ESC_ADDR_SYNC_STATUS, 2);
#endif

	if (bSynchronMode)						/* Application is synchronized to DC-, SM2- or SM3-event */
	{		
		if ( bEcatOutputUpdateRunning )		// Get Process Outputs
			isr_update_outputs();				// EtherCAT slave is in OPERATIONAL, update outputs 
		else
			isr_reset_outputs();
		if ( bEcatInputUpdateRunning )		//Update Process Inputs
			isr_update_inputs();				/* EtherCAT slave is at least in SAFE-OPERATIONAL, update inputs */

	}	
	ec_active=1;	
}


#ifdef USE_SYNC0
void __attribute__((__interrupt__, no_auto_psv)) _INT2Interrupt(void)
{
	//Interrupt service routine for the interrupts from the EtherCAT Slave Controller
	//Do the processing of PDOs here
	UINT8 dummy[2];

	ACK_SYNC_INT;						/* reset the interrupt flag */

//	if ( bDcSyncActive )				/* read the sync 0 status to acknowledge the SYNC interrupt */
//		ISR_EscReadAccess( dummy, ESC_ADDR_SYNC_STATUS, 2);

if (bSynchronMode)						/* Application is synchronized to DC-, SM2- or SM3-event */
	{	
		if ( bEcatInputUpdateRunning )		//Update PDO datastructure for INT0
			isr_update_input_pdo();				/* EtherCAT slave is at least in SAFE-OPERATIONAL, update inputs */   
	}
	ec_active=1;
}
#endif


/////////////////////////////////////////////////////////////
void setup_ethercat(void)
{  
	int i;
	UINT8 u8PDICtrl = 0;

	/* Hardcode PDO size for now*/
	nPdInputSize=PDO_STATUS_SIZE;	
	nPdOutputSize=PDO_COMMAND_SIZE;
	
	// Initialize structs
	memset((unsigned char *)&ec_stat,0,sizeof(ec_stat_t));
	memset((unsigned char *)&ec_cmd,0,sizeof(ec_cmd_t));
#if defined USE_ACTX1_PDO_V2
       memset((unsigned char *)&ec_stat_out,0,sizeof(ec_stat_out_t));
       memset((unsigned char *)&ec_cmd_in,0,sizeof(ec_cmd_in_t));
       memset((unsigned char *)&ec_cmd_ext,0,sizeof(ec_cmd_ext_t));
       memset((unsigned char *)&ec_stat_ext,0,sizeof(ec_stat_ext_t));
       for (i=0;i<NUM_DBG_CH;i++)
		 stat_ext_idx[i]=0;
#endif
	memset(pdo_cmd,0,PDO_COMMAND_SIZE);
	memset(pdo_stat,0,PDO_STATUS_SIZE);

	for (i=0;i<NUM_DBG_CH;i++)
	{
		ec_debug[i]=0;
		ec_flags[i]=0;
	}

	ec_wd_expired=0;
	ec_wd_timestamp=0;
	ec_active=0;
	//Setup SPI
	setup_spi();

	//Setup synchronization and timers
	INTCON2bits.INT0EP=1;			//INT0 on falling edge
	INTCON2bits.INT1EP=1;			//INT1 on falling edge
	INTCON2bits.INT2EP=1;			//INT2 on falling edge

	SPI_SEL = SPI_DEACTIVE;		/* deactivate SPI */

	ACK_ESC_INT;				/* reset ESC interrupt */
#ifdef USE_SYNC0
	ACK_SYNC_INT;  				/* RJK: reset INT2 for SYNC0 */
#endif
#if DC_SUPPORTED
	ACK_ECAT_TIMER_INT;
	DISABLE_ECAT_TIMER_INT;
#endif
#if DC_SUPPORTED
//	ECAT_CAPTURE_CONFIG_REG = ESC_CAPTURE;
#endif

	//Make sure the ESC is booted completely
	do
	{
		//Note: If PDI Type is SPI, DPRAM size could be changed in future (ASIC)
		HW_EscReadAccess( (UINT8 *)(&u8PDICtrl), ESC_ADDR_PDICTRL, 1 );
		//ToggleHeartbeatLED();
	} while (u8PDICtrl != 5);
	
	HW_EscReadAccess(&nMaxSyncMan, ESC_ADDR_SYNCMANCHANNELS, 1 );

	HW_ResetIntMask(0);

#if AL_EVENT_ENABLED
	ENABLE_ESC_INT;
#ifdef USE_SYNC0
	ENABLE_SYNC_INT;   /* RJK: Enable INT2 for SYNC0  */
#endif
#endif

	
	///////////////////////////////////////////////////
	//Prepare to run 
	ECAT_Init();					//Initialize the EtherCAT Slave Interface
}

void setup_spi(void)
{
		/*	
	Initialize and enable the SPI as: 
	* SPI-Master with clock Fosc/4=10Mhz
	* High level of clock as idle state
	* SPI_MODE == 3 
	* LATE_SAMPLE = FALSE
	* Save input data at middle of data output time
	* Data transmitting on rising edge of SC 
*/

	//The ET1200 should be configured to SPI Mode 3
	//SPI_CLK is then max 50ns, or 20Mhz 
	//FCSK = FCY/(PrimaryPrescale*SecondaryPrescale) = 40M/(4*2)=5Mhz

	_SPI1IF=0; //Clear interrupt
	_SPI1IE=0; //Disable interrupt
	
	SPI1CON1=0;
	SPI1CON1bits.MSTEN=1;		//Act as SPI Master
	SPI1CON1bits.PPRE=0b10;		// ((0b10, 4:1), (0b00, 64:1) (0b11,1:1) prescalar
	SPI1CON1bits.SPRE=0b111;	// (0b111, 1:1),(0b110, 2:1) secondary scalar
    SPI1CON1bits.CKP=1;			//Idle state is high
    SPI1CON1bits.CKE=0;			//Data changes on clock transition from idle to active
    SPI1CON1bits.SMP=0;			//Input data sampled at middle of data output time 
    SPI1CON1bits.MODE16=0;		//Communication is 8bit bytes

	SPI1STATbits.SPIROV=0;	//Clear overflow flag
	SPI1STATbits.SPIEN=1;	//Enable SPI
	SPI_SEL = SPI_DEACTIVE; //Clear slave select (active low)
	//_SPI1IE=1; //Enable interrupt
}

#endif
