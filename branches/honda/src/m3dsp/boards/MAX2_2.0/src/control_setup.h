

#ifndef CONTROL_SETUP_H
#define CONTROL_SETUP_H

#include "p33Fxxxx.h"

///////////////////////////////////////////////////////
                /* BLDC */
///////////////////////////////////////////////////////

#define PWM_4Q
// ec 45 flat ribbon cable
#define BLDC_HALL_2 PORTBbits.RB4		//RB4	INPUT	PIN33	RP4/CN1 (HALL1)
#define BLDC_HALL_1 PORTBbits.RB9		//RB9	INPUT	PIN1	RP9/CN21 (HALL2)
#define BLDC_HALL_3 PORTBbits.RB8		//RB8	INPUT	PIN44	RP8/CN22 (HALL3)

#define BLDC_HALL_STATE (BLDC_HALL_3<<2)|(BLDC_HALL_2<<1)|BLDC_HALL_1

#define BLDC_PU_1 CNPU1bits.CN1PUE
#define BLDC_PU_2 CNPU2bits.CN21PUE
#define BLDC_PU_3 CNPU2bits.CN22PUE

#define BLDC_CN_1 CNEN1bits.CN1IE
#define BLDC_CN_2 CNEN2bits.CN21IE
#define BLDC_CN_3 CNEN2bits.CN22IE

///////////////////////////////////////////////////////
                /* ADC */
///////////////////////////////////////////////////////

#define ADC_NUM_CH 4
#define ADC_CURRENT_A 0
#define ADC_CURRENT_B 1
#define ADC_AMP_TEMP 2
#define ADC_EXT_TEMP 3


///////////////////////////////////////////////////////
                /* CURRENT */
///////////////////////////////////////////////////////

// for Q5 format, multiplier is .9155 mA/tick
// .9155 = 3.3/1024 * 1000/.11 * 1/2^5
// 32768*.9155 =
#define CURRENT_MA_MULT     30000
#define CURRENT_MA_SHIFT    15
// other direction
#define CURRENT_ADC_MULT    35791
#define CURRENT_ADC_SHIFT   15

#define CURRENT_SIGN_TABLE  {0,-1,-1,-1,1,-1,1,0}

///////////////////////////////////////////////////////
                /* STATE_MACHINE */
///////////////////////////////////////////////////////

#define MAX_CURRENT_MA 13000

#endif

