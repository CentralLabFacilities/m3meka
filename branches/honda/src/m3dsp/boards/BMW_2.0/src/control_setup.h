

#ifndef CONTROL_SETUP_H
#define CONTROL_SETUP_H

#include "p33Fxxxx.h"

///////////////////////////////////////////////////////
                /* BLDC */
///////////////////////////////////////////////////////

// ec 32 flat ribbon cable
#define BLDC_HALL_1 PORTBbits.RB0
#define BLDC_HALL_2 PORTBbits.RB1
#define BLDC_HALL_3 PORTBbits.RB2

#define BLDC_HALL_STATE (BLDC_HALL_3<<2)|(BLDC_HALL_2<<1)|BLDC_HALL_1

#define BLDC_PU_1 CNPU1bits.CN4PUE
#define BLDC_PU_2 CNPU1bits.CN5PUE
#define BLDC_PU_3 CNPU1bits.CN6PUE

#define BLDC_CN_1 CNEN1bits.CN4IE
#define BLDC_CN_2 CNEN1bits.CN5IE
#define BLDC_CN_3 CNEN1bits.CN6IE


///////////////////////////////////////////////////////
                /* CURRENT */
///////////////////////////////////////////////////////

// sensitivity = 185 mV/A
// for Q5 format, multiplier is .5444 mA/tick
// .5444 = 3.3/1024 * 1000/.185 * 1/2^5
// 2^16*.5444 =
#define CURRENT_MA_MULT     35676
#define CURRENT_MA_SHIFT    16
// other direction 2^15*1/.5444
#define CURRENT_ADC_MULT    60191
#define CURRENT_ADC_SHIFT   15

#endif

