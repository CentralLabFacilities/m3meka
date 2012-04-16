#ifndef CURRENT2_H
#define CURRENT2_H

// for Q6 format, multiplier is .4578 mA/tick
// 32768*.4578 = 15001
//#define CURRENT_MA_MULT     15001
//#define CURRENT_MA_SHIFT    15

// for Q5 format, multiplier is .9155 mA/tick
// .9155 = 3.3/1024 * 1000/.11 * 1/2^5
// 32768*.9155 =
#define CURRENT_MA_MULT     30000
#define CURRENT_MA_SHIFT    15
// other direction
#define CURRENT_ADC_MULT    35791


int current_control(int current_reading);
int get_current_ma();
void set_current_command_ma(int current_desired_ma);

#endif

