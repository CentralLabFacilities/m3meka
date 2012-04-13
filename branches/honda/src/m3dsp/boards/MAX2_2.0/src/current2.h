#ifndef CURRENT2_H
#define CURRENT2_H

// for Q6 format, multiplier is .4578 mA/tick
// 32768*.4578 = 15001
#define CURRENT_MA_MULT     15001
#define CURRENT_MA_SHIFT    15

int current_control(int current_command, int current_reading);
int get_current_ma();

#endif

