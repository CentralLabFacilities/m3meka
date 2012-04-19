
#include "setup.h"
#include "current2.h"

static int current_meas;
static int current_desired;

int current_control(int current_reading)
{
    static long ki_sum = 0;
    int error = current_desired-current_reading;
    int command;
    const int kp = ec_cmd.command[0].k_p;
    const int kp_shift = ec_cmd.command[0].k_p_shift;
    const int ki = ec_cmd.command[0].k_i;
    const int ki_shift = ec_cmd.command[0].k_i_shift;
    const long ki_limit = (long) ec_cmd.command[0].k_i_limit<<6;

    current_meas = current_reading;

    ki_sum += ki*error;
    ki_sum = CLAMP(ki_sum,-ki_limit,ki_limit);
    command = (__builtin_mulss(-kp,error)>>(6+kp_shift)) - (ki_sum>>(6+ki_shift));

    return (command);
}

int get_current_ma()
{
    //return (current_meas);
    return (__builtin_mulsu(current_meas,CURRENT_MA_MULT)>>CURRENT_MA_SHIFT);
}

int get_max_current_ma()
{
    // return the max of the two current sensors
    return (0);
}

void set_current_command_ma(int current_desired_ma)
{
    // sets current_desired in shifted adc ticks
    current_desired = __builtin_mulsu(current_desired_ma,CURRENT_ADC_MULT)>>CURRENT_MA_SHIFT;

}

