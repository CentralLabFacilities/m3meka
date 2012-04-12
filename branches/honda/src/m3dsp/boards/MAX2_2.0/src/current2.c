
#include "setup.h"
#include "current2.h"

int current_control(int current_command, int current_reading)
{
    static long ki_sum = 0;
    int error = current_command<<6-current_reading;
    int command;
    const int kp = ec_cmd.command[0].k_p;
    const int kp_shift = ec_cmd.command[0].k_p_shift;
    const int ki = ec_cmd.command[0].k_i;
    const int ki_shift = ec_cmd.command[0].k_i_shift;
    const long ki_limit = (long) ec_cmd.command[0].k_i_limit<<6;

    error = current_command-current_reading;

    ki_sum += ki*error;
    ki_sum = CLAMP(ki_sum,-ki_limit,ki_limit);
    command = (__builtin_mulss(-kp,error)>>(6+kp_shift)) - (ki_sum>>(6+ki_shift));

    return (command);
}

