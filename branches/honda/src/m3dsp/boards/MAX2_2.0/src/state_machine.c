

#include "setup.h"

enum dsp_state {
    DSP_OFF,
    DSP_STARTUP,
    DSP_PWM,
    DSP_CURRENT,
    DSP_BRAKE
};

int current_desired;
int pwm_desired;

void step_state()
{
    static enum dsp_state dsp = DSP_OFF;
    static int count = 0;

    if (count<10000) {
        count++;
        dsp = DSP_STARTUP;
    }

    switch (dsp) {
        case DSP_OFF:
            current_desired = 0;
            pwm_desired = 0;
            break;
        case DSP_STARTUP:
            current_desired = 0;
            pwm_desired = 0;
            break;
        case DSP_PWM:
            current_desired = 0;
            pwm_desired = ec_cmd.command[0].pwm_desired;
            break;
        case DSP_CURRENT:
            current_desired = ec_cmd.command[0].current_desired;
            pwm_desired = 0;
            break;
        case DSP_BRAKE:
        default:
            current_desired = 0;
            pwm_desired = 0;
            break;
    }
}

