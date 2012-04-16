

#include "setup.h"



static enum dsp_state dsp = DSP_OFF;

int pwm_desired;

void step_state()
{
    static int count = 0;

    if (count<1000) {
        count++;
        dsp = DSP_STARTUP;
    } else {
        switch (ec_cmd.command[0].mode) {
            case 0:
                dsp = DSP_OFF;
                break;
            case 1:
                dsp = DSP_PWM;
                break;
            case 2:
                dsp = DSP_OFF;
                break;
            case 3:
                dsp = DSP_CURRENT;
                break;
            case 4:
            default:
                dsp = DSP_BRAKE;
                break;
        }
    }

    switch (dsp) {
        case DSP_OFF:
            set_current_command_ma(0);
            pwm_desired = 0;
            set_bldc_open();
            break;
        case DSP_STARTUP:
            set_current_command_ma(0);
            pwm_desired = 0;
            set_bldc_open();
            set_adc_zeros();
            break;
        case DSP_PWM:
            set_current_command_ma(0);
            pwm_desired = ec_cmd.command[0].pwm_desired;
            set_bldc_commutation();
            break;
        case DSP_CURRENT:
            set_current_command_ma(ec_cmd.command[0].current_desired);
            pwm_desired = 0;
            set_bldc_commutation();
            break;
        case DSP_BRAKE:
        default:
            set_current_command_ma(0);
            pwm_desired = 0;
            set_bldc_brake();
            break;
    }
}

enum dsp_state get_dsp_state()
{
    return dsp;
}

