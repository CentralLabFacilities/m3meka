

#include "setup.h"



static enum dsp_state dsp = DSP_OFF;
int trace_temperature_max = 10000;  // centikelvin

void step_state()
{
    static int count = 0;
    static int temperature_count = 0;

    if (temperature_count++%20 == 0)
        step_temperature_model(get_current_ma());

    if (get_model_temperature_cK() > trace_temperature_max)
        dsp = DSP_ERROR;

    if (count<1000) {
        count++;
        dsp = DSP_STARTUP;
    } else if (dsp != DSP_ERROR) { // latch in the error mode
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
            set_pwm_desired(0);
            set_bldc_open();
            break;
        case DSP_STARTUP:
            set_current_command_ma(0);
            set_pwm_desired(0);
            set_bldc_open();
            set_adc_zeros();
            break;
        case DSP_PWM:
            set_current_command_ma(0);
            set_pwm_desired(ec_cmd.command[0].pwm_desired);
            set_bldc_commutation();
            break;
        case DSP_CURRENT:
            set_current_command_ma(ec_cmd.command[0].current_desired);
            set_pwm_desired(0);
            set_bldc_commutation();
            break;
        case DSP_BRAKE:
            set_current_command_ma(0);
            set_pwm_desired(0);
            set_bldc_brake();
            break;
        case DSP_ERROR:
        default:
            set_current_command_ma(0);
            set_pwm_desired(0);
            set_bldc_open();
            break;
    }
}

enum dsp_state get_dsp_state()
{
    return dsp;
}

