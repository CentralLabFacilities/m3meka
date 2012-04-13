

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

enum dsp_state {
    DSP_OFF,
    DSP_STARTUP,
    DSP_PWM,
    DSP_CURRENT,
    DSP_BRAKE
};

extern int current_desired;
extern int pwm_desired;

void step_state();
enum dsp_state get_dsp_state();

#endif
