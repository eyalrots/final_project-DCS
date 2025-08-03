#include "../header/api.h"

extern volatile FSM_state_t state;
extern volatile SYS_mode_t lpm_mode;

volatile unsigned int echo;

void move_motor_to_new_angle(unsigned int new_angle) {
    /* 
        Moving the motor is done by setting a new angle -
            using a new duty cycle value.
        The motor operates at 40Hz.
    */
    unsigned int duty_cycle = 0;
    unsigned int on_time_us = 0;
    unsigned int in_range_angle = 0;

    // set angle to range [0,180]
    if (new_angle < 0) {
        return;
    }
    in_range_angle %= 180;
    // set on time -> 0.6ms + 1.9ms*(angle/180)
    // 0 degrees = 0.6ms ; range = 1.9ms ; relative shift for angle = new_angle/180
    on_time_us = 600 + ((1900 * new_angle) / 180);

    generate_pwm_wave_at_ton_freq(1, on_time_us, 40);
}

void scan_with_motor() {
    /*
        Moves with steps of pre-defined size (in header file).
        When the motor arrives to a sample point - the PWM shuts off.
    */
}