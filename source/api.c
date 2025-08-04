#include "../header/api.h"

extern volatile FSM_state_t state;
extern volatile SYS_mode_t lpm_mode;

volatile unsigned int echo_rising_edge, echo_falling_edge;

void get_distance_from_sensor(unsigned int *distance_cm) {
    /*
        The distance is calculated by the difference between the
            rising and falling edge of the echo signal.
        The difference is the HIGH time of the signal.
    */
    unsigned int echo_high_time = 0;

    generate_trigger_for_distance_sensor();
    timer1_A2_start_capture();
    // Timer clk = ~1M -> clk count ~= time in [us].
    echo_high_time = echo_falling_edge - echo_rising_edge;
    // calculate distance ; speed of sound = ~34645
    *distance_cm = echo_high_time * 17322;
}

void move_motor_to_new_angle(unsigned int new_angle) {
    /* 
        Moving the motor is done by setting a new angle -
            using a new duty cycle value.
        The motor operates at 40Hz.
    */
    unsigned int on_time_us = 0;
    unsigned int in_range_angle = 0;

    // set angle to range [0,180]
    if (new_angle < 0) {
        return;
    }
    in_range_angle %= 180;
    /* 
        Set on time -> 0.6ms + 1.9ms*(angle/180)
        0 degrees = 0.6ms ; range = 1.9ms
            relative shift for angle = new_angle/180
    */
    on_time_us = 600 + ((1900 * new_angle) / 180);

    generate_pwm_wave_with_Ton_at_freq(on_time_us, 40);
}

void scan_with_motor() {
    /*
        Moves with steps of pre-defined size (in header file).
        When the motor arrives to a sample point - the PWM shuts off.
    */
    unsigned int current_angle = 0;
    unsigned int current_distance_cm = 0;

    // init lcd
    lcd_clear();
    lcd_puts("dist:000; at:000");
    while (state==state1) {
        // move motor to the next angle
        move_motor_to_new_angle(current_angle);
        // wait 4ms = 4000us - adjusted for actual frequency -> 4834.
        timer0_start_delay(4384);
        // turn of PWM
        turn_off_pwm();
        // get distance
        get_distance_from_sensor(&current_distance_cm);
        // print data on lcd
        print_num(current_distance_cm, 5, 3, 0x30);
        print_num(current_angle, 14, 3, 0x30);
        // set next angle = current angle + step
        current_angle += SCAN_STEP;
        // make sure angle in range [0,180]
        current_angle = current_angle>180? current_angle-180 : current_angle;
    }
}