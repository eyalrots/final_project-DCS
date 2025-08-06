#include "../header/api.h"
#include <stdint.h>

extern volatile FSM_state_t state;
extern volatile SYS_mode_t lpm_mode;

extern volatile circular_buffer_t transmit_buffer;

volatile unsigned int echo_rising_edge, echo_falling_edge;

// unsigned int distance_samples[NUM_OF_SAMPLES];

void init_sample_array(unsigned int *array, unsigned int size) {
    unsigned int i = 0;

    for (i = 0; i < size; i++) {
        array[i] = 0;
    }
}

void get_distance_from_sensor(uint32_t *distance_cm) {
    /*
        The distance is calculated by the difference between the
            rising and falling edge of the echo signal.
        The difference is the HIGH time of the signal.
    */
    uint32_t echo_high_time = 0;

    
    generate_trigger_for_distance_sensor();
    timer1_A2_start_capture();
    // Timer clk = ~1M -> clk count ~= time in [us].
    echo_high_time = echo_falling_edge - echo_rising_edge;
    // calculate distance ; speed of sound = ~34645
    *distance_cm = (echo_high_time * 173) / 10000;
}

void move_motor_to_new_angle(unsigned int new_angle) {
    /* 
        Moving the motor is done by setting a new angle -
            using a new duty cycle value.
        The motor operates at 40Hz.
    */
    unsigned int on_time_us = 0;
    /* 
        Set on time -> 0.6ms + 1.9ms*(angle/180)
        0 degrees = 0.6ms ; range = 1.9ms
            relative shift for angle = new_angle/180
    */
    on_time_us = 600 + ((190 * new_angle) / 18);
    generate_pwm_wave_with_Ton_at_freq(on_time_us, 40);
}

void go_to_zero() {
    turn_on_pwm();
    move_motor_to_new_angle(0);
    timer0_start_delay(0xffff);
    timer0_start_delay(0xffff);
    timer0_start_delay(0xffff);
    timer0_start_delay(0xffff);
    timer0_start_delay(0xffff);
    timer0_start_delay(0xffff);
}

void print_array(unsigned int *array, unsigned int size) {
    unsigned int i = 0;

    for (i = 0; i < size; i++) {
        print_num(array[i], 8, 3, 0x30);
        timer0_start_delay(0xffff);
        timer0_start_delay(0xffff);
        timer0_start_delay(0xffff);
        timer0_start_delay(0xffff);
        timer0_start_delay(0xffff);
        timer0_start_delay(0xffff);
    }
}

void scan_with_motor() {
    /*
        Moves with steps of pre-defined size (in header file).
        When the motor arrives to a sample point - the PWM shuts off.
    */
    unsigned int current_angle = 0;
    unsigned int current_distance_cm = 0;
    distance_sample_t current_sample;

    // init lcd
    lcd_clear();
    lcd_puts("dist:000; at:000");
    // start at angle 0
    go_to_zero();
    // init array
//    init_sample_array(distance_samples, NUM_OF_SAMPLES);

    while (current_angle <= 180) {
        // move motor to the next angle
        move_motor_to_new_angle(current_angle);
        // wait 4ms = 4000us - adjusted for actual frequency -> 4834.
        timer0_start_delay(0xf000);
        // get distance
        get_distance_from_sensor(&current_distance_cm);
        // Turn on PWM
        turn_on_pwm();
        // print data on lcd
        print_num(current_distance_cm, 8, 3, 0x30);
        print_num(current_angle, 16, 3, 0x30);
        // create sample object
        current_sample.distance_cm = current_distance_cm;
        current_sample.angle = current_angle;
        // insert sample to transmit buffer
        transmit_buffer.buffer[transmit_buffer.write] = current_sample;
        transmit_buffer.write += sizeof(distance_sample_t);
        transmit_buffer.write %= BUFFER_SIZE;
        transmit_buffer.size++;
        // set next angle = current angle + step
        current_angle += SCAN_STEP;
    }
}

void counting() {
    uint16_t test_num = 0x1234;
    //uint8_t test = (uint8_t)(test_num >> 8);
    uint8_t test = (uint8_t)(test_num >> 8);
    print_num(test, 3, 3, 0x30);
}
