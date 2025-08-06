#include "../header/api.h"
#include <stdint.h>

extern volatile FSM_state_t state;
extern volatile SYS_mode_t lpm_mode;

extern volatile circular_buffer_t transmit_buffer;

volatile unsigned int echo_rising_edge, echo_falling_edge;
volatile uint8_t requested_angle;

// unsigned int distance_samples[NUM_OF_SAMPLES];

void init_sample_array(unsigned int *array, unsigned int size) {
    unsigned int i = 0;

    for (i = 0; i < size; i++) {
        array[i] = 0;
    }
}

void get_distance_from_sensor(uint16_t *distance_cm) {
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

void move_motor_to_new_angle(uint8_t new_angle) {
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
        Between each angle there is a delay - 
            this delay if to prevent colitions between capture and compare.
        When the buffer fills up, a transmition occurres.
    */
    uint8_t current_angle = 0;
    uint16_t current_distance_cm = 0;
    distance_sample_t current_sample;
    uint16_t bytes_transmitted = 0;

    // init lcd
    lcd_clear();
    lcd_puts("dist:000; at:000");
    // start at angle 0
    go_to_zero();

    while (current_angle <= 180) {
        // move motor to the next angle
        move_motor_to_new_angle(current_angle);
        /* Turn off PWM */
        disconnect_from_pwm();
        // wait for a bit, let everythong settle down...
        timer0_start_delay(0x4000);
        // get distance
        get_distance_from_sensor(&current_distance_cm);
        /* Turn on PWM */
        connect_to_pwm();
        turn_on_pwm();
        // print data on lcd
        print_num(current_distance_cm, 8, 3, 0x30);
        print_num(current_angle, 16, 3, 0x30);
        // create sample object
        current_sample.distance_cm = current_distance_cm;
        current_sample.angle = current_angle;
        // insert sample to transmit buffer
        memcpy((uint8_t*)&transmit_buffer.buffer[transmit_buffer.write], (uint8_t*)&current_sample, sizeof(current_sample));
        transmit_buffer.write++;
        transmit_buffer.write %= BUFFER_LEN;
        transmit_buffer.size++;
        // transmit
        if (transmit_buffer.size == BUFFER_LEN) {
            uart_write((uint8_t*)&transmit_buffer.buffer[transmit_buffer.read], BUFFER_LEN*sizeof(distance_sample_t));
            transmit_buffer.read = transmit_buffer.write;
            bytes_transmitted += transmit_buffer.size * sizeof(distance_sample_t);
            transmit_buffer.size = 0;
        }
        // set next angle = current angle + step
        current_angle += SCAN_STEP;
    }
    /* Clean what remains in the buffer */
    if (transmit_buffer.size > 0) {
        uart_write((uint8_t*)&transmit_buffer.buffer[transmit_buffer.read], transmit_buffer.size*sizeof(distance_sample_t));
        transmit_buffer.read = transmit_buffer.write;
        bytes_transmitted += transmit_buffer.size * sizeof(distance_sample_t);
        print_num(bytes_transmitted, 8, 3, 0x30);
        transmit_buffer.size = 0;
    }

    while(state==state1) continue;
}

void scan_at_given_angle() {
    uint16_t current_distance_cm = 0;
    
    lcd_clear();
    lcd_puts("dist:000; at:000");
    print_num(requested_angle, 16, 3, 0x30);
    go_to_zero();

    while (requested_angle == -1) continue;
    /* Move the motor to the requested angle by user */
    move_motor_to_new_angle(requested_angle);
    disconnect_from_pwm();
    while (state == state2) {
        /* Get distance from sensor */
        get_distance_from_sensor(&current_distance_cm);
        /* Print distance on LCD */
        print_num(current_distance_cm, 8, 3, 0x30);
        /* Transmit distance to user */
        uart_write((uint8_t*)&current_distance_cm, sizeof(uint16_t));
    }
    requested_angle = -1;
    connect_to_pwm();
}

void counting() {
    uint16_t test_num = 0x1234;
    //uint8_t test = (uint8_t)(test_num >> 8);
    uint8_t test = (uint8_t)(test_num >> 8);
    print_num(test, 3, 3, 0x30);
}
