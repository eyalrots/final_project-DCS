#include "../header/api.h"
#include <stdint.h>

extern volatile FSM_state_t state;
extern volatile SYS_mode_t lpm_mode;

extern volatile circular_buffer_t transmit_buffer;
extern volatile uint8_t adc_buffer[];

volatile unsigned int echo_rising_edge, echo_falling_edge;
volatile uint8_t requested_angle;
volatile uint8_t received_new_anlge_flag = 0;
volatile uint16_t current_distance;

volatile uint16_t files[10];

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

void get_distance_from_ldrs(uint16_t *ldr1_distance, uint16_t *ldr2_distance) {
    // *ldr1_distance = adc_buffer[0] * 330 / 1023;
    // *ldr2_distance = adc_buffer[3] * 330 / 1023;

    ADCconfigLDR1();
    enable_ADC();
    enterLPM(mode0);
    disable_ADC();
    *ldr1_distance = sample_ADC();

    ADCconfigLDR2();
    enable_ADC();
    enterLPM(mode0);
    disable_ADC();
    *ldr2_distance = sample_ADC();
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
    
void wait_for_motor() {
    timer0_start_delay(0xffff);
    timer0_start_delay(0xffff);
    timer0_start_delay(0xffff);
    timer0_start_delay(0xffff);
    timer0_start_delay(0xffff);
    timer0_start_delay(0xffff);
}
    
void calculate_distance_ldr(uint16_t* distance) {
    uint16_t ldr1_dist = 0;
    uint16_t ldr2_dist = 0;
    /* get distance from LDRs */
    get_distance_from_ldrs(&ldr1_dist, &ldr2_dist);
    // print_num(ldr1_dist, 8, 4, 0x30);
    // print_num(ldr2_dist, 16, 4, 0x30);
    *distance = (ldr1_dist + ldr2_dist) / 2;
}

void go_to_zero() {
    turn_on_pwm();
    move_motor_to_new_angle(0);
    wait_for_motor();
}

void scan_with_motor(uint8_t type) {
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
        if (type == 0) {
            get_distance_from_sensor(&current_distance_cm);
        } else if (type == 1) {
            calculate_distance_ldr(&current_distance_cm);
        }
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
            transmit_buffer.size = 0;
        }
        // set next angle = current angle + step
        current_angle += SCAN_STEP;
    }
    /* Clean what remains in the buffer */
    if (transmit_buffer.size > 0) {
        uart_write((uint8_t*)&transmit_buffer.buffer[transmit_buffer.read], transmit_buffer.size*sizeof(distance_sample_t));
        transmit_buffer.read = transmit_buffer.write;
        transmit_buffer.size = 0;
    }

    while(state==state1) continue;
}

void scan_with_sonic() {
    scan_with_motor(0);
}

void scan_with_ldr() {
    scan_with_motor(1);
}

void scan_at_given_angle() {
    uint16_t current_distance_cm = 0;
    distance_sample_t current_sample;
    uint8_t flag = 0;
    
    lcd_clear();
    lcd_puts("dist:000; at:000");
    go_to_zero();
    
    while (state == state2) {
        /* Move the motor to the requested angle by user */
        if (received_new_anlge_flag) {
            connect_to_pwm();
            move_motor_to_new_angle(requested_angle);
            print_num(requested_angle, 16, 3, 0x30);
            wait_for_motor();
            disconnect_from_pwm();
            current_sample.angle = requested_angle;
            received_new_anlge_flag = 0;
            flag = 1;
        }
        if (flag) {
            /* Get distance from sensor */
            get_distance_from_sensor(&current_distance_cm);
            /* Print distance on LCD */
            print_num(current_distance_cm, 8, 3, 0x30);
            /* Transmit distance to user */
            current_sample.distance_cm = current_distance_cm;
            // insert sample to transmit buffer
            memcpy((uint8_t*)&transmit_buffer.buffer[transmit_buffer.write], (uint8_t*)&current_sample, sizeof(current_sample));
            transmit_buffer.write++;
            transmit_buffer.write %= BUFFER_LEN;
            transmit_buffer.size++;
            // transmit
            if (transmit_buffer.size == BUFFER_LEN) {
                uart_write((uint8_t*)&transmit_buffer.buffer[transmit_buffer.read], BUFFER_LEN*sizeof(distance_sample_t));
                transmit_buffer.read = transmit_buffer.write;
                transmit_buffer.size = 0;
                return;
            }
            // uart_write((uint8_t*)&current_distance_cm, sizeof(uint16_t));
        }
    }
}

void counting() {
    file_header_t* header;
    uint16_t file_addr;
    uint16_t i = 0;
    lcd_clear();
    lcd_puts("reading...");
    while (state==state4) {
        enterLPM(mode0);
        header = (file_header_t*)(SEG_D);
        file_addr = header->address;
        for (i = 0; i < header->size; i++) {
            lcd_data(*(uint8_t*)file_addr);
        }
    }
}
