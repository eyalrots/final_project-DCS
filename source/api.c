#include "../header/api.h"
#include <stdint.h>
#include <stdlib.h>

extern volatile FSM_state_t state;
extern volatile SYS_mode_t lpm_mode;

extern volatile circular_buffer_t transmit_buffer;
extern volatile uint8_t adc_buffer[];
extern volatile uint8_t cur_char;

extern volatile uint16_t available_space;
extern volatile uint16_t num_of_files;
extern volatile uint16_t cur_header;
extern volatile uint16_t file_location;
extern volatile uint8_t pb_pressed;
extern volatile uint8_t is_script;

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
    timer0_start_delay(0xffff, 0);
    timer0_start_delay(0xffff, 0);
    timer0_start_delay(0xffff, 0);
    timer0_start_delay(0xffff, 0);
    timer0_start_delay(0xffff, 0);
    timer0_start_delay(0xffff, 0);
}
    
void calculate_distance_ldr(uint16_t* distance) {
    uint16_t ldr1_dist = 0;
    uint16_t ldr2_dist = 0;
    uint16_t ldr1_cm;
    uint16_t ldr2_cm;
    /* get distance from LDRs */
    get_distance_from_ldrs(&ldr1_dist, &ldr2_dist);
    /* get cm from calibration data */
    *distance = (ldr1_dist + ldr2_dist) / 2;
}

void go_to_zero() {
    turn_on_pwm();
    move_motor_to_new_angle(0);
    wait_for_motor();
}

void scan_with_motor(uint8_t type, uint8_t start, uint8_t end) {
    /*
        Moves with steps of pre-defined size (in header file).
        Between each angle there is a delay - 
            this delay if to prevent colitions between capture and compare.
        When the buffer fills up, a transmition occurres.
    */
    uint8_t current_angle = start;
    uint16_t current_distance_cm = 0;
    distance_sample_t current_sample;
    uint16_t bytes_transmitted = 0;

    // init lcd
    lcd_clear();
    lcd_puts("dist:000; at:000");
    // start at angle 0
    go_to_zero();

    while (current_angle <= end) {
        // move motor to the next angle
        move_motor_to_new_angle(current_angle);
        /* Turn off PWM */
        disconnect_from_pwm();
        // wait for a bit, let everythong settle down...
        timer0_start_delay(0x4000, 0);
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
    disconnect_from_pwm();
    turn_off_pwm();
}

void scan_with_sonic() {
    scan_with_motor(0, 0, 180);
}

void scan_with_ldr() {
    scan_with_motor(1, 0, 180);
}

void scan_at_given_angle() {
    uint16_t current_distance_cm = 0;
    distance_sample_t current_sample;
    uint8_t flag = 0;
    
    lcd_clear();
    lcd_puts("dist:000; at:000");
    go_to_zero();
    
    while (state == state2 || state==state4) {
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
            // if (s_type==0)
                get_distance_from_sensor(&current_distance_cm);
            // else
                // calculate_distance_ldr(&current_distance_cm);
            /* Print distance on LCD */
            print_num(current_distance_cm, 8, 3, 0x30);
            /* Transmit distance to user */
            current_sample.distance_cm = current_distance_cm;
            /* sent distance over uart */
            uart_write((uint8_t*)&current_distance_cm, 2);
            /* check end if state */
            if (pb_pressed==1) {
                pb_pressed = 0;
                lcd_clear();
                return;
            }
        }
    }
}

void erase_info() {
    uint8_t* temp_ptr;
    uint8_t temp_buffer[40];
    uint8_t i = 0;

    lcd_puts("erasing segs...");
    /* save important data form SEG B */
    temp_ptr = (uint8_t*)LDR_CALIB;
    for (i = 0; i < 40; i++) {
        temp_buffer[i] = *temp_ptr++;
    }
    erase_seg((uint8_t*)SEG_B);
    erase_seg((uint8_t*)SEG_C);
    erase_seg((uint8_t*)SEG_D);
    temp_ptr = (uint8_t*)LDR_CALIB;
    open_flash();
    /* rewrite data to segment B */
    for (i = 0; i < 40; i++) {
        *temp_ptr++ = temp_buffer[i];
        while(FCTL3 & BUSY);
    }
    close_flash();
    available_space = FILE_MEM_SIZE;
    num_of_files = 0;
    file_location = SEG_4;
    cur_header = SEG_D;
    while (state==state7);
}

void inc_lcd(uint8_t range, uint8_t delay){
    uint8_t i = 0;
    lcd_clear();
    lcd_puts("inc: 000");
    while (range--) {
        print_num(i++,8,3,0x30);
        timer0_start_delay(delay*10, 1);
    }    
}

void dec_lcd(uint8_t range, uint8_t delay){
    lcd_clear();
    lcd_puts("dec: 000");
    while (range--) {
        print_num(range,8,3,0x30);
        timer0_start_delay(delay*10, 1);
    }
}

void rra_lcd(uint8_t ch, uint8_t delay) {
    lcd_clear();
    int start = 0, temp = 0;
    int flag = 0;
    int counter = 0;

    while(counter < 33) {
        lcd_init();
        lcd_clear();
        flag = start >= 16 ? 1 : 0;
        if (flag) {
            lcd_new_line;
            temp = start-16;
        }
        else {
            lcd_home();
            temp = start;
        }
        print_b(ch, temp);
        start = (start+1)&31;
        timer0_start_delay(delay*10, 1);
        counter++;
    }
    lcd_clear();
}

void script_mode() {
    uint8_t* file_ptr;
    uint16_t file_size;
    file_header_t* file_header;
    uint8_t command;
    uint8_t operand_1;
    uint8_t operand_2;
    uint8_t delay = 48;
    if (is_script) {
        /* get basic file information */
        file_header = (file_header_t*)(cur_header - sizeof(file_header_t));
        file_size = file_header->size;
        file_ptr = (uint8_t*)(file_header->address);
        /* execute ISA in file */
        while (file_size) {
            file_size--;
            command = *(file_ptr++);
            switch (command) {
                case 0x01: // inc_lcd
                    operand_1 = *(file_ptr++);
                    file_size--;
                    inc_lcd(operand_1, delay);   
                    break;
                case 0x02: // dec_lcd
                    operand_1 = *(file_ptr++);
                    file_size--;
                    dec_lcd(operand_1, delay);
                    break; 
                case 0x03: // rra_lcd
                    operand_1 = *(file_ptr++);
                    file_size--;
                    rra_lcd(operand_1, delay);
                    break;
                case 0x04: // set_delay
                    operand_1 = *(file_ptr++);
                    file_size--;   
                    delay = operand_1 >= 48 ? 48 : operand_1;
                    break;
                case 0x05: // clear_lcd
                    lcd_clear();
                    break;
                case 0x06: // servo_deg
                    operand_1 = *(file_ptr++);
                    file_size--;
                    requested_angle = operand_1;
                    received_new_anlge_flag = 1;
                    while (!(IFG2 & UCA0TXIFG));
                    UCA0TXBUF = 0x02;
                    scan_at_given_angle(0);   
                    break;
                case 0x07: // servo_scan
                    operand_1 = *(file_ptr++);
                    file_size--;
                    operand_2 = *(file_ptr++);
                    file_size--;
                    while (!(IFG2 & UCA0TXIFG));
                    UCA0TXBUF = 0x01;
                    while (!(IFG2 & UCA0TXIFG));
                    UCA0TXBUF = operand_1;
                    while (!(IFG2 & UCA0TXIFG));
                    UCA0TXBUF = operand_2;
                    lcd_puts("servo scan...");
                    scan_with_motor(0, operand_1, operand_2);
                    break;    
                case 0x08: // sleep
                    is_script = 0;
                    while (!(IFG2 & UCA0TXIFG));
                    UCA0TXBUF = 0x03;
                    state = state0;
            }
        }
        is_script = 0;
    }
}

file_header_t* find_next_text_header(uint16_t* addr, uint16_t start_addr,uint8_t start_index){
    uint8_t index = start_index;
    file_header_t* header = (file_header_t*)start_addr;
    *addr = start_addr;
    while (header->type)
    {
        if (index == num_of_files) {
            header = (file_header_t*)SEG_D;
            index = 0;
        }
        else {
            header++;
            *addr += sizeof(file_header_t);
            index++;
        }
        if (index == start_index) {
            return NULL;
        }
    }
    return header;
}

void read_file(uint16_t adder,uint16_t size){
    uint16_t index = adder;
    uint16_t stop_adder  = adder + size;
    unsigned char ch;
    uint16_t j = 0;
    lcd_clear();
    lcd_puts("start reading...");
    while (state == state5) {
        enterLPM(mode0);
        if (pb_pressed==1) {   
            lcd_clear();
            for(j = 0; j < 32; j++) {
                ch = *((uint8_t*)index++);
                if (j == 17) {
                    lcd_new_line;
                }
                if (index <= stop_adder) {
                    lcd_data(ch);
                }
            }
            index -= 16;
            if (index >= stop_adder) {
                index = adder;
            }
        }
        if (pb_pressed==2) {
            return;
        }
        
    }
    
    
}

void file_mode(){ // i asume fill mode is in state6 
    uint16_t addr = SEG_D;
    uint16_t size;
    uint8_t index;
    file_header_t* f_header;
    file_header_t* f_next_header;

    f_header = find_next_text_header(&addr, SEG_D,0);
    if (!f_header)
    {   
        lcd_clear();
        lcd_puts("0 text files");
        return;
    }
    index = (addr - SEG_D) / sizeof(file_header_t);

    f_next_header = index+1==num_of_files ? find_next_text_header(&addr, SEG_D,0) :
                                            find_next_text_header(&addr, addr+sizeof(file_header_t),++index);
    if (!f_next_header)
    {
        f_next_header = f_header;
    }
    while (state == state5){
        lcd_puts((const char*)f_header->name);
        lcd_new_line;
        if (f_next_header == f_header) {
            lcd_puts("only 1 file");
        }
        else{
            lcd_puts((const char*)f_next_header->name);
        }
        enterLPM(mode0);
        /* button 0 */
        if (pb_pressed==1) {
            f_header = f_next_header;
            
            index = (addr - SEG_D) / sizeof(file_header_t);
            f_next_header = index+1==num_of_files ? find_next_text_header(&addr, SEG_D,0) :
                                                    find_next_text_header(&addr, addr+sizeof(file_header_t),++index);

            pb_pressed = 0;
        } 
        /* button 1 */
        else if (pb_pressed==2) {
            read_file(f_header->address,f_header->size);
            pb_pressed = 0;
        }
        lcd_clear();
    }
    
}

void calibrate_ldr() {
    uint16_t* flash_ptr = NULL;
    uint8_t samples_saved = 0;
    uint16_t distace = 0;
    uint16_t samples[20] = {632, 374, 441, 455, 444, 487, 451, 531, 534, 553,
                            70, 324, 399, 448, 458, 497, 470, 455, 408, 496};
    /* 1-> LDR1 :: 2-> LDR2 */
    flash_ptr = (uint16_t*)LDR_CALIB;
    /* init segment for new calibration */
    erase_seg((uint8_t*)SEG_B);
    available_space = FILE_MEM_SIZE;
    num_of_files = 0;
    /* save samples of calibration */
    while (samples_saved < 20) {
        // /* mesure LDR value */
        // if (samples_saved < 10)
        //     ADCconfigLDR1();
        // else
        //     ADCconfigLDR2();
        // enable_ADC();
        // enterLPM(mode0);
        // disable_ADC();
        // distace = sample_ADC();
        // enterLPM(mode0);
        // /* save value in memort when ready */
        // if (pb_pressed==1) {
            open_flash();
            *flash_ptr++ = samples[samples_saved];
            close_flash();
            samples_saved++;
            // pb_pressed = 0;
            print_num(distace, 16, 5, 0x30);
            print_num(samples_saved, 3, 3, 0x30);
        // }
    }
    while (state==state7);
}

void read_calibration() {
    uint8_t i;
    uint16_t* flash_ptr = (uint8_t*)LDR_CALIB;
    for (i = 0; i < 40; i++) {
        print_num(*flash_ptr++, 16, 3,  0x30);
        print_num(i, 3, 2, 0x30);
        enterLPM(mode0);
    }
}
