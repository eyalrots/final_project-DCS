#include "../header/halGPIO.h"
#include <stdint.h>

extern volatile FSM_state_t state;
extern volatile SYS_mode_t lpm_mode;

extern volatile unsigned int echo_rising_edge, echo_falling_edge;
extern volatile circular_buffer_t transmit_buffer;
extern volatile uint8_t requested_angle;

volatile unsigned int count = 0x0;

// System configuration
void system_config() {
    __GPIO_config();
   __timerA0_delay_config();
    __timer1_pwm_config();
    __timer1_A2_capture_config();
    __adc_config();
    __UART_config();
    lcd_init();

    // __timer0_A0_config();
}

// Polling cased delay functions
void delay(unsigned int t) {
    for (;t > 0; t--);
}
// Delay usec functions
void DelayUs(unsigned int cnt){
    unsigned char i;
    for(i=cnt ; i>0 ; i--) asm("nop"); // the command asm("nop") takes about 1usec
}
// Delay msec functions
void DelayMs(unsigned int cnt){
    unsigned char i;
    for(i=cnt ; i>0 ; i--) DelayUs(1000);
}

// Enter LMP0 mode
void enterLPM(unsigned char LPM_level){
    if (LPM_level == 0x00)
      _BIS_SR(LPM0_bits);     /* Enter Low Power Mode 0 */
    else if(LPM_level == 0x01)
      _BIS_SR(LPM1_bits);     /* Enter Low Power Mode 1 */
    else if(LPM_level == 0x02)
      _BIS_SR(LPM2_bits);     /* Enter Low Power Mode 2 */
    else if(LPM_level == 0x03)
      _BIS_SR(LPM3_bits);     /* Enter Low Power Mode 3 */
    else if(LPM_level == 0x04)
      _BIS_SR(LPM4_bits);     /* Enter Low Power Mode 4 */
}

// enable / disable interrupts
void enable_interrupts() {
    _BIS_SR(GIE);
}
void disable_interrupts() {
    _BIC_SR(GIE);
}

//-------------------------------------------------------------
//                          LCD
//-------------------------------------------------------------
// send command to LCD
void lcd_cmd(unsigned char c) {
    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    if (LCD_MODE == FOURBIT_MODE)
    {
        LCD_DATA_WRITE &= ~OUTPUT_DATA;// clear bits before new write
        LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
        lcd_strobe();
    }
    else
    {
        LCD_DATA_WRITE = c;
        lcd_strobe();
    }
}

// send data to LCD
void lcd_data(unsigned char c) {
    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_RS(1);
    if (LCD_MODE == FOURBIT_MODE)
    {
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
            LCD_DATA_WRITE &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> (8 - LCD_DATA_OFFSET));
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
    }
    else
    {
            LCD_DATA_WRITE = c;
            lcd_strobe();
    }

    LCD_RS(0);
}

// write string to LCD
void lcd_puts(const char * s) {
    while (*s)
        lcd_data(*s++);
}

// initialize LCD
void lcd_init() {
    char init_value;

    if (LCD_MODE == FOURBIT_MODE) init_value = 0x03 << LCD_DATA_OFFSET;
    else init_value = 0x3F;

    LCD_RS_DIR(OUTPUT_PIN);
    LCD_EN_DIR(OUTPUT_PIN);
    LCD_RW_DIR(OUTPUT_PIN);
    LCD_DATA_DIR |= OUTPUT_DATA;
    LCD_RS(0);
    LCD_EN(0);
    LCD_RW(0);

    // sanity check - need mental health hospital?
    P2DIR |= 0xe0;
    P2OUT |= 0xe0;
    P2OUT &= ~0xe0;

    DelayMs(15);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayMs(5);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayUs(200);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();

    if (LCD_MODE == FOURBIT_MODE){
        LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
        lcd_strobe();
        lcd_cmd(0x28); // Function Set
    }
    else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots

    lcd_cmd(0xF); //Display On, Cursor On, Cursor Blink
    lcd_cmd(0x1); //Display Clear
    lcd_cmd(0x6); //Entry Mode
    lcd_cmd(0x80); //Initialize DDRAM address to zero
}

// LCD strobe function
void lcd_strobe() {
    LCD_EN(1);
    asm("NOP");
    LCD_EN(0);
}

// Complex print functions for LCD
// print letter (b) at location (start) on LCD
void print_b(char b, int start) {
    for (;start > 0; start--) {
        lcd_cursor_right();
    }

    lcd_data(b);
    cursor_off;
}
// print a number (num) of length (len) at location (start) with a fill character (fill)
void print_num(uint16_t num, int start, int len, char fill) {
    lcd_home();
    unsigned int i;
    char digit;
    for (i=start; i>0; i--) lcd_cursor_right();
    i=0;
    while (num != 0 && i < len) {
        digit  = num % 10 + 0x30;
        num /= 10;
        lcd_cursor_left();
        lcd_data(digit);
        lcd_cursor_left();
        i++;
    }

    while (i < len) {
        lcd_cursor_left();
        lcd_data(fill);
        lcd_cursor_left();
        i++;
    }
}
//-------------------------------------------------------------

//-------------------------------------------------------------
//                          TIMERS
//-------------------------------------------------------------
// Timer A1
void set_TA1CCR0(unsigned int new_value) {
    TA1CCR0 = new_value;
}
void set_TA1CCR1(unsigned int new_value) {
    TA1CCR1 = new_value;
}
void turn_off_pwm() {
    // stop timer
    TA1CTL &= ~MC_1;
    TA1CTL |= TACLR;
}
void turn_on_pwm() {
    // enable timer at up-mode
    TA1CTL = TASSEL_2 + MC_1;
}
// Timer A0
void timer0_start_delay(unsigned int time_us) {
    /*
        Timer0 A0 used for delay.
        Input: time for delay in us.
        Since timer clk is ~1M -> register value ~= time_us.
    */
    unsigned int in_range_time_us = 0;

    // set time to range [0,485]ms
    in_range_time_us = time_us>=485001? 485000 : time_us;
    // set register to correct delay
    TA0CCR0 = in_range_time_us;
    TA0CTL = TASSEL_2 + MC_1;
    __bis_SR_register(LPM0_bits + GIE);
}
//-------------------------------------------------------------

//-------------------------------------------------------------
//                          ADC
//-------------------------------------------------------------

//-------------------------------------------------------------

//-------------------------------------------------------------
//                          UART
//-------------------------------------------------------------
void uart_write(uint8_t *buffer, uint16_t size) {
    uint16_t i;
    
    if (!buffer) return;

    for (i = 0; i < size; i++) {
        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = buffer[i];
    }
}
//-------------------------------------------------------------

//-------------------------------------------------------------
//                  General Functions
//-------------------------------------------------------------
void generate_pwm_wave_with_Ton_at_freq(unsigned int on_time, unsigned int freq) {
    // T_on received in us
    unsigned int pwm_period = 0;

    // Turn on timer
    turn_on_pwm();
    // on_time is in [us] and SMCLK freq = ~1MHz => register value ~= on_time.
    // set PWM period
    pwm_period = SMCLK / freq;
    // set timer values
    // Timer A1
    set_TA1CCR0(pwm_period);
    set_TA1CCR1(on_time);
}

void timer1_A2_start_capture() {
    echo_rising_edge = echo_falling_edge = 0;
    TA1CTL = TASSEL_2 + MC_2 + TACLR;
    // enable interrupt for capture
    TA1CCTL2 |= CCIE;
    // enter sleep
    __bis_SR_register(LPM0_bits + GIE);
}

void generate_trigger_for_distance_sensor() {
    // Set out=1
    DIST_TRIGGER_OUT |= DIST_TRIGGER_MUSK;
    // Wait for ~10us -> set 11 for adjusted SMCLK frequency.
    timer0_start_delay(11);
    // Set out=0
    DIST_TRIGGER_OUT &= ~DIST_TRIGGER_MUSK;
}
//-------------------------------------------------------------

//-------------------------------------------------------------
//                          ISRs
//-------------------------------------------------------------
// Timer A0 ISR -> end of delay
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VACTOR))) Timer_A (void)
#else
#error Compiler not supported!
#endif
{
    LPM0_EXIT;
    TACTL = MC_0+TACLR;
}

// TA1IV ISR for input capture on TA1CCR2 interrupt.
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A1_VACTOR))) Timer1_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    //static unsigned int count = 0x0;
    //print_num(count+1, 16, 1, 0x39);
    switch (__even_in_range(TA1IV, 0x0A)) {
        case TA1IV_NONE:
            break;
        case TA1IV_TACCR1:
            TA1CTL &= ~TAIFG;
            break;
        case TA1IV_TACCR2:
            //if (TA1CCTL2 & CCIFG) {
                // This means we are on the rising edge.
                if (TA1CCTL2 & CCI) {
                    // print counter
                    //print_num(count+1, 16, 1, 0x35);
                    echo_rising_edge = TA1CCR2;
                    //TA1CCTL2 &= ~CCIFG;
                    //count++;
                }
                // This means we are on the falling edge. 
                else {
                    // print counter
                    //print_num(count+1, 16, 1, 0x36);
                    echo_falling_edge = TA1CCR2;
                    // disable interrupt
                    TA1CCTL2 &= ~CCIE;
                    //TA1CCTL2 &= ~CCIFG;
                    //count = 0x0;
                    // exit sleep
                    __bic_SR_register_on_exit(LPM0_bits + GIE);
                }
            //}
            break;
        case TA1IV_6:
            break;
        case TA1IV_8:
            break;
        case TA1IV_TAIFG:
            break;
        default:
            break;
    }
}

// Uart Tx ISR
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR() {
    static uint16_t i = 0;
    distance_sample_t sample;

    if (i >= 3) {
        UCA0TXBUF = '\n';
        i = 0;
    } else if (transmit_buffer.size > 0) {
        sample = transmit_buffer.buffer[transmit_buffer.read];
        if (i == 0) {
            UCA0TXBUF = (uint8_t)sample.distance_cm;
            i++;
        } else if (i == 1) {
            UCA0TXBUF = (uint8_t)(sample.distance_cm >> 8);
            i++;
        } else {
            UCA0TXBUF = sample.angle;
            transmit_buffer.read += sizeof(distance_sample_t);
            transmit_buffer.read %= BUFFER_LEN;
            transmit_buffer.size--;
            i++;
        }
    }
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR() {
    static uint8_t input_flag = 0;

    if (input_flag) {
        requested_angle = UCA0RXBUF;
        input_flag = !input_flag;
    } else {
        switch (UCA0RXBUF) {
            case '1':
                state = state1;
                break;
            case '2':
                state = state2;
                input_flag = !input_flag;
                break;
            default:
                state = state0;
                break;
        }
    }

    // exit lpm
    switch (lpm_mode) {
        case mode0:
            LPM0_EXIT;
            break;
        case mode1:
            LPM1_EXIT;
            break;
        case mode2:
            LPM2_EXIT;
            break;
        case mode3:
            LPM3_EXIT;
            break;
        case mode4:
            LPM4_EXIT;
            break;
    }
}
//-------------------------------------------------------------
