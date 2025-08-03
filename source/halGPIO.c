#include "../header/halGPIO.h"

extern volatile FSM_state_t state;
extern volatile SYS_mode_t lpm_mode;

extern volatile unsigned int echo;

// System configuration
void system_config() {
    __GPIO_config();
    __timerA0_config();
    __timerA0_reg_2_delay_config();
    __timerA1_reg_0_1_config();
    __timerA1_reg_2_config();
    __adc_config();
    __UART_config();
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
void print_num(unsigned int num, int start, int len, char fill) {
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
// Timer A0
void set_TA0CCR0(unsigned int new_value) {
    TA0CCR0 = new_value;
}
void set_TA0CCR1(unsigned int new_value) {
    TA0CCR1 = new_value;
}
void set_TA0CCR2(unsigned int new_value) {
    TA0CCR2 = new_value;
}
void start_timer_delay() {
    TA0CTL |= MC_1;
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
// Enable Transmission to PC
void uart_tx_enable(void) {
    IE2 |= UCA0TXIE;
}
// Disable Transmission to PC
void uart_tx_disable(void) {
    IE2 &= ~UCA0TXIE;
}
// Enable Receiving from PC
void uart_rx_enable(void) {
    IE2 |= UCA0RXIE;
}
// Disable Receiving from PC
void uart_rx_disable(void) {
    IE2 &= ~UCA0RXIE;
}
//-------------------------------------------------------------

//-------------------------------------------------------------
//                          Functions
//-------------------------------------------------------------
void generate_pwm_wave_at_ton_freq(int timer, unsigned int on_time, unsigned int freq) {
    // T_on received in us
    unsigned int pwm_period = 0;

    // on_time is in us and SMCLK freq is ~1MHz => register value ~= on_time.
    // set PWM period
    pwm_period = SMCLK / freq;
    // set timer values
    if (timer) {    // Timer A1
        set_TA1CCR0(pwm_period);
        set_TA1CCR1(on_time);
    } else {        // Timer A0 -> duty cycle = 50%
        set_TA0CCR0(pwm_period);
        set_TA0CCR1(pwm_period >> 1);
    }
}

void timer_delay(unsigned int delay) {
    // Delay in ms
    unsigned int actual_value = (delay * SMCLK) / 1000;
    set_TA0CCR2(actual_value);
    // start clk
    start_timer_delay();
}
//-------------------------------------------------------------

//-------------------------------------------------------------
//                          ISRs
//-------------------------------------------------------------
// for vscode to not give error now -> It is a TI compiler function...
int __even_in_range(int val1, int val2) {
    return 0;
}

// TA0IV ISR for delay functionality on TA0CCR2 interrupt
#pragma vector=TIMER0_A2_VECTOR
__interrupt void timerA0_handler(void) {
    switch (__even_in_range(TA0IV, 0x0A)) {
        case TA0IV_NONE:
            break;
        case TA0IV_TACCR1:
            TA1CTL &= ~TAIFG;
            break;
        case TA0IV_TACCR2:
            LPM0_EXIT;
            TA0CTL = MC_0+TACLR;
            break;
        case TA0IV_6:
            break;
        case TA0IV_8:
            break;
        case TA0IV_TAIFG:
            break;
        default:
            break;
    }
}

// TA1IV ISR for input capture on TA1CCR2 interrupt
#pragma vector=TIMER1_A2_VECTOR
__interrupt void timerA1_handler(void) {
    switch (__even_in_range(TA1IV, 0x0A)) {
        case TA1IV_NONE:
            break;
        case TA1IV_TACCR1:
            TA1CTL &= ~TAIFG;
            break;
        case TA1IV_TACCR2:
            if (TA1CCTL2 & CCI) {
                echo = TA1CCR2;
            }
            break;
        case TA1IV_TAIFG:
            break;
        default:
            break;
    }
}
//-------------------------------------------------------------

// edited using tablet we app!