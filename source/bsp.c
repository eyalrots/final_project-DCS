#include "../header/bsp.h"

void __GPIO_config() {
    WDTCTL = WDTHOLD | WDTPW;           // Stop WDT
    
    // NOTE: LCD was P1.5-7 ; changed to P1.4-7 - could generate problems!
    // LCD configuration
    LCD_DATA_WRITE  &= ~LCD_DATA;       // Bit clear P1.4-P1.7
    LCD_DATA_DIR    |= LCD_DATA;        // P1.4-P1.7 -> output('1')
    LCD_DATA_SEL    &= ~LCD_DATA;       // GPIO capabilities
    LCD_CTL_SEL     &= ~LCD_DATA;       // Bit clear P2.5-P2.7

    // PushButtons Setup
    // PB0 - P1.0
    PBsArrPortSel   &= ~BIT0;           // GPIO capability
    PBsArrPortDir   &= ~BIT0;           // input direction
    PBsArrIntEdgeSel|= BIT0;            // pull-up mode
    PBsArrIntEn     |= BIT0;
    PBsArrIntPend   &= ~BIT0;           // clear pending interrupts
    // PB1 - P1.3
    PBsArrPortSel   &= ~BIT3;           // GPIO capability
    PBsArrPortDir   &= ~BIT3;           // input direction
    PBsArrIntEdgeSel|= BIT3;            // pull-up mode
    PBsArrIntEn     |= BIT3;
    PBsArrIntPend   &= ~BIT3;           // clear pending interrupts

    // Servo engine -> Timer A1 PWM ; P2.2 as of Data-Sheet.
    SERVO_DIR       |= BIT2;            // output mode (output compare)
    SERVO_SEL       |= BIT2;            // Primary peripheral (Timer A1)
    SERVO_OUT       &= ~BIT2;           // Set out=0

    // Distance sensor -> Trigger on Timer B + Echo on Timer A2
    // Trigger - P2.3
    DIST_TRIGGER_DIR    |= DIST_TRIGGER_MUSK;       // output mode
    DIST_TRIGGER_SEL    &= ~DIST_TRIGGER_MUSK;      // I/O capabilities
    DIST_TRIGGER_OUT    &= ~DIST_TRIGGER_MUSK;      // Set out=0
    // Echo - P2.4
    DIST_ECHO_DIR       &= ~DIST_ECHO_MUSK;         // input mode (input capture)
    DIST_ECHO_SEL       |= DIST_ECHO_MUSK;          // Primary peripheral (Timer A2)

    // LDR configuration - P2.0-1
    LDR_DIR         &= ~LDR_MUSK;       // input mode
    LDR_SEL         &= ~LDR_MUSK;       // I/O capabilities

    _BIS_SR(GIE);                 // enable interrupts globally
}

// TIMERS
void __timerA0_delay_config() {
    /*
        Timer0 A0 used for delay.
        clk: SMCLK = ~1MHz.
        We want to get to a resolution of us in the delay
            therfore we do not divide the timer.
        Now TA0CCR0 value ~= delay in us.
    */
    // Clear timer register
    TA0CTL |= TACLR;
    // Set: clk=SMCLK ; enable interrupt
    TA0CTL = TASSEL_2 + TAIE;
    TA0CTL &= ~TAIFG;
}

void __timer1_pwm_config() {
    // TA1CCTL1 controls TA1CCR1 for output compare -> PWM wave duty_cycle
    TA1CTL = TASSEL_2;      // SMCLK
    TA1CCTL1 = OUTMOD_7;    // TA1CCR1 reset/set
}

void __timer1_A2_capture_config() {
    /*
        TA1CCTL2 controls TA1CCR2 for input capture
        TASSEL_2: clk = SMCLK.
        CAP: enable capture.
        CCIS_0: capture for P2.4 - CCI2A (Data Sheet).
        SCS: capture synchronize.
        CCIE: enable interrupts.
    */
    TA1CTL = TASSEL_2;      // SMCLK
    TA1CCTL2 = CAP + CM_3 + CCIE + SCS + CCIS_0;
}

// ADC -> getting LDR values (needs check -> later!)
void __adc_config() {
    ADC10CTL0 = SREF_0 + ADC10SHT_2 + MSC + ADC10IE;
    ADC10CTL0 &= ~ADC10IFG;

    ADC10CTL1 = INCH_3 + SHS_0 + ADC10SSEL_3 + CONSEQ_2;
    ADC10AE0 = BIT3; // enable P1.3 as analog input
}

// UART
void __UART_config() {

    P1SEL |= 0x06;       // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= 0x06;      // P1.1 = RXD, P1.2=TXD

    UCA0CTL1 |= UCSWRST; // hold in reset whilst configuring

    UCA0CTL1 |= UCSSEL_2; // SMCLK source
    UCA0MCTL = UCBRS0;
    UCA0BR0 = 109; //Baud Rate: 9600
    UCA0BR1 = 0;

    UCA0CTL1 &= ~UCSWRST;  // Release USCI from reset

    IE2 |= UCA0RXIE; //enable rx

}