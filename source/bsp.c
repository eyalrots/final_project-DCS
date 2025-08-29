#include "../header/bsp.h"

extern volatile uint8_t adc_buffer[];

void __GPIO_config() {
    WDTCTL = WDTHOLD | WDTPW;           // Stop WDT
    
    // NOTE: LCD was P1.5-7 ; changed to P1.4-7 - could generate problems!
    // LCD configuration
    LCD_DATA_WRITE  &= ~LCD_DATA;       // Bit clear P1.4-P1.7
    LCD_DATA_DIR    |= LCD_DATA;        // P1.4-P1.7 -> output('1')
    LCD_DATA_SEL    &= ~LCD_DATA;       // GPIO capabilities
    LCD_CTL_SEL     &= ~LCD_DATA;       // Bit clear P2.5-P2.7

    // PushButtons Setup
    // PB0 - P2.0 ; PB1 - P2.1
    PBsArrPortSel   &= ~PB_MUSK;        // GPIO capability
    PBsArrPortDir   &= ~PB_MUSK;        // input direction
    PBsArrIntEdgeSel|= PB_MUSK;         // pull-up mode
    PBsArrIntEn     |= PB_MUSK;
    PBsArrIntPend   &= ~PB_MUSK;        // clear pending interrupts

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

    // LDR configuration
    // LDR1 - P1.0 (A0@ADC)
    LDR_DIR         &= ~LDR1_MUSK;       // input mode
    LDR_SEL         &= ~LDR1_MUSK;       // I/O capabilities
    // LDR2 - P1.3 (A3@ADC)
    LDR_DIR         &= ~LDR2_MUSK;       // input mode
    LDR_SEL         &= ~LDR2_MUSK;       // I/O capabilities

    _BIS_SR(GIE);                 // enable interrupts globally
}

// TIMERS
void __timerA0_delay_config() {
    /*
        Timer0 A0 used for delay.
        clk: SMCLK = ~1MHz.
        We want to get to a resolution of us in the delay
            therefore we do not divide the timer.
        Now TA0CCR0 value ~= delay in us.
    */
    // Clear timer register
    TA0CTL |= TACLR;
    // enable interrupt
    TA0CCTL0 = CCIE;
    // Set: clk=SMCLK
    TA0CTL = TASSEL_2 + MC_0;
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

// ADC -> getting LDR values
void __adc_config() {
    /*
        SREF_0:     V_R+ = Vcc ; V_R- = Vss.
        ADC10SHT_2: Sample and hold time: 16*ADC10CLKs.
        MSC:        Multiple sample and conversion. (page 608 @ user_guide)
        ADC10IE:    Interrupt enable.
    */
    ADC10CTL0 = SREF_0 + ADC10SHT_2 + MSC + ADC10IE;
    /* Clear pending interrupt flags */
    ADC10CTL0 &= ~ADC10IFG;
    /*
        INCH_3:     Sample from A3 down to A0.
        SHS_0:      Sample and hold source select -> ADC10SC bit.
        ACD10SSEL_3:clk = SMCLK.
        CONSEQ_3:   Reapeat sequence of channels.
    */
    ADC10CTL1 = INCH_3 + SHS_0 + ADC10SSEL_3 + CONSEQ_3;
    /* Enable P1.0 and P1.3 as analog input */
    ADC10AE0 = BIT0 + BIT3;
    /* enable continuous mode */
    ADC10DTC0 = ADC10CT;
    /* Four transfers: A3 -> A0 */
    ADC10DTC1 = BIT3;
    /* Set address for transfer buffer */
    ADC10SA = &adc_buffer;
}

void __adc_config_2(){

  WDTCTL = WDTHOLD + WDTPW;                    // Stop WDT
//   Timer_1_CTL =  MC_1 +Timer_1_SSEL2 ;             //Continious up CCR0 + SMCLK

  ADC10CTL0 = ADC10ON + ADC10IE+ ADC10SHT_3 + SREF_0;   // ADC10 On/Enable           +
                                                  // Interrupt enable          +
                                                  // use 64 x ADC10CLK cycles  +
                                                  // Set ref to Vcc and Gnd

  ADC10CTL1 = INCH_0 + ADC10SSEL_3;                    // Input channel A0 (p1.0) + SMCLK
  ADC10AE0 &=  ~0x08;                                 // P1.3 Analog enable
  ADC10AE0 |=   0x01;                                 // P1.0 Analog enable

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
