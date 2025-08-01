#include "../header/bsp.h"

void __GPIO_config() {
    WDTCTL = WDTHOLD | WDTPW;           // Stop WDT

    // LCD configuration
    LCD_DATA_WRITE  &= ~LCD_DATA;       // Bit clear P1.5-P1.7
    LCD_DATA_DIR    |= LCD_DATA;        // P1.5-P1.7 -> output('1')
    LCD_DATA_SEL    &= ~LCD_DATA;       // GPIO capabilities
    LCD_CTL_SEL     &= ~LCD_DATA;       // Bit clear P2.5-P2.7

    // PushButtons Setup
    // PB0
    PBsArrPortSel   &= ~BIT0;           // GPIO capability
    PBsArrPortDir   &= ~BIT0;           // input direction
    PBsArrIntEdgeSel|= BIT0;            // pull-up mode
    PBsArrIntEn     |= BIT0;
    PBsArrIntPend   &= ~BIT0;           // clear pending interrupts
    // PB1
    PBsArrPortSel   &= ~BIT3;           // GPIO capability
    PBsArrPortDir   &= ~BIT3;           // input direction
    PBsArrIntEdgeSel|= BIT3;            // pull-up mode
    PBsArrIntEn     |= BIT3;
    PBsArrIntPend   &= ~BIT3;           // clear pending interrupts

    // Servo engine -> Timer A1 PWM
    SERVO_DIR       |= BIT0;            // output mode (output compare)
    SERVO_SEL       |= BIT0;            // Primary peripheral (Timer A1)
    SERVO_OUT       &= ~BIT0;           // Set out=0

    // Distance sensor -> Trigger on Timer B + Echo on Timer A2
    // Trigger
    DIST_TRIGGER_DIR    |= BIT1;        // output mode (output compare)
    DIST_TRIGGER_SEL    |= BIT1;        // Primary peripheral (Timer B)
    DIST_TRIGGER_OUT    &= ~BIT1;       // Set out=0
    // Echo
    DIST_ECHO_DIR       &= ~BIT2;       // input mode (input capture)
    DIST_ECHO_SEL       |= BIT2;        // Primary peripheral (Timer A2)

    // LDR configuration
    LDR_DIR         &= ~LDR_MUSK;       // input mode
    LDR_SEL         &= ~LDR_MUSK;       // IO capabilities

    _BIS_SR(GIE);                 // enable interrupts globally
}

// TIMERS
void __timerA0_config() {
    TA0CTL = TASSEL_2;      // SMCLK
    TA0CCTL1 = OUTMOD_7;    // TA0CCR1 reset/set
}

void __timerA1_config() {
    TA1CTL = TASSEL_2;      // SMCLK
    TA1CCTL1 = OUTMOD_7;    // TA1CCR1 reset/set
}

void __timerA2_config() {
    TA1CCTL2 = CAP + CM_1 + CCIE + SCS + CCIS_0;
    // CM_1 - /* Capture mode: 1 - pos. edge */
    // SCS - /* Capture synchronize */
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