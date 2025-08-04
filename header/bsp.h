#ifndef _bsp_H_
#define _bsp_H_

#include    "./msp_header/msp430g2553.h"    // VS-Code
//#include    <msp430g2553.h>                 // CCS


#define   LCD_DATA 0xF0
#define   debounceVal      250
#define   SMCLK            1095870

// LCDs abstraction
#define LCD_DATA_WRITE      P1OUT
#define LCD_DATA_DIR        P1DIR
#define LCD_DATA_READ       P1IN
#define LCD_DATA_SEL        P1SEL
#define LCD_CTL_SEL         P2SEL

// Servo engine -> Timer A1 PWM
#define SERVO_DIR           P2DIR
#define SERVO_SEL           P2SEL
#define SERVO_OUT           P2OUT

// Distance sensor -> Trigger on Timer A0 + Echo on Timer A2
#define DIST_TRIGGER_DIR    P2DIR
#define DIST_TRIGGER_SEL    P2SEL
#define DIST_TRIGGER_OUT    P2OUT
#define DIST_TRIGGER_MUSK   BIT3
#define DIST_ECHO_DIR       P2DIR
#define DIST_ECHO_SEL       P2DIR
#define DIST_ECHO_IN        P2IN
#define DIST_ECHO_MUSK      BIT4 // capture on A1 -> P2.4 (Data Sheet).

// LDR
#define LDR_DIR             P2DIR
#define LDR_SEL             P2SEL
#define LDR_IN              P2IN
#define LDR_MUSK            0x03    // 0011b

// UART abstraction
#define TXD BIT2
#define RXD BIT1

// PushButtons abstraction
#define PBsArrPort         P1IN
#define PBsArrIntPend      P1IFG
#define PBsArrIntEn        P1IE
#define PBsArrIntEdgeSel   P1IES
#define PBsArrPortSel      P1SEL
#define PBsArrPortDir      P1DIR
#define PB0                0x01
#define PB1                0x02
#define PB2                0x04
#define PB3                0x08

void __GPIO_config();
void __timerA0_delay_config();;
void __timer1_pwm_config();
void __timer1_A2_capture_config();
void __adc_config();
void __UART_config();

typedef enum {
    state0,
    state1,
    state2,
    state3,
    state4,
    state5,
    state6,
    state7,
    state8,
}FSM_state_t;

typedef enum {
    mode0,
    mode1,
    mode2,
    mode3,
    mode4
}SYS_mode_t;

#endif