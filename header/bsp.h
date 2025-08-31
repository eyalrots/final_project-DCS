#ifndef _bsp_H_
#define _bsp_H_

//#include    "./msp_header/msp430g2553.h"    // VS-Code
#include    <msp430g2553.h>                 // CCS
#include    <stdint.h>
#include    <stddef.h>


#define   BUFFER_LEN       8
#define   LCD_DATA         0xF0
#define   debounceVal      250
#define   SMCLK            1095870
#define   SEG_D            0x1000
#define   SEG_C            0x1040
#define   SEG_B            0x1080
#define   SEG_4            0xF600
#define   SEG_3            0xF800
#define   SEG_2            0xFA00
#define   SEG_1            0xFC00
#define   START_OF_FILES   0xF600 /* start of segment 4 */
#define   END_OF_FILES     0xFDFF /*  end of segment 1  */
#define   FILE_MEM_SIZE    0x800  /*        2KB         */
#define   LDR_CALIB        0x1080 /* LDR calibration data @ segment B */
#define   AVAILABLE_SPACE  0x10BE /*   2B number @ end of sement B    */
#define   NUM_OF_FILES     0x10BD /*   B number of files in memory    */
#define   HEADER_SIZE      10

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
#define DIST_ECHO_SEL       P2SEL
#define DIST_ECHO_IN        P2IN
#define DIST_ECHO_MUSK      BIT4 // capture on A1 -> P2.4 (Data Sheet).

// LDR
#define LDR_DIR             P1DIR
#define LDR_SEL             P1SEL
#define LDR_IN              P1IN
#define LDR1_MUSK           BIT0    // 0001b
#define LDR2_MUSK           BIT3    // 1000b

// UART abstraction
#define TXD BIT2
#define RXD BIT1

// PushButtons abstraction
#define PBsArrPort         P2IN
#define PBsArrIntPend      P2IFG
#define PBsArrIntEn        P2IE
#define PBsArrIntEdgeSel   P2IES
#define PBsArrPortSel      P2SEL
#define PBsArrPortDir      P2DIR
#define PB_MUSK            0x03
#define PB0                0x01
#define PB1                0x02
#define PB2                0x04
#define PB3                0x08

void __GPIO_config();
void __timerA0_delay_config();
void __timer1_pwm_config();
void __timer1_A2_capture_config();
void __adc_config();
void __adc_config_2();
void __UART_config();
void FlashConfig();

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

typedef enum {
    text,
    script
}file_type_t;

/* file object */
typedef struct __attribute__ ((packed)) file_header {
    uint16_t size;
    uint8_t name[7];
    uint8_t type;
    uint16_t address;
}file_header_t;

// distance sample consists of distance[cm] and angle[deg].
typedef struct __attribute__ ((packed)) distance_sample {
    uint16_t distance_cm;
    uint8_t  angle;
} distance_sample_t;

// Circular buffer of distance samples.
typedef struct __attribute__ ((packed)) circular_buffer {
    distance_sample_t buffer[BUFFER_LEN];
    uint8_t size;
    uint8_t read;
    uint8_t write;
} circular_buffer_t;

inline void memset(uint8_t *ptr, uint8_t value, uint32_t size) {
    while (size--) {
        *ptr++ = value;
    }
}

inline void memcpy(uint8_t *dst, const uint8_t *src, uint16_t size) {
    while (size--) {
        *dst++ = *src++;
    }
}
#endif
