#ifndef _halGPIO_H_
#define _halGPIO_H_

#include  "../header/bsp.h"             // private library - BSP layer
// #include  "../header/app.h"          // private library - APP layer

extern void system_config();
extern void delay(unsigned int);
extern void enterLPM(unsigned char);
extern void enable_interrupts();
extern void disable_interrupts();

extern __interrupt void PBs_handler();
extern __interrupt void USCI0RX_ISR();
extern __interrupt void USCI0TX_ISR();
extern __interrupt void ADC_handler();
extern __interrupt void timerA0_handler(void);
extern __interrupt void timerA1_handler(void);

// #define CHECKBUSY    1  // using this define, only if we want to read from LCD

#ifdef CHECKBUSY
    #define LCD_WAIT lcd_check_busy()
#else
    #define LCD_WAIT DelayMs(5)
#endif

/*----------------------------------------------------------
  CONFIG: change values according to your port pin selection
------------------------------------------------------------*/
#define LCD_EN(a)   (!a ? (P2OUT&=~0X20) : (P2OUT|=0X20)) // P2.5 is lcd enable pin
#define LCD_EN_DIR(a)   (!a ? (P2DIR&=~0X20) : (P2DIR|=0X20)) // P2.5 pin direction

#define LCD_RS(a)   (!a ? (P2OUT&=~0X40) : (P2OUT|=0X40)) // P2.6 is lcd RS pin
#define LCD_RS_DIR(a)   (!a ? (P2DIR&=~0X40) : (P2DIR|=0X40)) // P2.6 pin direction

#define LCD_RW(a)   (!a ? (P2OUT&=~0X80) : (P2OUT|=0X80)) // P2.7 is lcd RW pin
#define LCD_RW_DIR(a)   (!a ? (P2DIR&=~0X80) : (P2DIR|=0X80)) // P2.7 pin direction

#define LCD_DATA_OFFSET 0x04 //data pin selection offset for 4 bit mode, variable range is 0-4, default 0 - Px.0-3, no offset


/*---------------------------------------------------------
  END CONFIG
-----------------------------------------------------------*/
#define FOURBIT_MODE    0x0
#define EIGHTBIT_MODE   0x1
#define LCD_MODE        FOURBIT_MODE

#define OUTPUT_PIN      1
#define INPUT_PIN       0
#define OUTPUT_DATA     (LCD_MODE ? 0xFF : (0x0F << LCD_DATA_OFFSET))
#define INPUT_DATA      0x00

#define LCD_STROBE_READ(value)  LCD_EN(1), \
                asm("nop"), asm("nop"), \
                value=LCD_DATA_READ, \
                LCD_EN(0)

#define lcd_cursor(x)       lcd_cmd(((x)&0x7F)|0x80)
#define lcd_clear()         lcd_cmd(0x01)
#define lcd_putchar(x)      lcd_data(x)
#define lcd_goto(x)         lcd_cmd(0x80+(x))
#define lcd_cursor_right()  lcd_cmd(0x14)
#define lcd_cursor_left()   lcd_cmd(0x10)
#define lcd_display_shift() lcd_cmd(0x1C)
#define lcd_home()          lcd_cmd(0x02)
#define cursor_off          lcd_cmd(0x0C)
#define cursor_on           lcd_cmd(0x0F)
#define lcd_function_set    lcd_cmd(0x3C) // 8bit,two lines,5x10 dots
#define lcd_new_line        lcd_cmd(0xC0)

extern void lcd_cmd(unsigned char);
extern void lcd_data(unsigned char);
extern void lcd_puts(const char * s);
extern void lcd_init();
extern void lcd_strobe();
extern void DelayMs(unsigned int);
extern void DelayUs(unsigned int);

/* open flash for reading / writing */
inline void open_flash(void) {
    while(FCTL3 & BUSY);
    /* unlock */
    FCTL3 = FWKEY;
    /* write mode */
    FCTL1 = FWKEY + WRT;
}
/* close flash */
inline void close_flash(void) {
    while(FCTL3 & BUSY);
    /* clear WRT */
    FCTL1 = FWKEY;
    /* relock */
    FCTL3 = FWKEY + LOCK;
}

// Enable Transmission to PC
inline void uart_tx_enable(void) {
    IE2 |= UCA0TXIE;
}
// Disable Transmission to PC
inline void uart_tx_disable(void) {
    IE2 &= ~UCA0TXIE;
}
// Enable Receiving from PC
inline void uart_rx_enable(void) {
    IE2 |= UCA0RXIE;
}
// Disable Receiving from PC
inline void uart_rx_disable(void) {
    IE2 &= ~UCA0RXIE;
}
void uart_write(uint8_t *buffer, uint16_t size);

void turn_off_pwm();
void turn_on_pwm();
void timer0_start_delay(unsigned int time_us, uint8_t unit);
void generate_pwm_wave_with_Ton_at_freq(unsigned int on_time, unsigned int freq);
void timer1_A2_start_capture();
void generate_trigger_for_distance_sensor();
// void read_adc_mem(uint16_t* mem_value);
void enable_ADC(void);
void disable_ADC(void);
void ADCconfigLDR1();
void ADCconfigLDR2();
unsigned int sample_ADC();
void erase_seg(uint8_t* seg_addr);

void print_b(char b, int start);
void print_num(uint16_t num, int start, int len, char fill);

inline void disconnect_from_pwm() {
    TA1CCTL1 = OUTMOD_5;
}

inline void connect_to_pwm() {
    TA1CCTL1 = OUTMOD_7;
}

#endif
