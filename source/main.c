#include "../header/api.h"

volatile FSM_state_t state;
volatile SYS_mode_t lpm_mode;

volatile circular_buffer_t transmit_buffer;

volatile uint8_t adc_buffer[8];

int main() {
    uint8_t *ptr = NULL;

    state = state0;
    lpm_mode = mode0;
    system_config();
    lcd_clear();

    // init transmit buffer
    memset((uint8_t*)&transmit_buffer, 0, sizeof(transmit_buffer));

    while (1) {
        switch (state) {
            case state0:
                // lcd_clear();
                enterLPM(mode0);
                break;
            case state1:
                scan_with_sonic();
                state=state0;
                break;
            case state2:
                scan_at_given_angle();
                state=state0;
                break;
            case state3:
                scan_with_ldr();
                state=state0;
                break;
            case state4:
                // print_num(4, 1, 1, 0x30);
                // counting();
                // state=state0;
                break;
            case state5:
                file_mode();
                break;
            case state6:
                counting();
                break;
            case state7:
                erase_info();
                break;
            default:
                state=state0;
                break;
        }
    }
}
