#include "../header/api.h"

volatile FSM_state_t state;
volatile SYS_mode_t lpm_mode;

volatile circular_buffer_t transmit_buffer;

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
                enterLPM(mode0);
                break;
            case state1:
                scan_with_motor();
                break;
            case state2:
                scan_at_given_angle();
            default:
                state=state0;
                break;
        }
    }
}
