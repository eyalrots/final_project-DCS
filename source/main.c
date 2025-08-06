#include "../header/api.h"

volatile FSM_state_t state;
volatile SYS_mode_t lpm_mode;

volatile circular_buffer_t transmit_buffer;

int main() {
    state = state0;
    lpm_mode = mode0;
    system_config();
    lcd_clear();

    // init transmit buffer
    transmit_buffer.size = 0;
    transmit_buffer.read = 0;
    transmit_buffer.write = 0;

    while (1) {
        switch (state) {
            case state0:
                enterLPM(mode0);
                break;
            case state1:
                scan_with_motor();
                state=state0;
                break;
            case state2:
                counting();
            default:
                state=state0;
                break;
        }
    }
}
