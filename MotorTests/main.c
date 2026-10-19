#include "pico/stdlib.h"
#include <pololu_zumo_2040_robot.h>

int main(void) {
    motors_init();
    sleep_ms(2000);

    motors_set_left_speed(1200);
    sleep_ms(1500);
    motors_set_left_speed(0);

    while(true) {
        tight_loop_contents();
    }
}
