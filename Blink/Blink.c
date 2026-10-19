#include <pico/stdlib.h>

int main() {
  stdio_init_all();
  gpio_init(25);
  gpio_set_dir(25, GPIO_OUT);

    while (1) {
      gpio_put(25, 0);     // yellow LED on
      sleep_ms(100);
      gpio_put(25, 1);     // yellow LED off
      sleep_ms(600);
    }
}
