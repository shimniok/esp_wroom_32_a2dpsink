#include <driver/gpio.h>
#include <stdint.h>

#define INIT_PIN(pin)                                   \
  {                                                     \
    gpio_set_direction((pin), GPIO_MODE_OUTPUT);        \
    gpio_set_drive_capability((pin), GPIO_DRIVE_CAP_3); \
    gpio_set_level((pin), 1);                           \
  }

void lm1972_init(void) {
  INIT_PIN(CONFIG_EXAMPLE_LM1972_CLK_PIN);
  INIT_PIN(CONFIG_EXAMPLE_LM1972_DIN_PIN);
  INIT_PIN(CONFIG_EXAMPLE_LM1972_LD_PIN);
}

/**
 * lm1972_send_bit
 * @brief  send next bit to the LM1972
 * @param  is_one  send 1 if true, 0 otherwise
 */
void lm1972_send_bit(uint8_t is_one) {}

void lm1972_set_clk(uint8_t is_high) {}

void lm1972_set_data(uint8_t is_high) {}

void lm1972_set_ld(uint8_t is_high) {}

void lm1972_send_byte(uint8_t byte) {
  uint8_t b;
  for (b = 0xf0; b > 0; b >>= 1) {
    lm1972_send_bit(((b & byte) == b) ? 1 : 0);
  }
}

void lm1972_set_volume(uint8_t channel, uint8_t volume) {
  lm1972_send_byte(channel);
  lm1972_send_byte(volume);
}