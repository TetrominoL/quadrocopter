#include <stdint.h>

void esc_init();

void esc_set_speed_all(uint8_t speed);

void esc_set_speed(uint8_t motorNo, uint8_t speed);

uint8_t esc_get_speed(uint8_t motorNo);

void esc_calib();

void esc_set_speed_all_abs(uint8_t speed);

void esc_set_speed_abs(uint8_t motorNo, uint8_t speed);