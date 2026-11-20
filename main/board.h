#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

enum { LCD_W = 240, LCD_H = 135, LCD_STRIPE = 15 };
typedef enum { KEY_NONE, KEY_TAP, KEY_LONG } key_event_t;
typedef struct { key_event_t a, b; } keys_t;
esp_err_t board_init(void);
esp_err_t board_backlight_on(void);
/* Takes a DMA-capable native RGB565 stripe; converts it in place for SPI. */
esp_err_t board_push_stripe(int y, uint16_t *rgb);
keys_t board_keys(void);
