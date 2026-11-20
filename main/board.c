// M5Stack StickS3 (K150): ST7789, M5PM1 and two active-low keys.
#include "board.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define TAG "sticks3"
static i2c_master_dev_handle_t pmic;
static esp_lcd_panel_handle_t lcd;
static SemaphoreHandle_t dma_done;
static QueueHandle_t key_events;
static esp_err_t start_key_polling(void);

static esp_err_t pmic_write(uint8_t reg, uint8_t data) {
    const uint8_t msg[] = {reg, data};
    return i2c_master_transmit(pmic, msg, sizeof(msg), 50);
}

static esp_err_t pmic_update(uint8_t reg, uint8_t mask, bool on) {
    uint8_t value;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(pmic, &reg, 1, &value, 1, 50), TAG, "M5PM1 read");
    return pmic_write(reg, on ? (value | mask) : (value & ~mask));
}

static esp_err_t power_init(void) {
    i2c_master_bus_handle_t bus;
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_1, .sda_io_num = GPIO_NUM_47, .scl_io_num = GPIO_NUM_48,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true,
    };
    const i2c_device_config_t pmic_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = 0x6e, .scl_speed_hz = 100000,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus), TAG, "PMIC bus");
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &pmic_cfg, &pmic), TAG, "PMIC device");
    uint8_t reg = 0, id[2] = {0};
    esp_err_t err = ESP_ERR_TIMEOUT;
    for (int i = 0; i < 8; ++i) {
        err = i2c_master_transmit_receive(pmic, &reg, 1, id, sizeof(id), 25);
        if (err == ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(25));
    }
    ESP_RETURN_ON_ERROR(err, TAG, "M5PM1 not found");
    ESP_RETURN_ON_FALSE(id[0] == 0x50 && id[1] == 0x20, ESP_ERR_NOT_FOUND, TAG, "unexpected PMIC ID: %02x%02x", id[1], id[0]);
    // Keep the PMIC I2C interface awake, then raise LCD L3B.
    ESP_RETURN_ON_ERROR(pmic_write(0x09, 0), TAG, "M5PM1 I2C awake");
    ESP_RETURN_ON_ERROR(pmic_update(0x16, 0x04, false), TAG, "L3B GPIO select");
    ESP_RETURN_ON_ERROR(pmic_update(0x10, 0x04, true), TAG, "L3B GPIO output");
    ESP_RETURN_ON_ERROR(pmic_update(0x13, 0x04, false), TAG, "L3B push-pull");
    ESP_RETURN_ON_ERROR(pmic_update(0x11, 0x04, true), TAG, "L3B enable");
    vTaskDelay(pdMS_TO_TICKS(20));
    return ESP_OK;
}

static bool on_color_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *ev, void *user) {
    (void)io;
    (void)ev;
    BaseType_t wake = pdFALSE;
    xSemaphoreGiveFromISR(user, &wake);
    return wake == pdTRUE;
}

esp_err_t board_init(void) {
    ESP_RETURN_ON_ERROR(power_init(), TAG, "board power");
    const gpio_config_t keys = {
        .pin_bit_mask = (1ULL << GPIO_NUM_11) | (1ULL << GPIO_NUM_12),
        .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&keys), TAG, "buttons");
    const spi_bus_config_t bus = {
        .mosi_io_num = GPIO_NUM_39, .miso_io_num = -1, .sclk_io_num = GPIO_NUM_40,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = LCD_W * LCD_STRIPE * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO), TAG, "LCD bus");
    esp_lcd_panel_io_handle_t io;
    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = GPIO_NUM_45, .cs_gpio_num = GPIO_NUM_41, .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8, .lcd_param_bits = 8, .trans_queue_depth = 1,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_cfg, &io), TAG, "LCD IO");
    dma_done = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(dma_done, ESP_ERR_NO_MEM, TAG, "LCD semaphore");
    const esp_lcd_panel_io_callbacks_t callbacks = {.on_color_trans_done = on_color_done};
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_register_event_callbacks(io, &callbacks, dma_done), TAG, "LCD callback");
    const esp_lcd_panel_dev_config_t lcd_cfg = {
        .reset_gpio_num = GPIO_NUM_21, .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .data_endian = LCD_RGB_DATA_ENDIAN_BIG, .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(io, &lcd_cfg, &lcd), TAG, "LCD driver");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(lcd), TAG, "LCD reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(lcd), TAG, "LCD init");
    // Match M5GFX: LCMCTRL 0x0c clears the reset-time XBGR bit that XORs MADCTL.BGR.
    const uint8_t lcmctrl = 0x0c;
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xc0, &lcmctrl, sizeof(lcmctrl)), TAG, "LCD module control");
    // INVON is the panel's normal polarity, not a user-visible negative palette.
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(lcd, true), TAG, "LCD polarity");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(lcd, true), TAG, "LCD landscape");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(lcd, true, false), TAG, "LCD mirror");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(lcd, 40, 53), TAG, "LCD offset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(lcd, true), TAG, "LCD on");
    return start_key_polling();
}

esp_err_t board_backlight_on(void) {
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 256, .clk_cfg = LEDC_AUTO_CLK,
    };
    const ledc_channel_config_t chan = {
        .gpio_num = GPIO_NUM_38, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0, .duty = 150,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "LCD backlight timer");
    return ledc_channel_config(&chan);
}

esp_err_t board_push_stripe(int y, uint16_t *rgb) {
    for (unsigned i = 0; i < LCD_W * LCD_STRIPE; ++i) rgb[i] = __builtin_bswap16(rgb[i]);
    ESP_RETURN_ON_ERROR(esp_lcd_panel_draw_bitmap(lcd, 0, y, LCD_W, y + LCD_STRIPE, rgb), TAG, "draw");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(dma_done, pdMS_TO_TICKS(100)) == pdTRUE,
                        ESP_ERR_TIMEOUT, TAG, "LCD DMA deadline");
    return ESP_OK;
}

typedef struct { bool raw, stable, sent; int64_t changed, pressed; } button_t;
static key_event_t read_key(button_t *b, gpio_num_t pin, int64_t now) {
    bool down = !gpio_get_level(pin);
    if (down != b->raw) { b->raw = down; b->changed = now; }
    if (down != b->stable && now - b->changed >= 20000) {
        b->stable = down;
        if (down) { b->pressed = b->changed; b->sent = false; }
        else if (!b->sent) return KEY_TAP;
    }
    if (b->stable && b->raw && !b->sent && now - b->pressed >= 550000) {
        b->sent = true;
        return KEY_LONG;
    }
    return KEY_NONE;
}

static void poll_keys(void *arg) {
    (void)arg;
    static button_t a, b;
    const int64_t now = esp_timer_get_time();
    const keys_t keys = {read_key(&a, GPIO_NUM_11, now), read_key(&b, GPIO_NUM_12, now)};
    if (keys.a || keys.b) xQueueSend(key_events, &keys, 0);
}

static esp_err_t start_key_polling(void) {
    key_events = xQueueCreate(8, sizeof(keys_t));
    ESP_RETURN_ON_FALSE(key_events, ESP_ERR_NO_MEM, TAG, "button queue");
    // Collect taps while the measurement task processes a long record.
    const esp_timer_create_args_t args = {
        .callback = poll_keys, .name = "keys", .skip_unhandled_events = true,
    };
    esp_timer_handle_t timer;
    ESP_RETURN_ON_ERROR(esp_timer_create(&args, &timer), TAG, "button timer");
    return esp_timer_start_periodic(timer, 5000);
}

keys_t board_keys(void) {
    keys_t keys = {0};
    xQueueReceive(key_events, &keys, 0);
    return keys;
}
