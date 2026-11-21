/*
 * ws2812.c —— WS2812 用 RMT 驱动
 *
 * 时序（10 MHz 计数，1 tick = 0.1 µs）：
 *   0 位：高 0.3 µs + 低 0.9 µs   -> {1, 3, 0, 9}
 *   1 位：高 0.8 µs + 低 0.45 µs  -> {1, 8, 0, 4}
 * 发完把数据脚按低电平（>50 µs 就是复位），灯就锁住颜色。
 *
 * 亮度用 BRIGHT_PCT 压：全亮的 WS2812 在桌面上晃眼睛，当状态灯用没必要。
 */
#include <string.h>

#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#include "ws2812.h"

static const char *TAG = "ws2812";

#define RES_HZ      10000000            /* 10 MHz */
#define BRIGHT_PCT  25                  /* 亮度百分比 0..100：25 = 满亮度的四分之一 */

static rmt_channel_handle_t s_chan;
static rmt_encoder_handle_t s_enc;
static int s_gpio = -1;

/* 一批传完再传下一批，所以静态缓冲是安全的 */
static rmt_symbol_word_t s_syms[24];

esp_err_t ws2812_init(int gpio)
{
    if (s_chan) {
        return ESP_OK;
    }

    const rmt_tx_channel_config_t tx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = gpio,
        .mem_block_symbols = 48,
        .resolution_hz = RES_HZ,
        .trans_queue_depth = 1,
    };
    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &s_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel: %s", esp_err_to_name(err));
        return err;
    }

    rmt_copy_encoder_config_t copy_cfg;
    err = rmt_new_copy_encoder(&copy_cfg, &s_enc);
    if (err != ESP_OK) {
        rmt_del_channel(s_chan);
        s_chan = NULL;
        return err;
    }
    err = rmt_enable(s_chan);
    if (err != ESP_OK) {
        rmt_del_encoder(s_enc);
        rmt_del_channel(s_chan);
        s_chan = NULL;
        s_enc = NULL;
        return err;
    }
    s_gpio = gpio;
    return ESP_OK;
}

esp_err_t ws2812_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_chan) {
        return ESP_ERR_INVALID_STATE;
    }

    r = (uint8_t)((uint32_t)r * BRIGHT_PCT / 100);
    g = (uint8_t)((uint32_t)g * BRIGHT_PCT / 100);
    b = (uint8_t)((uint32_t)b * BRIGHT_PCT / 100);

    /* WS2812 的线序是 G-R-B，高位先发 */
    const uint8_t grb[3] = { g, r, b };

    for (int i = 0; i < 24; i++) {
        int byte = i / 8;
        int bit = 7 - (i % 8);
        bool one = (grb[byte] >> bit) & 1;
        s_syms[i] = one
            ? (rmt_symbol_word_t){ .level0 = 1, .duration0 = 8, .level1 = 0, .duration1 = 4 }
            : (rmt_symbol_word_t){ .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 };
    }

    const rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    esp_err_t err = rmt_transmit(s_chan, s_enc, s_syms, sizeof(s_syms), &tx_cfg);
    if (err != ESP_OK) {
        return err;
    }
    return rmt_tx_wait_all_done(s_chan, 100);
}

void ws2812_deinit(void)
{
    if (s_chan) {
        rmt_disable(s_chan);
        rmt_del_encoder(s_enc);
        rmt_del_channel(s_chan);
        s_chan = NULL;
        s_enc = NULL;
    }
    if (s_gpio >= 0) {
        gpio_reset_pin(s_gpio);
        gpio_set_direction(s_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(s_gpio, 0);
    }
}
