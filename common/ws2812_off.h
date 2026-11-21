/*
 * ws2812_off.h —— 把板载 WS2812（RGB 灯，GPIO48）熄灭
 *
 * 为什么要专门写：WS2812 是"带锁存"的，断电才自动灭；只要芯片一直通电
 * （烧录、复位都不断电），它会一直亮着上一帧的颜色。要灭就得发一帧全 0。
 *
 * 用 RMT 发 24 个"0"位（T0H=0.3µs / T0L=0.9µs @10MHz 计数）。
 * 不能靠 GPIO + 忙等，0.3µs 用 delay_us 打不出来。
 */
#ifndef WS2812_OFF_H
#define WS2812_OFF_H

#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

static inline esp_err_t ws2812_off(int gpio)
{
    rmt_channel_handle_t chan = NULL;
    const rmt_tx_channel_config_t tx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = gpio,
        .mem_block_symbols = 48,
        .resolution_hz = 10 * 1000 * 1000,   /* 1 tick = 0.1 µs */
        .trans_queue_depth = 1,
    };
    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &chan);
    if (err != ESP_OK) {
        return err;
    }

    rmt_encoder_handle_t enc = NULL;
    rmt_copy_encoder_config_t copy_cfg;   /* 空结构体，不需要填 */
    err = rmt_new_copy_encoder(&copy_cfg, &enc);
    if (err != ESP_OK) {
        rmt_del_channel(chan);
        return err;
    }

    const rmt_symbol_word_t zero_bit = {
        .level0 = 0, .duration0 = 3,     /* 0.3 µs 高 */
        .level1 = 0, .duration1 = 9,     /* 0.9 µs 低 */
    };
    rmt_symbol_word_t syms[24];
    for (int i = 0; i < 24; i++) {
        syms[i] = zero_bit;
    }

    err = rmt_enable(chan);
    if (err == ESP_OK) {
        const rmt_transmit_config_t tx_cfg_run = { .loop_count = 0 };
        err = rmt_transmit(chan, enc, syms, sizeof(syms), &tx_cfg_run);
        if (err == ESP_OK) {
            rmt_tx_wait_all_done(chan, 100);
        }
        rmt_disable(chan);
    }

    rmt_del_encoder(enc);
    rmt_del_channel(chan);

    /* 发完把数据脚按死在低电平，别让它浮空去捡噪声 */
    gpio_reset_pin(gpio);
    gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(gpio, 0);

    return err;
}

#endif /* WS2812_OFF_H */
