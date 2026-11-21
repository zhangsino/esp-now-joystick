/*
 * ws2812.h —— 板载 RGB 灯（WS2812）最小驱动
 *
 * 用 RMT 发 24 位 GRB。RMT 通道建一次、反复用；每帧 24 个 symbol ≈ 30 µs。
 * 不能在更新时反复建/删通道，那样每次要几十毫秒。
 */
#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>
#include "esp_err.h"

#define WS2812_GPIO_DEFAULT 48

esp_err_t ws2812_init(int gpio);
esp_err_t ws2812_set(uint8_t r, uint8_t g, uint8_t b);
void ws2812_deinit(void);

#endif
