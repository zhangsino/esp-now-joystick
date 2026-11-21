/*
 * ssd1306.h —— 128x64 I2C OLED 最小驱动（ESP-IDF v5.5 新版 i2c_master）
 *
 * 只用到一个显存缓冲（1 KB，页格式）：
 *   写完所有图元再调 oled_flush()，一次把 8 页全推过去。
 *   400 kHz 下一帧约 23 ms，所以刷新上限约 40 fps；接收板只干这一件事，够用。
 */
#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/* 引脚按 hobby/AGENTS.md 的既定分配：SDA=17 SCL=18（和 IMU 共用，地址不冲突） */
#define OLED_SDA_GPIO 17
#define OLED_SCL_GPIO 18
#define OLED_ADDR     0x3C

esp_err_t oled_init(int sda_gpio, int scl_gpio);
void oled_clear(void);
void oled_pixel(int x, int y);
void oled_hline(int x, int y, int w);
void oled_vline(int x, int y, int h);
void oled_rect(int x, int y, int w, int h);            /* 空心 */
void oled_fill_rect(int x, int y, int w, int h);       /* 实心 */
void oled_circle(int cx, int cy, int r);               /* 空心圆 */
void oled_disc(int cx, int cy, int r);                 /* 实心圆 */
void oled_text(int x, int y, const char *s);           /* 6x8，y 为字符顶边 */
void oled_text_center(int y, const char *s);
esp_err_t oled_flush(void);

#endif
