/*
 * ssd1306.c —— 128x64 I2C OLED
 *
 * 页格式显存：fb[page][col]，每字节竖着 8 个点，bit0 在最上面。
 * 命令流：每帧先发 0x00 控制字节，再发命令；0x40 控制字节后是显存数据。
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "ssd1306.h"
#include "i2c_bus.h"
#include "font6x8.h"

static const char *TAG = "oled";

#define W 128
#define H 64
#define PAGES (H / 8)

static uint8_t s_fb[PAGES][W];
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

static esp_err_t cmd(uint8_t c)
{
    uint8_t b[2] = { 0x00, c };
    return i2c_master_transmit(s_dev, b, 2, 100);
}


/* 屏挂在共享总线上（i2c_bus）。屏上电慢，探 5 次再判死。 */
esp_err_t oled_init_quick(int sda_gpio, int scl_gpio)
{
    (void)sda_gpio;
    (void)scl_gpio;

    if (!i2c_bus_get()) {
        esp_err_t e = i2c_bus_init(I2C_BUS_SDA, I2C_BUS_SCL);
        if (e != ESP_OK) {
            return e;
        }
    }
    if (!s_bus) {
        s_bus = i2c_bus_get();
    }

    if (!s_dev) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = OLED_ADDR,
            .scl_speed_hz = I2C_BUS_HZ,
        };
        esp_err_t e = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
        if (e != ESP_OK) {
            return e;
        }
    }

    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 5; i++) {
        err = i2c_master_probe(s_bus, OLED_ADDR, 100);
        if (err == ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    if (err != ESP_OK) {
        const uint8_t probe[] = { 0x3C, 0x3D, 0x68 };
        char found[40] = {0};
        for (size_t i = 0; i < sizeof(probe); i++) {
            if (i2c_master_probe(s_bus, probe[i], 50) == ESP_OK) {
                size_t n = strlen(found);
                snprintf(found + n, sizeof(found) - n, " 0x%02X", probe[i]);
            }
        }
        ESP_LOGE(TAG, "0x3C 无应答；总线上找到:%s", found[0] ? found : "（什么都没有）");
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
        return err;
    }

    const uint8_t seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14,
        0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1,
        0xDB, 0x40, 0xA4, 0xA6, 0xAF,
    };
    for (size_t i = 0; i < sizeof(seq); i++) {
        err = cmd(seq[i]);
        if (err != ESP_OK) {
            return err;
        }
    }
    oled_clear();
    return oled_flush();
}

esp_err_t oled_init(int sda_gpio, int scl_gpio)
{
    esp_err_t err = oled_init_quick(sda_gpio, scl_gpio);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OLED 就绪（0x3C）");
    }
    return err;
}

void oled_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

void oled_pixel(int x, int y)
{
    if (x < 0 || x >= W || y < 0 || y >= H) {
        return;
    }
    s_fb[y >> 3][x] |= (uint8_t)(1u << (y & 7));
}

void oled_hline(int x, int y, int w)
{
    for (int i = 0; i < w; i++) {
        oled_pixel(x + i, y);
    }
}

void oled_vline(int x, int y, int h)
{
    for (int i = 0; i < h; i++) {
        oled_pixel(x, y + i);
    }
}

void oled_rect(int x, int y, int w, int h)
{
    oled_hline(x, y, w);
    oled_hline(x, y + h - 1, w);
    oled_vline(x, y, h);
    oled_vline(x + w - 1, y, h);
}

void oled_fill_rect(int x, int y, int w, int h)
{
    for (int j = 0; j < h; j++) {
        oled_hline(x, y + j, w);
    }
}

/* 中点画圆，只画轮廓；半径小的时候 1 px 线在 OLED 上够清楚 */
void oled_circle(int cx, int cy, int r)
{
    int x = r, y = 0, err = 1 - r;
    while (x >= y) {
        oled_pixel(cx + x, cy + y);
        oled_pixel(cx + y, cy + x);
        oled_pixel(cx - y, cy + x);
        oled_pixel(cx - x, cy + y);
        oled_pixel(cx - x, cy - y);
        oled_pixel(cx - y, cy - x);
        oled_pixel(cx + y, cy - x);
        oled_pixel(cx + x, cy - y);
        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x) + 1;
        }
    }
}

void oled_disc(int cx, int cy, int r)
{
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy <= r * r) {
                oled_pixel(cx + dx, cy + dy);
            }
        }
    }
}

void oled_text(int x, int y, const char *s)
{
    for (; *s; s++, x += 6) {
        unsigned char c = (unsigned char)*s;
        if (c < 32 || c > 126) {
            c = ' ';
        }
        const uint8_t *g = font6x8[c - 32];
        for (int col = 0; col < 6; col++) {
            uint8_t bits = g[col];
            for (int row = 0; row < 8; row++) {
                if (bits & (1u << row)) {
                    oled_pixel(x + col, y + row);
                }
            }
        }
    }
}

void oled_text_center(int y, const char *s)
{
    int w = (int)strlen(s) * 6;
    oled_text((W - w) / 2, y, s);
}

esp_err_t oled_flush(void)
{
    for (int p = 0; p < PAGES; p++) {
        esp_err_t err = cmd(0xB0 + p);
        if (err != ESP_OK) return err;
        err = cmd(0x00);
        if (err != ESP_OK) return err;
        err = cmd(0x10);
        if (err != ESP_OK) return err;

        uint8_t buf[1 + W];
        buf[0] = 0x40;
        memcpy(buf + 1, s_fb[p], W);
        err = i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}
