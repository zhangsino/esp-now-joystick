/*
 * ssd1306.c —— 128x64 I2C OLED
 *
 * 页格式显存：fb[page][col]，每字节竖着 8 个点，bit0 在最上面。
 * 命令流：每帧先发 0x00 控制字节，再发命令；0x40 控制字节后是显存数据。
 */
#include <string.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "ssd1306.h"
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


esp_err_t oled_init(int sda_gpio, int scl_gpio)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_ADDR,
        .scl_speed_hz = 400000,
    };
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        return err;
    }

    /* 上电后先确认屏在总线上，不在就早点报出来，别让后面全是超时刷屏 */
    err = i2c_master_probe(s_bus, OLED_ADDR, 200);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x3C 无应答（SDA=%d SCL=%d）: %s", sda_gpio, scl_gpio, esp_err_to_name(err));
        return err;
    }

    const uint8_t seq[] = {
        0xAE,             /* display off */
        0xD5, 0x80,       /* clock div */
        0xA8, 0x3F,       /* multiplex 64 */
        0xD3, 0x00,       /* display offset 0 */
        0x40,             /* start line 0 */
        0x8D, 0x14,       /* charge pump on */
        0x20, 0x00,       /* horizontal addressing */
        0xA1,             /* segment remap */
        0xC8,             /* COM 反扫 -> 正立（实测） */
        0xDA, 0x12,       /* COM pins */
        0x81, 0xCF,       /* contrast */
        0xD9, 0xF1,       /* precharge */
        0xDB, 0x40,       /* VCOMH */
        0xA4,             /* 显示显存 */
        0xA6,             /* 正常（非反白） */
        0xAF,             /* display on */
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
