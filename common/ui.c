/*
 * ui.c —— 128x64 界面（共用一个屏幕布局，别在两块板上各画一套）
 *
 * 布局（128x64）：
 *   y 0..7    状态行：链路质量% + 对端 RSSI + 丢包
 *   y 10..40  两个摇杆盘（左：Rz + 油门 Z；右：X + Y）
 *   y 44..62  24 个按键位，上排 bit0-11，下排 bit12-23
 */
#include <stdio.h>
#include <string.h>

#include "ssd1306.h"
#include "ui.h"

#define FULL 660
#define STICK_R   15
#define STICK_LX  32
#define STICK_RX  96
#define STICK_CY  25
#define BTN_ROW1  47
#define BTN_ROW2  58

static void draw_stick(int cx, int cy, int ax, int ay)
{
    oled_circle(cx, cy, STICK_R);
    oled_hline(cx - 3, cy, 7);
    oled_vline(cx, cy - 3, 7);

    int dx = (int)((int32_t)ax * (STICK_R - 2) / FULL);
    int dy = (int)((int32_t)ay * (STICK_R - 2) / FULL);
    if (dx > STICK_R - 2) dx = STICK_R - 2;
    if (dx < -(STICK_R - 2)) dx = -(STICK_R - 2);
    if (dy > STICK_R - 2) dy = STICK_R - 2;
    if (dy < -(STICK_R - 2)) dy = -(STICK_R - 2);
    oled_disc(cx + dx, cy - dy, 2);   /* 原始"推上=正"，屏幕向下为正 */
}

static void draw_buttons(uint32_t buttons)
{
    for (int i = 0; i < 24; i++) {
        int col = i % 12;
        int row = i / 12;
        int cx = 8 + col * 10;
        int cy = row == 0 ? BTN_ROW1 : BTN_ROW2;
        if (buttons & (1u << i)) {
            oled_disc(cx, cy, 3);
        } else {
            oled_pixel(cx, cy);
        }
    }
}

void ui_live(const ui_state_t *s)
{
    char line[24];
    oled_clear();

    /* 顶行分三段放：左边链路+信号，中间拨盘，右边丢包。
       分段画是为了插值时不会把右边的数字挤掉。 */
    if (s->link_on) {
        snprintf(line, sizeof(line), "%3lu%%%+5ddB",
                 (unsigned long)s->pct, s->rssi);
    } else {
        snprintf(line, sizeof(line), "NO LINK");
    }
    oled_text(0, 0, line);

    snprintf(line, sizeof(line), "D%+4d", s->dial);
    oled_text(70, 0, line);

    if (s->link_on) {
        uint32_t lost = s->lost > 999 ? 999 : s->lost;
        snprintf(line, sizeof(line), "L%lu", (unsigned long)lost);
        oled_text(104, 0, line);
    }

    draw_stick(STICK_LX, STICK_CY, s->rz, s->z);
    draw_stick(STICK_RX, STICK_CY, s->x, s->y);
    draw_buttons(s->buttons);

    oled_flush();
}

void ui_message(const char *l1, const char *l2)
{
    oled_clear();
    if (l1 && l1[0]) {
        oled_text_center(20, l1);
    }
    if (l2 && l2[0]) {
        oled_text_center(34, l2);
    }
    oled_flush();
}
