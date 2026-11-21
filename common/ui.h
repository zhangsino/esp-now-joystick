/*
 * ui.h —— 两块板共用的 128x64 界面
 *
 * 发送板（手持端）和接收板（无头盒子）画的是同一套画面，只是数据来源不同：
 *   发送板：数据来自本地 RC2，link = 有没有收到接收板回的状态包
 *   接收板：数据来自空口，  link = 有没有收到摇杆包
 */
#ifndef UI_H
#define UI_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool     rc2_on;      /* RC2 认到了 */
    bool     link_on;     /* 对方在不在（1 秒内有包） */
    int16_t  x, y, z, rz, dial;
    uint32_t buttons;
    int16_t  rssi;        /* 对端看到的信号 dBm */
    uint32_t lost;        /* 累计丢包 */
    uint32_t pct;         /* 链路质量 0..100 */
} ui_state_t;

void ui_live(const ui_state_t *s);                  /* 实时画面：摇杆盘 + 24 点 */
void ui_message(const char *l1, const char *l2);    /* 两行大字 */

#endif
