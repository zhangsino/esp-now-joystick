/*
 * now_proto.h —— 摇杆板 ESP-NOW 空口协议（发送端/接收端共用）
 *
 * 只依赖 stdint.h，C 和 C++ 都能用。改这里 = 两端都要重烧。
 *
 * 设计约束：
 *   - ESP-NOW 单包上限 250 B，这里 22 B，留足余量；
 *   - 小端、紧凑排列，收发两端同一份结构体，不做字节序转换（都是 Xtensa LE）；
 *   - 只有一个校验字节（异或）。ESP-NOW 本身有 802.11 CRC，这里只是防"串台"和改坏版本。
 */
#ifndef NOW_PROTO_H
#define NOW_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define NOW_MAGIC0 0x4A          /* 'J' 摇杆包 */
#define NOW_MAGIC1 0x53          /* 'S' */
#define NOW_ST0    0x52          /* 'R' 接收板回传的状态包 */
#define NOW_ST1    0x53          /* 'S' */
#define NOW_VER    3

/* flags 位定义 */
#define NOW_FLAG_RC2_ON    (1u << 0)   /* RC2 已枚举、正在出报告 */
#define NOW_FLAG_CAL       (1u << 1)   /* 正在标定中位 */
#define NOW_FLAG_ARMED     (1u << 2)   /* 允许车动（谁解锁由手持端决定，车只管执行） */
#define NOW_FLAG_TILT      (1u << 3)   /* 本次数据来自 6050 倾斜，不是摇杆 */

/* 轴满行程（实测，见 dji_rc2/README.md：逻辑 ±1024，实际只到 ±660） */
#define NOW_AXIS_FULL 660

typedef struct __attribute__((packed)) {
    uint8_t  magic[2];      /* 'J','S' */
    uint8_t  ver;           /* NOW_VER */
    uint8_t  flags;         /* NOW_FLAG_* */
    uint32_t seq;           /* 发送序号，从 1 递增；接收端据此算丢包 */
    int16_t  x;             /* 右摇杆 左右   -660..+660（右为 +） */
    int16_t  y;             /* 右摇杆 上下   -660..+660（上为 +） */
    int16_t  z;             /* 左摇杆 上下   -660..+660（上为 +，无回中弹簧） */
    int16_t  rz;            /* 左摇杆 左右   -660..+660（右为 +） */
    int16_t  dial;          /* 右侧拨盘（第 5 轴，byte 11-12），回中 */
    uint32_t buttons;       /* 24 位按键，bit0..bit23 */
    uint8_t  rc2_hz;        /* RC2 原始报告率（约 247），仅显示用 */
    uint8_t  xor8;          /* 从 magic[0] 到 rc2_hz 的异或 */
} now_joystick_t;

/* 字节数在编译期就得定住，别到板上才发现结构体被填充 */
typedef char now_joystick_size_check[(sizeof(now_joystick_t) == 24) ? 1 : -1];

static inline uint8_t now_xor8(const now_joystick_t *p)
{
    const uint8_t *b = (const uint8_t *)p;
    uint8_t v = 0;
    for (size_t i = 0; i < offsetof(now_joystick_t, xor8); i++) {
        v ^= b[i];
    }
    return v;
}

static inline void now_pack(now_joystick_t *p)
{
    p->magic[0] = NOW_MAGIC0;
    p->magic[1] = NOW_MAGIC1;
    p->ver = NOW_VER;
    p->xor8 = now_xor8(p);
}

/* ---------------- 接收板 -> 发送板：状态包 ---------------- *
 * 接收板每 200 ms 播一次，发送板据此知道"我的包被收到了"以及对方看到的信号。
 * 用 MAC 层单播发回去（esp_now_send 给单播地址时，回调里的状态就是对方到底收没收到）。
 */
typedef struct __attribute__((packed)) {
    uint8_t  magic[2];      /* 'R','S' */
    uint8_t  ver;
    uint8_t  flags;
    uint32_t rx_seq;        /* 最后收到的摇杆包序号 */
    uint32_t rx_total;      /* 累计收到多少包 */
    uint32_t lost;          /* 估算丢包 */
    int8_t   rssi;          /* 收摇杆包时的信号强度 */
    uint8_t  rx_hz;         /* 每秒收包数 */
    uint8_t  xor8;
} now_status_t;

typedef char now_status_size_check[(sizeof(now_status_t) == 19) ? 1 : -1];

static inline uint8_t now_xor8_buf(const void *p, size_t n)
{
    const uint8_t *b = (const uint8_t *)p;
    uint8_t v = 0;
    for (size_t i = 0; i < n; i++) {
        v ^= b[i];
    }
    return v;
}

static inline void now_status_pack(now_status_t *p)
{
    p->magic[0] = NOW_ST0;
    p->magic[1] = NOW_ST1;
    p->ver = NOW_VER;
    p->xor8 = now_xor8_buf(p, offsetof(now_status_t, xor8));
}

/* 返回 0 = 合法 */
static inline int now_status_unpack(const now_status_t *p)
{
    if (p->magic[0] != NOW_ST0 || p->magic[1] != NOW_ST1) return -1;
    if (p->ver != NOW_VER) return -2;
    if (p->xor8 != now_xor8_buf(p, offsetof(now_status_t, xor8))) return -3;
    return 0;
}

/* 返回 0 = 合法 */
static inline int now_unpack(const now_joystick_t *p)
{
    if (p->magic[0] != NOW_MAGIC0 || p->magic[1] != NOW_MAGIC1) {
        return -1;
    }
    if (p->ver != NOW_VER) {
        return -2;
    }
    if (p->xor8 != now_xor8(p)) {
        return -3;
    }
    return 0;
}

#endif /* NOW_PROTO_H */
