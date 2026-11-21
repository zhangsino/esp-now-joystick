/*
 * 摇杆接收板 —— ESP-NOW 收包 + 回传状态（屏可选）
 *
 * 这块板现在是"无头小盒子"：收发送板的摇杆包，每 200 ms 把
 * 「我收到了多少包 / 信号多强 / 丢了多少」回传给发送板。
 * 发送板据此知道自己有没有被接收到，屏也画在发送板那一侧。
 *
 * OLED 仍然保留支持：插着就画同一套画面，没插就只做回传（oled_init 失败不影响运行）。
 *
 * 引脚：OLED SDA=17 SCL=18 VCC=3V3；日志走原生 USB 口（开了 USJ 副控制台）。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"

#include "now_proto.h"
#include "ssd1306.h"
#include "ui.h"
#include "ws2812.h"

static const char *TAG = "now_rx";

#define NOW_CHANNEL     11
#define LINK_TIMEOUT_MS 1000
#define STATUS_PERIOD_MS 200

#define LED_HOLD_MS  250    /* 动作之后亮多久 */
#define LED_MOVE_TH  40     /* 摇杆变化超过这个幅度才算动了（满行程 660） */

/* ------------------------------ 收包状态 ------------------------------ */

typedef struct {
    now_joystick_t pkt;
    uint32_t rx_seq;
    uint32_t lost;
    uint32_t rx_total;
    int8_t   rssi;
    int64_t  last_rx_us;
    uint32_t rx_cnt;
    uint32_t hz;
} rx_state_t;

static rx_state_t s_rx;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

#define LED_LEFT  0
#define LED_RIGHT 1
#define LED_BTN   2
static volatile int64_t s_led_until[3];

static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN];
static volatile bool s_peer_ok;
static volatile bool s_peer_new;
static const uint8_t s_bcast[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static volatile uint32_t s_tx_ok, s_tx_fail;

static void now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len < (int)sizeof(now_joystick_t)) {
        return;
    }
    now_joystick_t p;
    memcpy(&p, data, sizeof(p));
    if (now_unpack(&p) != 0) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    if (s_rx.rx_total > 0) {
        /* 和上一帧比：哪个变了就让对应颜色亮一会儿 */
        int64_t now = esp_timer_get_time();
        const now_joystick_t *q = &s_rx.pkt;
        if (abs((int)p.rz - q->rz) > LED_MOVE_TH || abs((int)p.z - q->z) > LED_MOVE_TH) {
            s_led_until[LED_LEFT] = now + LED_HOLD_MS * 1000LL;
        }
        if (abs((int)p.x - q->x) > LED_MOVE_TH || abs((int)p.y - q->y) > LED_MOVE_TH) {
            s_led_until[LED_RIGHT] = now + LED_HOLD_MS * 1000LL;
        }
        if (p.buttons != q->buttons) {
            s_led_until[LED_BTN] = now + LED_HOLD_MS * 1000LL;
        }
    }
    /* 序号倒退 = 发送板重启过了，计数从头来（否则 L 会一直累积到 999 下不来） */
    if (s_rx.rx_total > 0 && p.seq < s_rx.rx_seq) {
        ESP_LOGI(TAG, "发送板重启（seq %lu -> %lu），丢包计数清零",
                 (unsigned long)s_rx.rx_seq, (unsigned long)p.seq);
        s_rx.lost = 0;
        s_rx.rx_total = 0;
        s_rx.rx_seq = 0;
    }
    if (s_rx.rx_seq != 0 && p.seq > s_rx.rx_seq + 1) {
        s_rx.lost += p.seq - s_rx.rx_seq - 1;
    }
    s_rx.rx_seq = p.seq;
    s_rx.pkt = p;
    s_rx.rssi = info && info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;
    s_rx.last_rx_us = esp_timer_get_time();
    s_rx.rx_cnt++;
    s_rx.rx_total++;
    portEXIT_CRITICAL(&s_lock);

    if (info && info->src_addr &&
        (!s_peer_ok || memcmp(s_peer_mac, info->src_addr, ESP_NOW_ETH_ALEN) != 0)) {
        memcpy(s_peer_mac, info->src_addr, ESP_NOW_ETH_ALEN);
        s_peer_new = true;
    }
}

static void now_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_tx_ok++;
    } else {
        s_tx_fail++;
    }
}

static bool add_peer(const uint8_t *mac)
{
    esp_now_peer_info_t pi = {0};
    memcpy(pi.peer_addr, mac, ESP_NOW_ETH_ALEN);
    pi.channel = NOW_CHANNEL;
    pi.ifidx = ESP_IF_WIFI_STA;
    pi.encrypt = false;
    esp_err_t e = esp_now_add_peer(&pi);
    return e == ESP_OK || e == ESP_ERR_ESPNOW_EXIST;
}

/* ------------------------------ 回传状态 ------------------------------ */

static void status_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    uint32_t seq = 0;

    while (true) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(STATUS_PERIOD_MS));

        if (s_peer_new) {
            s_peer_new = false;
            if (s_peer_ok) {
                esp_now_del_peer(s_peer_mac);
            }
            s_peer_ok = add_peer(s_peer_mac);
            if (s_peer_ok) {
                ESP_LOGI(TAG, "发送板上线 %02x:%02x:%02x:%02x:%02x:%02x（改单播回传）",
                         s_peer_mac[0], s_peer_mac[1], s_peer_mac[2],
                         s_peer_mac[3], s_peer_mac[4], s_peer_mac[5]);
            }
        }

        now_status_t st = {0};
        portENTER_CRITICAL(&s_lock);
        st.rx_seq = s_rx.rx_seq;
        st.rx_total = s_rx.rx_total;
        st.lost = s_rx.lost;
        st.rssi = s_rx.rssi;
        st.rx_hz = (uint8_t)(s_rx.hz > 255 ? 255 : s_rx.hz);
        portEXIT_CRITICAL(&s_lock);
        st.flags = 0;
        now_status_pack(&st);

        const uint8_t *dst = s_peer_ok ? s_peer_mac : s_bcast;
        esp_now_send(dst, (const uint8_t *)&st, sizeof(st));
        seq++;
    }
}

/* ------------------------------ RGB 灯 ------------------------------ */
/* 左摇杆动=红，右摇杆动=绿，按键动=蓝；同时动就是叠加出来的颜色（三个都动=白）。
   静止 250 ms 之后全灭。 */
static void led_task(void *arg)
{
    int last = -1;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(40));
        int64_t now = esp_timer_get_time();
        int m = 0;
        for (int i = 0; i < 3; i++) {
            if (now < s_led_until[i]) {
                m |= 1 << i;
            }
        }
        if (m != last) {
            ws2812_set((m & (1 << LED_LEFT)) ? 255 : 0,
                       (m & (1 << LED_RIGHT)) ? 255 : 0,
                       (m & (1 << LED_BTN)) ? 255 : 0);
            last = m;
        }
    }
}

/* ------------------------------ 屏幕（可选） ------------------------------ */

static bool s_oled_ok;

static void display_task(void *arg)
{
    while (true) {
        if (!s_oled_ok) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        rx_state_t st;
        portENTER_CRITICAL(&s_lock);
        st = s_rx;
        portEXIT_CRITICAL(&s_lock);

        bool linked = st.last_rx_us != 0 &&
                      (esp_timer_get_time() - st.last_rx_us) < LINK_TIMEOUT_MS * 1000LL;

        if (!linked) {
            ui_message("NO LINK", "WAITING...");
        } else {
            ui_state_t u = {
                .rc2_on = (st.pkt.flags & NOW_FLAG_RC2_ON) != 0,
                .link_on = true,
                .x = st.pkt.x, .y = st.pkt.y, .z = st.pkt.z,
                .rz = st.pkt.rz, .dial = st.pkt.dial,
                .buttons = st.pkt.buttons,
                .rssi = st.rssi,
                .lost = st.lost,
                .pct = st.hz * 2 > 100 ? 100 : st.hz * 2,
            };
            ui_live(&u);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/* ------------------------------ 统计 ------------------------------ */

static void rate_task(void *arg)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        portENTER_CRITICAL(&s_lock);
        s_rx.hz = s_rx.rx_cnt;
        s_rx.rx_cnt = 0;
        rx_state_t st = s_rx;
        portEXIT_CRITICAL(&s_lock);

        uint32_t ok = s_tx_ok, fail = s_tx_fail;
        s_tx_ok = s_tx_fail = 0;

        ESP_LOGI(TAG, "rx=%luHz rssi=%d lost=%lu seq=%lu btn=0x%06lx rc2=%d "
                      "X=%d Y=%d Z=%d Rz=%d Dial=%d | status tx %lu/%lu peer=%d",
                 (unsigned long)st.hz, st.rssi, (unsigned long)st.lost,
                 (unsigned long)st.rx_seq, (unsigned long)st.pkt.buttons,
                 (st.pkt.flags & NOW_FLAG_RC2_ON) ? 1 : 0,
                 st.pkt.x, st.pkt.y, st.pkt.z, st.pkt.rz, st.pkt.dial,
                 (unsigned long)ok, (unsigned long)(ok + fail), s_peer_ok ? 1 : 0);
    }
}

/* ------------------------------ 初始化 ------------------------------ */

static void wifi_now_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(NOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "本机 STA MAC %02x:%02x:%02x:%02x:%02x:%02x  ch%d",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], NOW_CHANNEL);

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(now_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(now_send_cb));
    add_peer(s_bcast);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "NOW 接收板启动（%s）", esp_app_get_description()->version);

    if (ws2812_init(WS2812_GPIO_DEFAULT) == ESP_OK) {
        ws2812_set(0, 0, 0);
        ESP_LOGI(TAG, "RGB 灯就绪（GPIO%d），左摇杆=红 右摇杆=绿 按键=蓝", WS2812_GPIO_DEFAULT);
    } else {
        ESP_LOGW(TAG, "RGB 灯初始化失败，只是没有灯，不影响收包");
    }

    wifi_now_init();

    err = oled_init(OLED_SDA_GPIO, OLED_SCL_GPIO);
    if (err == ESP_OK) {
        s_oled_ok = true;
        ui_message("ESP-NOW RX", "WAITING...");
    } else {
        ESP_LOGI(TAG, "OLED 不在（%s），按无头接收盒跑", esp_err_to_name(err));
    }

    xTaskCreate(status_task, "status", 4096, NULL, 4, NULL);
    xTaskCreate(led_task, "led", 3072, NULL, 2, NULL);
    xTaskCreate(display_task, "disp", 4096, NULL, 3, NULL);
    xTaskCreate(rate_task, "rate", 3072, NULL, 2, NULL);

    ESP_LOGI(TAG, "ready, 等发送端发包");
}
