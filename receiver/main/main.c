/*
 * 摇杆接收板 —— ESP-NOW → 128x64 OLED
 *
 * 收到发送端（RC2 板）的 now_joystick_t，画成：
 *   顶部  链路状态（收包率 / RSSI / 丢包数）
 *   中间  左右两个摇杆盘（左盘竖轴是油门，没有回中弹簧）
 *   底部  24 个按键位，亮 = 按下；上排 bit0-11，下排 bit12-23
 *
 * 引脚（既定分配，见 hobby/AGENTS.md）：
 *   OLED SDA=GPIO17  SCL=GPIO18  VCC=3V3  GND=GND   地址 0x3C
 *   日志走 UART0（CH343 那个口，115200）——只有这个口能同时烧录和看日志。
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

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

#include "ssd1306.h"
#include "now_proto.h"
#include "ws2812_off.h"

static const char *TAG = "now_rx";

/* 两端必须同一个信道。家里 2.4G 全挤在 ch6，所以用 11（和 r3car 一致）。 */
#define NOW_CHANNEL 11

#define LINK_TIMEOUT_MS 1000   /* 超过这么久没包就算断链 */

#define WS2812_GPIO 48         /* 板载 RGB 灯（WS2812） */

/* ------------------------------ 收包状态 ------------------------------ */

typedef struct {
    now_joystick_t pkt;
    uint32_t rx_seq;        /* 本端收到的最新序号 */
    uint32_t lost;          /* 累计丢包（按序号跳变估） */
    int      rssi;
    int64_t  last_rx_us;
    uint32_t rx_cnt;        /* 本秒收包数 */
    uint32_t hz;            /* 上一秒的收包率 */
} rx_state_t;

static rx_state_t s_rx;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

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
    if (s_rx.rx_seq != 0 && p.seq > s_rx.rx_seq + 1) {
        s_rx.lost += p.seq - s_rx.rx_seq - 1;
    }
    s_rx.rx_seq = p.seq;
    s_rx.pkt = p;
    s_rx.rssi = info && info->rx_ctrl ? info->rx_ctrl->rssi : 0;
    s_rx.last_rx_us = esp_timer_get_time();
    s_rx.rx_cnt++;
    portEXIT_CRITICAL(&s_lock);
}

/* ------------------------------ 画界面 ------------------------------ */

#define STICK_R     15
#define STICK_LX    32
#define STICK_RX    96
#define STICK_CY    25
#define BTN_ROW1_Y  47
#define BTN_ROW2_Y  58

static void draw_stick(int cx, int cy, int ax, int ay)
{
    oled_circle(cx, cy, STICK_R);
    /* 中位十字，方便看"回中没有" */
    oled_hline(cx - 3, cy, 7);
    oled_vline(cx, cy - 3, 7);

    int dx = (int)((int32_t)ax * (STICK_R - 2) / NOW_AXIS_FULL);
    int dy = (int)((int32_t)ay * (STICK_R - 2) / NOW_AXIS_FULL);
    if (dx > STICK_R - 2) dx = STICK_R - 2;
    if (dx < -(STICK_R - 2)) dx = -(STICK_R - 2);
    if (dy > STICK_R - 2) dy = STICK_R - 2;
    if (dy < -(STICK_R - 2)) dy = -(STICK_R - 2);
    oled_disc(cx + dx, cy - dy, 2);   /* y 取负：原始"推上=正"，屏幕向下为正 */
}

static void draw_buttons(uint32_t buttons)
{
    for (int i = 0; i < 24; i++) {
        int col = i % 12;
        int row = i / 12;
        int cx = 8 + col * 10;
        int cy = row == 0 ? BTN_ROW1_Y : BTN_ROW2_Y;
        if (buttons & (1u << i)) {
            oled_disc(cx, cy, 3);
        } else {
            oled_pixel(cx, cy);
        }
    }
}

static void draw_screen(void)
{
    rx_state_t st;
    portENTER_CRITICAL(&s_lock);
    st = s_rx;
    portEXIT_CRITICAL(&s_lock);

    bool linked = st.last_rx_us != 0 &&
                  (esp_timer_get_time() - st.last_rx_us) < LINK_TIMEOUT_MS * 1000LL;

    oled_clear();

    char line[24];
    if (linked && !(st.pkt.flags & NOW_FLAG_RC2_ON)) {
        /* 链路通、但发送端还没认出 RC2 —— 这时别画摇杆，会误导 */
        oled_text_center(24, "LINK OK");
        oled_text_center(36, "RC2 NOT FOUND");
    } else if (linked) {
        uint32_t lost = st.lost > 9999 ? 9999 : st.lost;
        snprintf(line, sizeof(line), "%2luHZ %3dDB %+5d",
                 (unsigned long)st.hz, st.rssi, st.pkt.dial);
        oled_text(0, 0, line);
        snprintf(line, sizeof(line), "L%lu", (unsigned long)lost);
        oled_text(96, 0, line);
        draw_stick(STICK_LX, STICK_CY, st.pkt.rz, st.pkt.z);
        draw_stick(STICK_RX, STICK_CY, st.pkt.x, st.pkt.y);
        draw_buttons(st.pkt.buttons);
    } else {
        oled_text_center(24, "NO SIGNAL");
        snprintf(line, sizeof(line), "%2luHZ", (unsigned long)st.hz);
        oled_text_center(36, line);
    }

    esp_err_t err = oled_flush();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "oled flush: %s", esp_err_to_name(err));
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

        ESP_LOGI(TAG, "rx=%luHz rssi=%d lost=%lu seq=%lu btn=0x%06lx rc2=%d "
                      "X=%d Y=%d Z=%d Rz=%d Dial=%d",
                 (unsigned long)st.hz, st.rssi, (unsigned long)st.lost,
                 (unsigned long)st.rx_seq, (unsigned long)st.pkt.buttons,
                 (st.pkt.flags & NOW_FLAG_RC2_ON) ? 1 : 0,
                 st.pkt.x, st.pkt.y, st.pkt.z, st.pkt.rz, st.pkt.dial);
    }
}

static void display_task(void *arg)
{
    while (true) {
        draw_screen();          /* 400 kHz 下约 23 ms，一帧 */
        vTaskDelay(pdMS_TO_TICKS(30));
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
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));   /* 关省电，否则收包会成串丢 */
    ESP_ERROR_CHECK(esp_wifi_set_channel(NOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "本机 STA MAC %02x:%02x:%02x:%02x:%02x:%02x  ch%d",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], NOW_CHANNEL);

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(now_recv_cb));
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

    /* 板载 RGB 灯先灭掉：WS2812 断电才会自己灭，这里主动发一帧全 0 */
    if (ws2812_off(WS2812_GPIO) == ESP_OK) {
        ESP_LOGI(TAG, "RGB 灯已关（GPIO%d）", WS2812_GPIO);
    }

    wifi_now_init();

    err = oled_init(OLED_SDA_GPIO, OLED_SCL_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OLED 初始化失败: %s（检查 SDA=17 / SCL=18 / VCC / GND）",
                 esp_err_to_name(err));
        /* 屏不在也要让链路跑起来，串口照样能看到数据 */
    } else {
        oled_clear();
        oled_text_center(20, "ESP-NOW RX");
        oled_text_center(32, "WAITING...");
        oled_flush();
    }

    xTaskCreate(display_task, "disp", 4096, NULL, 4, NULL);
    xTaskCreate(rate_task, "rate", 3072, NULL, 2, NULL);

    ESP_LOGI(TAG, "ready, 等发送端发包");
}
