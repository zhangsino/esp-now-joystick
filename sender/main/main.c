/*
 * 摇杆发送板（手持端）—— DJI RC2 (USB Host) → 小屏 + ESP-NOW 广播
 *
 * 这版把屏挪到发送板上：一插上 RC2，屏上立刻打 "RC2 CONNECTED"，
 * 之后显示实时画面（摇杆盘 + 24 个按键位 + 链路质量）。
 *
 * 链路是**双向**的：接收板每 200 ms 回一个状态包（它自己的收包率/信号/丢包），
 * 发送板收到就说明"我发的包被收到了"，同时把对端的 MAC 学下来，改成单播发。
 *
 * USB 这半边沿用 ~/esp/rc2_web 里已验证的代码（247 Hz，13 字节报告），只换了出口。
 * 引脚：RC2 插 USB(OTG) 口（GPIO19/20）；日志走 UART0（CH343）；OLED 走 I2C 17/18。
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
#include "driver/gpio.h"

#include "usb/usb_host.h"
#include "usb/hid_host.h"

#include "now_proto.h"
#include "ssd1306.h"
#include "ui.h"
#include "ws2812.h"

static const char *TAG = "now_tx";

#define NOW_CHANNEL     11
#define REPORT_LEN      13
#define SEND_PERIOD_MS  20      /* 50 Hz */
#define PEER_TIMEOUT_MS 1500    /* 这么久没收到状态包就算对面没了 */
#define SPLASH_MS       2000    /* "RC2 CONNECTED" 停留时间 */

/* ------------------------------ RC2 状态 ------------------------------ */

typedef struct {
    int16_t  x, y, z, rz, dial;
    uint32_t buttons;
    bool     connected;
    uint32_t hz;
} rc2_state_t;

static rc2_state_t s_state;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_rate_cnt;
static volatile bool s_sim;

/* ------------------------------ 链路状态 ------------------------------ */

static uint8_t s_peer_mac[ESP_NOW_ETH_ALEN];
static volatile bool s_peer_ok;
static volatile bool s_peer_new;
static volatile int8_t  s_peer_rssi;
static volatile uint32_t s_peer_lost;
static volatile int64_t s_peer_last_us;
static volatile uint32_t s_tx_ok, s_tx_fail;
static volatile uint32_t s_tx_pct;

static const uint8_t s_bcast[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static bool peer_alive(void)
{
    int64_t t = s_peer_last_us;
    return t != 0 && (esp_timer_get_time() - t) < PEER_TIMEOUT_MS * 1000LL;
}

static void now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (!info || !info->src_addr || len < (int)sizeof(now_status_t)) {
        return;
    }
    now_status_t p;
    memcpy(&p, data, sizeof(p));
    if (now_status_unpack(&p) != 0) {
        return;
    }

    if (!s_peer_ok || memcmp(s_peer_mac, info->src_addr, ESP_NOW_ETH_ALEN) != 0) {
        memcpy(s_peer_mac, info->src_addr, ESP_NOW_ETH_ALEN);
        s_peer_new = true;
    }
    s_peer_rssi = p.rssi;
    s_peer_lost = p.lost;
    s_peer_last_us = esp_timer_get_time();
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

/* ------------------------------ USB HID ------------------------------ */

static usb_host_client_handle_t s_client;

static int clamp_intr_ep_mps(usb_config_desc_t *cfg)
{
    uint8_t *p = (uint8_t *)cfg;
    uint8_t *end = p + cfg->wTotalLength;
    int fixed = 0;

    while (p + 2 <= end) {
        uint8_t len = p[0];
        uint8_t type = p[1];
        if (len < 2 || p + len > end) {
            break;
        }
        if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT && len >= 7) {
            usb_ep_desc_t *ep = (usb_ep_desc_t *)p;
            if ((ep->bmAttributes & 0x03) == USB_TRANSFER_TYPE_INTR && ep->wMaxPacketSize > 64) {
                ESP_LOGW(TAG, "clamp EP 0x%02X wMaxPacketSize %u -> 64",
                         ep->bEndpointAddress, ep->wMaxPacketSize);
                ep->wMaxPacketSize = 64;
                fixed++;
            }
        }
        p += len;
    }
    return fixed;
}

static void patch_device_descriptor(uint8_t addr)
{
    usb_device_handle_t dev = NULL;
    if (usb_host_device_open(s_client, addr, &dev) != ESP_OK) {
        ESP_LOGW(TAG, "device_open(%u) for patching failed", addr);
        return;
    }
    const usb_config_desc_t *cfg = NULL;
    if (usb_host_get_active_config_descriptor(dev, &cfg) == ESP_OK && cfg) {
        clamp_intr_ep_mps((usb_config_desc_t *)cfg);
    } else {
        ESP_LOGW(TAG, "get_active_config_descriptor failed");
    }
    usb_host_device_close(s_client, dev);
}

static void client_event_cb(const usb_host_client_event_msg_t *event, void *arg)
{
    (void)event;
    (void)arg;
}

static void client_task(void *arg)
{
    while (true) {
        usb_host_client_handle_events(s_client, pdMS_TO_TICKS(10));
    }
}

static void hid_interface_cb(hid_host_device_handle_t hid_dev,
                             const hid_host_interface_event_t event, void *arg)
{
    uint8_t buf[64];
    size_t len = 0;

    switch (event) {
    case HID_HOST_INTERFACE_EVENT_INPUT_REPORT: {
        if (hid_host_device_get_raw_input_report_data(hid_dev, buf, sizeof(buf), &len) != ESP_OK) {
            return;
        }
        if (len < REPORT_LEN) {
            return;
        }
        const uint8_t *p = (len == REPORT_LEN + 1) ? buf + 1 : buf;

        int16_t x  = (int16_t)(p[3] | (p[4] << 8));
        int16_t y  = (int16_t)(p[5] | (p[6] << 8));
        int16_t z  = (int16_t)(p[7] | (p[8] << 8));
        int16_t rz = (int16_t)(p[9] | (p[10] << 8));
        int16_t dl = (int16_t)(p[11] | (p[12] << 8));
        uint32_t b = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);

        portENTER_CRITICAL(&s_lock);
        s_state.x = x;
        s_state.y = y;
        s_state.z = z;
        s_state.rz = rz;
        s_state.dial = dl;
        s_state.buttons = b;
        portEXIT_CRITICAL(&s_lock);

        s_rate_cnt++;
        break;
    }
    case HID_HOST_INTERFACE_EVENT_TRANSFER_ERROR:
        ESP_LOGW(TAG, "HID transfer error");
        break;
    case HID_HOST_INTERFACE_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "RC2 拔出");
        portENTER_CRITICAL(&s_lock);
        s_state.connected = false;
        portEXIT_CRITICAL(&s_lock);
        hid_host_device_close(hid_dev);
        break;
    default:
        break;
    }
}

static void hid_driver_cb(hid_host_device_handle_t hid_dev,
                          const hid_host_driver_event_t event, void *arg)
{
    if (event != HID_HOST_DRIVER_EVENT_CONNECTED) {
        return;
    }

    hid_host_dev_params_t params = {0};
    if (hid_host_device_get_params(hid_dev, &params) == ESP_OK) {
        ESP_LOGI(TAG, "HID 接口: addr=%u iface=%u subclass=%u proto=%u",
                 params.addr, params.iface_num, params.sub_class, params.proto);
    }
    hid_host_dev_info_t info = {0};
    if (hid_host_get_device_info(hid_dev, &info) == ESP_OK) {
        ESP_LOGI(TAG, "VID=0x%04X PID=0x%04X", info.VID, info.PID);
    }

    patch_device_descriptor(params.addr);

    const hid_host_device_config_t dev_cfg = {
        .callback = hid_interface_cb,
        .callback_arg = NULL,
    };
    esp_err_t err = hid_host_device_open(hid_dev, &dev_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hid_host_device_open: %s", esp_err_to_name(err));
        return;
    }
    err = hid_host_device_start(hid_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "hid_host_device_start: %s", esp_err_to_name(err));
        hid_host_device_close(hid_dev);
        return;
    }

    portENTER_CRITICAL(&s_lock);
    s_state.connected = true;
    portEXIT_CRITICAL(&s_lock);
    ESP_LOGI(TAG, "RC2 就绪，开始出报告");
}

static void usb_task(void *arg)
{
    const usb_host_config_t host_cfg = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));

    const usb_host_client_config_t client_cfg = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = NULL,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_cfg, &s_client));
    xTaskCreate(client_task, "usb_client", 4096, NULL, 5, NULL);

    const hid_host_driver_config_t hid_cfg = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = tskNO_AFFINITY,
        .callback = hid_driver_cb,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(hid_host_install(&hid_cfg));

    while (true) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(pdMS_TO_TICKS(1000), &flags);
    }
}

/* ------------------------------ ESP-NOW ------------------------------ */

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
    add_peer(s_bcast);   /* 还没学到接收板之前先广播 */
}

/* ------------------------------ 发送 ------------------------------ */

static void sim_sample(now_joystick_t *p)
{
    int32_t t = (int32_t)(esp_timer_get_time() / 1000 % 8000);
    int32_t tri = (t < 4000) ? (t * 2 - 4000) : (12000 - t * 2);
    p->x  = (int16_t)(tri / 8);
    p->rz = (int16_t)((t < 4000 ? -4000 : 4000) - tri / 2);
    p->y  = (int16_t)(tri / 6);
    p->z  = (int16_t)(tri / 4);
    p->dial = (int16_t)(tri / 10);
    p->buttons = 1u << ((esp_timer_get_time() / 500000) % 24);
    p->flags = NOW_FLAG_RC2_ON;
}

static void send_task(void *arg)
{
    uint32_t seq = 0;
    TickType_t last = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(SEND_PERIOD_MS));

        /* 学到新的接收板就登记成单播对端：这样 esp_now_send 的回调才代表"对方收到了" */
        if (s_peer_new) {
            s_peer_new = false;
            if (s_peer_ok) {
                esp_now_del_peer(s_peer_mac);
            }
            s_peer_ok = add_peer(s_peer_mac);
            if (s_peer_ok) {
                ESP_LOGI(TAG, "接收板上线 %02x:%02x:%02x:%02x:%02x:%02x（改单播）",
                         s_peer_mac[0], s_peer_mac[1], s_peer_mac[2],
                         s_peer_mac[3], s_peer_mac[4], s_peer_mac[5]);
            }
        }

        now_joystick_t p = {0};
        rc2_state_t st;
        portENTER_CRITICAL(&s_lock);
        st = s_state;
        portEXIT_CRITICAL(&s_lock);

        p.x = st.x;
        p.y = st.y;
        p.z = st.z;
        p.rz = st.rz;
        p.dial = st.dial;
        p.buttons = st.buttons;
        p.flags = st.connected ? NOW_FLAG_RC2_ON : 0;
        if (!st.connected && s_sim) {
            sim_sample(&p);
        }
        p.seq = ++seq;
        p.rc2_hz = (uint8_t)(st.hz > 255 ? 255 : st.hz);
        now_pack(&p);

        const uint8_t *dst = (s_peer_ok && peer_alive()) ? s_peer_mac : s_bcast;
        esp_now_send(dst, (const uint8_t *)&p, sizeof(p));
    }
}

/* ------------------------------ 屏幕 ------------------------------ */

static bool s_oled_ok;

static void display_task(void *arg)
{
    bool was_on = false;
    int64_t splash_until = 0;

    while (true) {
        if (!s_oled_ok) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        rc2_state_t st;
        portENTER_CRITICAL(&s_lock);
        st = s_state;
        portEXIT_CRITICAL(&s_lock);

        if (st.connected && !was_on) {
            splash_until = esp_timer_get_time() + SPLASH_MS * 1000LL;
            ESP_LOGI(TAG, "屏：RC2 CONNECTED");
        }
        was_on = st.connected;

        if (!st.connected) {
            ui_message("PLUG IN RC2", "WAITING...");
        } else if (esp_timer_get_time() < splash_until) {
            ui_message("RC2 CONNECTED", "USB HOST OK");
        } else {
            ui_state_t u = {
                .rc2_on = true,
                .link_on = peer_alive(),
                .x = st.x, .y = st.y, .z = st.z, .rz = st.rz, .dial = st.dial,
                .buttons = st.buttons,
                .rssi = s_peer_rssi,
                .lost = s_peer_lost,
                .pct  = s_tx_pct,
            };
            ui_live(&u);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/* ------------------------------ 模拟模式 ------------------------------ */

static void btn_task(void *arg)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    int last = 1;
    while (true) {
        int now = gpio_get_level(0);
        if (last == 1 && now == 0) {
            s_sim = !s_sim;
            ESP_LOGW(TAG, "模拟摇杆 %s（BOOT 键切换）", s_sim ? "开" : "关");
        }
        last = now;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ------------------------------ 统计 ------------------------------ */

static void rate_task(void *arg)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t c = s_rate_cnt;
        s_rate_cnt = 0;

        portENTER_CRITICAL(&s_lock);
        s_state.hz = c;
        rc2_state_t st = s_state;
        portEXIT_CRITICAL(&s_lock);

        uint32_t ok = s_tx_ok, fail = s_tx_fail;
        s_tx_ok = s_tx_fail = 0;
        uint32_t tot = ok + fail;
        s_tx_pct = tot ? (ok * 100 / tot) : 0;

        /* 没有对端时是广播，esp_now 的发送回调永远报成功，那个百分比是假的，别显示 */
        char txbuf[16];
        if (s_peer_ok) {
            snprintf(txbuf, sizeof(txbuf), "%lu%%", (unsigned long)s_tx_pct);
        } else {
            snprintf(txbuf, sizeof(txbuf), "广播");
        }
        ESP_LOGI(TAG, "rc2=%d hz=%lu X=%d Y=%d Z=%d Rz=%d Dial=%d btn=0x%06lx "
                      "tx=%s peer=%d rssi=%d lost=%lu%s",
                 st.connected ? 1 : 0, (unsigned long)c,
                 st.x, st.y, st.z, st.rz, st.dial, (unsigned long)st.buttons,
                 txbuf, s_peer_ok ? 1 : 0, s_peer_rssi, (unsigned long)s_peer_lost,
                 s_sim ? "  [SIM]" : "");
    }
}

/* ------------------------------ main ------------------------------ */

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "NOW 发送板启动（%s）", esp_app_get_description()->version);

    if (ws2812_init(WS2812_GPIO_DEFAULT) == ESP_OK) {
        ws2812_set(0, 0, 0);
        ESP_LOGI(TAG, "RGB 灯已关（GPIO%d）", WS2812_GPIO_DEFAULT);
    }

    wifi_now_init();

    err = oled_init(OLED_SDA_GPIO, OLED_SCL_GPIO);
    if (err == ESP_OK) {
        s_oled_ok = true;
        ui_message("ESP-NOW TX", "WAITING...");
    } else {
        ESP_LOGW(TAG, "OLED 不在（%s）：屏没接也能跑，日志里照样有数据",
                 esp_err_to_name(err));
    }

    xTaskCreate(usb_task, "usb_task", 4096, NULL, 4, NULL);
    xTaskCreate(send_task, "send", 4096, NULL, 5, NULL);
    xTaskCreate(display_task, "disp", 4096, NULL, 3, NULL);
    xTaskCreate(rate_task, "rate", 3072, NULL, 2, NULL);
    xTaskCreate(btn_task, "btn", 3072, NULL, 2, NULL);

    ESP_LOGI(TAG, "ready: RC2 插 USB(OTG) 口；OLED 接 SDA17/SCL18");
}
