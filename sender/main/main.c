/*
 * 摇杆发送板 —— DJI RC2 (USB HID) → ESP-NOW 广播
 *
 * USB 这半边是从 ~/esp/rc2_web 搬过来的、已在板上跑通（247 Hz，13 字节报告），
 * 只砍掉网页/WebSocket，换成每 20 ms 广播一包 now_joystick_t。
 *
 * 关键坑（已在 rc2_web 里解决，别改回去）：
 *   RC2 的 HID 中断 IN 端点把 wMaxPacketSize 写成 512（全速上限 64），
 *   ESP32-S3 的 DWC FIFO 上限 408，占用接口会报 ESP_ERR_NOT_SUPPORTED。
 *   必须在 hid_host_device_open 之前把描述符里的这个值钳到 64。
 *
 * 引脚：RC2 插板子的 USB（OTG）口，也就是 GPIO19/20 那个口；
 *      日志走 UART0（CH343 那个口，115200）——两个口别插反。
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
#include "ws2812_off.h"

static const char *TAG = "now_tx";

#define NOW_CHANNEL 11
#define REPORT_LEN  13
#define SEND_PERIOD_MS 20      /* 50 Hz：够顺，又不会把 2.4G 占满 */

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

/* 没有 RC2 时按 k 发一路模拟扫动，用来单独验接收板和屏 */
static volatile bool s_sim;

/* ------------------------ 修 RC2 的坏描述符 ------------------------ */
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
        int16_t dl = (int16_t)(p[11] | (p[12] << 8));   /* 第 5 轴 = 右侧拨盘 */
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

static const uint8_t s_bcast[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static volatile uint32_t s_tx_fail;

static void now_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    if (status != ESP_NOW_SEND_SUCCESS) {
        s_tx_fail++;
    }
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
    ESP_ERROR_CHECK(esp_now_register_send_cb(now_send_cb));

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_bcast, ESP_NOW_ETH_ALEN);
    peer.channel = NOW_CHANNEL;
    peer.ifidx = ESP_IF_WIFI_STA;
    peer.encrypt = false;
    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGW(TAG, "add broadcast peer: %s（广播仍可发）", esp_err_to_name(err));
    }
}

/* ------------------------------ 发送 ------------------------------ */

static void sim_sample(now_joystick_t *p)
{
    /* 8 秒一个三角波周期，四根轴错开相位；按键每 0.5 秒换个位 */
    int32_t t = (int32_t)(esp_timer_get_time() / 1000 % 8000);
    int32_t tri = (t < 4000) ? (t * 2 - 4000) : (12000 - t * 2);   /* -4000..4000 */
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

        esp_now_send(s_bcast, (const uint8_t *)&p, sizeof(p));
    }
}

/* ------------------------------ 模拟模式（BOOT 键） ------------------------------ */
/* 没插 RC2 时按一下 BOOT 键（GPIO0）发一路模拟扫动，用来单独验接收板和屏。
 * 不用串口输入：默认控制台没装 UART 驱动，uart_read_bytes 会一直报 driver error。 */
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
    uint32_t last_fail = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t c = s_rate_cnt;
        s_rate_cnt = 0;

        portENTER_CRITICAL(&s_lock);
        s_state.hz = c;
        rc2_state_t st = s_state;
        portEXIT_CRITICAL(&s_lock);

        uint32_t fail = s_tx_fail;
        ESP_LOGI(TAG, "rc2=%d hz=%lu X=%d Y=%d Z=%d Rz=%d Dial=%d btn=0x%06lx 发送失败+%lu%s",
                 st.connected ? 1 : 0, (unsigned long)c,
                 st.x, st.y, st.z, st.rz, st.dial, (unsigned long)st.buttons,
                 (unsigned long)(fail - last_fail), s_sim ? "  [SIM]" : "");
        last_fail = fail;
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

    if (ws2812_off(48) == ESP_OK) {
        ESP_LOGI(TAG, "RGB 灯已关（GPIO48）");
    }

    wifi_now_init();

    xTaskCreate(usb_task, "usb_task", 4096, NULL, 4, NULL);
    xTaskCreate(send_task, "send", 4096, NULL, 5, NULL);
    xTaskCreate(rate_task, "rate", 3072, NULL, 2, NULL);
    xTaskCreate(btn_task, "btn", 3072, NULL, 2, NULL);

    ESP_LOGI(TAG, "ready: 把 RC2 插到板子的 USB(OTG) 口；没插 RC2 就按板上 BOOT 键切模拟数据");
}
