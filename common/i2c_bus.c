/*
 * i2c_bus.c —— 一条总线，OLED 和 MPU6050 共用
 */
#include "esp_log.h"
#include "i2c_bus.h"

static const char *TAG = "i2c";
static i2c_master_bus_handle_t s_bus;

esp_err_t i2c_bus_init(int sda, int scl)
{
    if (s_bus) {
        return ESP_OK;
    }
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "总线建不起来: %s", esp_err_to_name(err));
        s_bus = NULL;
        return err;
    }
    ESP_LOGI(TAG, "I2C 就绪 SDA=%d SCL=%d %dkHz", sda, scl, I2C_BUS_HZ / 1000);
    return ESP_OK;
}

i2c_master_bus_handle_t i2c_bus_get(void)
{
    return s_bus;
}
