/*
 * mpu_test —— 插在发送板上，逐个验手里那三片 MPU6050 哪片能用在倾斜控制上
 *
 * 接线（和 OLED 共用一条 I2C）：
 *   VCC=3V3  GND=GND  SDA=GPIO17  SCL=GPIO18  AD0=GND（地址 0x68）
 *
 * 它做三件事：
 *   1. 扫一遍 I2C，报总线上到底有哪些地址（OLED 应该是 0x3C）
 *   2. 读 WHO_AM_I（0x75，正常应回 0x68）——不改就说明是山寨片或者坏片
 *   3. 每秒 10 次打加速度/角速度，还有算出来的倾角，肉眼判断稳不稳
 *
 * 判断一片 6050 好不好，看这三条：
 *   - WHO_AM_I 是不是 0x68
 *   - 静止放着时，角速度是不是在 0 附近小幅抖（不是几百、不是死值）
 *   - 把板子朝不同方向转，ax/ay/az 是不是跟着变、正负对
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "mpu";

#define PIN_SDA   17
#define PIN_SCL   18
#define MPU_ADDR  0x68
#define OLED_ADDR 0x3C

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_mpu;

static esp_err_t rd(uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(s_mpu, &reg, 1, buf, n, 100);
}

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_mpu, b, 2, 100);
}

static void i2c_scan(void)
{
    printf("\n---- I2C 扫描（SDA=%d SCL=%d, 400kHz）----\n", PIN_SDA, PIN_SCL);
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_master_probe(s_bus, a, 20) == ESP_OK) {
            const char *what = "";
            if (a == OLED_ADDR) what = " <- OLED";
            else if (a == MPU_ADDR) what = " <- MPU6050";
            else if (a == 0x69) what = " <- MPU6050 (AD0 接高)";
            printf("  0x%02X%s\n", a, what);
            found++;
        }
    }
    if (!found) {
        printf("  （什么都没有：查 VCC/GND/线是否插到底）\n");
    }
    printf("---- 扫到 %d 个设备 ----\n\n", found);
}

void app_main(void)
{
    printf("\n\n==== MPU6050 选片测试 ====\n");

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_scan();

    if (i2c_master_probe(s_bus, MPU_ADDR, 100) != ESP_OK) {
        printf("0x68 上没东西。6050 没接好，或者 AD0 接到了 3V3（那就变成 0x69）。\n");
        while (1) vTaskDelay(pdMS_TO_TICKS(2000));
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_mpu));

    /* WHO_AM_I：读数应该是 0x68。连读 5 次，看稳不稳 */
    printf("---- WHO_AM_I（期望 0x68）----\n  ");
    for (int i = 0; i < 5; i++) {
        uint8_t who = 0;
        esp_err_t e = rd(0x75, &who, 1);
        if (e == ESP_OK) printf("0x%02X ", who);
        else printf("ERR(%s) ", esp_err_to_name(e));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    printf("\n\n");

    /* 退出睡眠、量程 ±2g / ±250dps、内部 8MHz 时钟（不用 DMP，够用） */
    wr(0x6B, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));
    wr(0x1C, 0x00);   /* ACCEL_CONFIG  ±2g   -> 16384 LSB/g */
    wr(0x1B, 0x00);   /* GYRO_CONFIG   ±250  -> 131 LSB/(°/s) */
    wr(0x19, 0x04);   /* SMPLRT_DIV    1kHz/(1+4) = 200 Hz */
    wr(0x1A, 0x03);   /* CONFIG        DLPF 44Hz */

    printf("时间     ax     ay     az  |  gx     gy     gz  |  俯仰   横滚\n");
    printf("        (g)    (g)    (g)  | (deg/s)          |  (度)   (度)\n");

    int64_t t0 = esp_timer_get_time();
    while (1) {
        uint8_t b[14];
        if (rd(0x3B, b, 14) == ESP_OK) {
            int16_t ax = (int16_t)((b[0] << 8) | b[1]);
            int16_t ay = (int16_t)((b[2] << 8) | b[3]);
            int16_t az = (int16_t)((b[4] << 8) | b[5]);
            int16_t gx = (int16_t)((b[8] << 8) | b[9]);
            int16_t gy = (int16_t)((b[10] << 8) | b[11]);
            int16_t gz = (int16_t)((b[12] << 8) | b[13]);

            float fx = ax / 16384.0f, fy = ay / 16384.0f, fz = az / 16384.0f;
            float pitch = atan2f(-fx, sqrtf(fy * fy + fz * fz)) * 57.2958f;
            float roll  = atan2f(fy, fz) * 57.2958f;

            printf("%5.1fs %6.2f %6.2f %6.2f | %6.1f %6.1f %6.1f | %6.1f %6.1f\n",
                   (esp_timer_get_time() - t0) / 1e6,
                   fx, fy, fz,
                   gx / 131.0f, gy / 131.0f, gz / 131.0f,
                   pitch, roll);
        } else {
            printf("读失败\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
