/*
 * mpu6050.c
 *
 * 为什么不用陀螺：倾斜遥控只需要"往哪边偏多少"，加速度计受重力就能算出来，
 * 而且没有零漂。陀螺在这儿的唯一用处是快速姿态变化，但人手的动作本来就慢，
 * 加进来反而要处理零偏标定、温度漂移那一堆事。不做。
 *
 * 滤波用一阶低通（指数加权），不用"攒 N 个取平均"——后者滞后等于 N/2 个周期，
 * 手感会发粘。这里 50 Hz 采样、系数 0.2，等效带宽约 2 Hz，手推一下跟得上。
 */
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/i2c_master.h"

#include "i2c_bus.h"
#include "mpu6050.h"

static const char *TAG = "mpu6050";

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

#define LPF_ALPHA   0.20f   /* 一阶低通系数，越小越平滑越滞后 */
#define DEG(x)      ((x) * 57.29578f)

static i2c_master_dev_handle_t s_dev;
static float s_ax, s_ay, s_az;
static bool  s_primed;
static float s_pitch_zero, s_roll_zero;

static esp_err_t rd(uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, 100);
}

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_dev, b, 2, 100);
}

esp_err_t mpu_whoami(uint8_t *who)
{
    if (!s_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    return rd(REG_WHO_AM_I, who, 1);
}

esp_err_t mpu_init(void)
{
    i2c_master_bus_handle_t bus = i2c_bus_get();
    if (!bus) {
        ESP_LOGE(TAG, "共享 I2C 总线还没建");
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_dev) {
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = MPU6050_ADDR,
            .scl_speed_hz = I2C_BUS_HZ,
        };
        esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &s_dev);
        if (err != ESP_OK) {
            return err;
        }
    }

    /* 上电后要一会儿才应答，探 5 次 */
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 5; i++) {
        err = i2c_master_probe(bus, MPU6050_ADDR, 100);
        if (err == ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "0x68 无应答（查 VCC=3V3 / GND / SDA=17 / SCL=18 / AD0=GND）");
        return err;
    }

    uint8_t who = 0;
    rd(REG_WHO_AM_I, &who, 1);
    if (who != 0x68) {
        ESP_LOGW(TAG, "WHO_AM_I = 0x%02X（不是 0x68，可能是仿片，先按能用处理）", who);
    }

    wr(REG_PWR_MGMT_1, 0x00);            /* 退出睡眠 */
    vTaskDelay(pdMS_TO_TICKS(50));
    wr(REG_ACCEL_CONFIG, 0x00);          /* ±2g，16384 LSB/g */
    wr(REG_GYRO_CONFIG, 0x00);           /* 不用陀螺，顺手设了 */
    wr(REG_SMPLRT_DIV, 0x04);            /* 200 Hz 输出 */
    wr(REG_CONFIG, 0x03);                /* DLPF 44 Hz */

    s_primed = false;
    s_pitch_zero = s_roll_zero = 0.0f;
    ESP_LOGI(TAG, "就绪（WHO_AM_I=0x%02X）", who);
    return ESP_OK;
}

esp_err_t mpu_read_accel(float *ax, float *ay, float *az)
{
    if (!s_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t b[6];
    esp_err_t err = rd(REG_ACCEL_XOUT_H, b, 6);
    if (err != ESP_OK) {
        return err;
    }
    float x = (int16_t)((b[0] << 8) | b[1]) / 16384.0f;
    float y = (int16_t)((b[2] << 8) | b[3]) / 16384.0f;
    float z = (int16_t)((b[4] << 8) | b[5]) / 16384.0f;

    if (!s_primed) {
        s_ax = x; s_ay = y; s_az = z;
        s_primed = true;
    } else {
        s_ax += LPF_ALPHA * (x - s_ax);
        s_ay += LPF_ALPHA * (y - s_ay);
        s_az += LPF_ALPHA * (z - s_az);
    }
    if (ax) *ax = s_ax;
    if (ay) *ay = s_ay;
    if (az) *az = s_az;
    return ESP_OK;
}

void mpu_get_angles(float *pitch, float *roll)
{
    /* 平端（屏幕朝上、元件面朝上）时 az≈1g，ax/ay≈0 */
    float p = DEG(atan2f(-s_ax, sqrtf(s_ay * s_ay + s_az * s_az))) - s_pitch_zero;
    float r = DEG(atan2f(s_ay, s_az)) - s_roll_zero;
    if (pitch) *pitch = p;
    if (roll)  *roll  = r;
}

void mpu_zero_here(void)
{
    float p = DEG(atan2f(-s_ax, sqrtf(s_ay * s_ay + s_az * s_az)));
    float r = DEG(atan2f(s_ay, s_az));
    s_pitch_zero = p;
    s_roll_zero = r;
    ESP_LOGI(TAG, "零点标定：pitch=%.1f roll=%.1f", p, r);
}
