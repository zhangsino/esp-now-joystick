/*
 * mpu6050.h —— 倾斜控制用的最小驱动
 *
 * 只用加速度计算俯仰/横滚，不用陀螺、不用 DMP：
 * 手持倾斜控制要的是"往哪边偏"，不是高精度姿态，加速度计够用而且没有漂移。
 */
#ifndef MPU6050_H
#define MPU6050_H

#include "esp_err.h"

#define MPU6050_ADDR 0x68

/* 用已经建好的共享 I2C 总线（i2c_bus_get()），地址 0x68 */
esp_err_t mpu_init(void);
esp_err_t mpu_whoami(uint8_t *who);

/* 读加速度（单位 g），已经做过低通 */
esp_err_t mpu_read_accel(float *ax, float *ay, float *az);

/* 倾角（度）：pitch 前后点头，roll 左右侧倾 */
void mpu_get_angles(float *pitch, float *roll);

/* 把当前姿态设成零点（开机/按开关时调一次） */
void mpu_zero_here(void);

#endif
