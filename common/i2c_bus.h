/*
 * i2c_bus.h —— 板子上唯一的一条 I2C 总线
 *
 * 以前这条总线是 ssd1306.c 自己管的，现在 6050 也要挂上来，所以抽出来：
 * 谁先要用谁初始化，之后大家共用同一个 handle。
 *
 * 引脚固定 SDA=17 / SCL=18（和 hobby/AGENTS.md 的既定分配一致），400 kHz。
 */
#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "driver/i2c_master.h"
#include "esp_err.h"

#define I2C_BUS_SDA   17
#define I2C_BUS_SCL   18
#define I2C_BUS_HZ    400000

esp_err_t i2c_bus_init(int sda, int scl);
i2c_master_bus_handle_t i2c_bus_get(void);

#endif
