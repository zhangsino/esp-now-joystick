# 共享组件：协议定义 + 屏幕驱动 + 界面绘制 + RGB 灯
idf_component_register(SRCS "ssd1306.c" "font6x8.c" "ui.c" "ws2812.c" "i2c_bus.c" "mpu6050.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_i2c esp_driver_rmt esp_driver_gpio)
