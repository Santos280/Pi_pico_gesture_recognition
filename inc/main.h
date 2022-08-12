#include<stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "mpu6050.h"
#include "ei_run_classifier.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "../pico-ssd1306/ssd1306.h"
#include "../pico-ssd1306/textRenderer/TextRenderer.h"
#include "OLED_bitmap.h"

#define SCL 1
#define SDA 0
#define CONVERT_G_TO_MS2    9.80665f

#define SSD1306_I2C     i2c1
#define SSD1306_SCL     3
#define SSD1306_SDA     2