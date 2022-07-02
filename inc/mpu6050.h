#include<stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#define I2C_PORT i2c0

#define MPU6050_ADDRESS 0x68

void mpu6050_reset();
void mpu6050_read(int16_t accelerometer[3],int16_t gyro[3],int16_t *temp);