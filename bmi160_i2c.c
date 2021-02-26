/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bmi160/bmi160.h"

/* Example code to talk to a bmi160 MEMS accelerometer and gyroscope

   Connections on Raspberry Pi Pico board.

   GPIO 4 (pin 6)-> SDA on MPU6050 board
   GPIO 5 (pin 7)-> SCL on MPU6050 board
   3.3v (pin 36) -> VCC on MPU6050 board
   GND (pin 38)  -> GND on MPU6050 board
*/

// By default these devices  are on bus address 0x68
static int addr = 0x68;

#define I2C_PORT i2c0


int8_t bmi_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len) {
    uint8_t temp_buf[] = {reg_addr, *buf};
    i2c_write_blocking(I2C_PORT, addr, temp_buf, 2, false);
    return 0;
}
 
int8_t bmi_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *buf, uint16_t len) {
    i2c_write_blocking(I2C_PORT, addr, &reg_addr, 1, true); // true to keep master control of bus
    i2c_read_blocking(I2C_PORT, addr, buf, len, false);
    return 0;
}   

void bmi_delay(uint32_t period){
    sleep_ms(1);
}


int main() {
    stdio_init_all();

    // This example will use I2C0 on GPIO4 (SDA) and GPIO5 (SCL) running at 400kHz.
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);
    
 // -------------------------------------------
 // bmi160 init   
    struct bmi160_dev sensor;

    sensor.id = BMI160_I2C_ADDR;
    sensor.interface = BMI160_I2C_INTF;
    sensor.read = bmi_read;
    sensor.write = bmi_write;
    sensor.delay_ms = bmi_delay;


    int8_t rslt = BMI160_OK;
    rslt = bmi160_init(&sensor);
    printf("bmi160_init:%i\r\n", rslt);

// ---------------------------------------------

    printf("bmi160 over I2C...\n");
    
    uint8_t reg_addr = BMI160_CHIP_ID_ADDR;
    uint8_t data;
    uint16_t len = 1;
    rslt = bmi160_get_regs(reg_addr, &data, len, &sensor);
    printf("rslt = %d\n",rslt);
    printf("Chip ID = %X\n",data);
 
 //  --------------------------------------------
 // bmi set sensor conf   
    // Configuring accel and gyro sensor
    rslt = BMI160_OK;

    /* Select the Output data rate, range of accelerometer sensor */
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    
    /* Select the power mode of accelerometer sensor */
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    sensor.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    sensor.gyro_cfg.range = BMI160_GYRO_RANGE_500_DPS;
    sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&sensor);
    printf("Configuration rslt = %d\n",rslt);
  //  --------------------------------------------
     
     /*  Set the Power mode  */
    rslt = bmi160_set_power_mode(&sensor);
    printf("bmi160_set_power_mode:%i\r\n", rslt);
     
    rslt = BMI160_OK;
    uint8_t reg2_addr = BMI160_I2C_ADDR;
    uint8_t data2;
    uint16_t len2 = 1;
    rslt = bmi160_get_regs(reg2_addr, &data2, len2, &sensor);    
    printf("rslt = %d\n",rslt);
    printf("Chip I2C Address = %X\n",data);     
                
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;

    while (1) {
        /* To read only Gyro data */
        rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);
        printf("\rAcc_X = %i, Acc_Y = %i, Acc_Z = %i, Gyro_X = %i, Gyro_Y = %i, Gyro_Z = %i",
        accel.x,accel.y,accel.z,
        gyro.x,gyro.y,gyro.z);
  //      printf("Gyro. X = %i, Y = %i, Z = %i\n",gyro.x,gyro.y,gyro.z); 
 /*       printf("Acc. X = %i, Y = %i, Z = %i\n",accel.x,accel.y,accel.z);
        printf("Gyro. X = %i, Y = %i, Z = %i\n",gyro.x,gyro.y,gyro.z);
        printf("---------------------------\n");
   */             
        sleep_ms(500);
    }

    return 0;
}
