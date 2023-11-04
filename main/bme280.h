#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           22                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           23                         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// device has default bus address of 0x77
#define bme280_ADDR 0x77

// hardware registers
#define REG_CONFIG 0xF5
#define REG_CTRL_MEAS 0xF4
#define REG_RESET 0xE0

#define REG_TEMP_XLSB 0xFC
#define REG_TEMP_LSB 0xFB
#define REG_TEMP_MSB 0xFA

#define REG_PRESSURE_XLSB 0xF9
#define REG_PRESSURE_LSB 0xF8
#define REG_PRESSURE_MSB 0xF7

// calibration registers
#define REG_DIG_T1_LSB 0x88
#define REG_DIG_T1_MSB 0x89
#define REG_DIG_T2_LSB 0x8A
#define REG_DIG_T2_MSB 0x8B
#define REG_DIG_T3_LSB 0x8C
#define REG_DIG_T3_MSB 0x8D
#define REG_DIG_P1_LSB 0x8E
#define REG_DIG_P1_MSB 0x8F
#define REG_DIG_P2_LSB 0x90
#define REG_DIG_P2_MSB 0x91
#define REG_DIG_P3_LSB 0x92
#define REG_DIG_P3_MSB 0x93
#define REG_DIG_P4_LSB 0x94
#define REG_DIG_P4_MSB 0x95
#define REG_DIG_P5_LSB 0x96
#define REG_DIG_P5_MSB 0x97
#define REG_DIG_P6_LSB 0x98
#define REG_DIG_P6_MSB 0x99
#define REG_DIG_P7_LSB 0x9A
#define REG_DIG_P7_MSB 0x9B
#define REG_DIG_P8_LSB 0x9C
#define REG_DIG_P8_MSB 0x9D
#define REG_DIG_P9_LSB 0x9E
#define REG_DIG_P9_MSB 0x9F

// number of calibration registers to be read
#define NUM_CALIB_PARAMS 24

#define bme280_SDA_PIN 8
#define bme280_SCL_PIN 9

struct bme280_calib_param {
    // temperature params
    uint16_t dig_t1;
    int16_t dig_t2;
    int16_t dig_t3;

    // pressure params
    uint16_t dig_p1;
    int16_t dig_p2;
    int16_t dig_p3;
    int16_t dig_p4;
    int16_t dig_p5;
    int16_t dig_p6;
    int16_t dig_p7;
    int16_t dig_p8;
    int16_t dig_p9;
};

void bme280_init();
void bme280_read_raw(int32_t* temp, int32_t* pressure);
void bme280_reset();
int32_t bme280_convert(int32_t temp, struct bme280_calib_param* params);
int32_t bme280_convert_temp(int32_t temp, struct bme280_calib_param* params);
int32_t bme280_convert_pressure(int32_t pressure, int32_t temp, struct bme280_calib_param* params);
void bme280_get_calib_params(struct bme280_calib_param* params);