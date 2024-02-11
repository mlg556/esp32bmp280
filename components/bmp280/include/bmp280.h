/*
 * BMP280 Pressure/Temperature Sensor I2C Driver
 * inspired from
 * https://github.com/adafruit/Adafruit_BMP280_Library
 * Mirac L. Gulgonul - November 2023
 */

/*
    ## ctrl_meas register

    osrs_t is Temperature oversampling:
    000 |Skipped (output set to 0x80000)
    001 |oversampling ×1
    010 |oversampling ×2
    011 |oversampling ×4
    100 |oversampling ×8
    101, 110, 111 |oversampling ×16

    osrs_p is pressure oversampling, same values

    mode is power mode:
    00    | sleep
    01,10 | forced
    11    | normal

    ctrl_meas register is:
    |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
    |   osrs_t[2:0]   |   osrs_p[2:0]   | mode[1:0] |

    for weather monitoring, the datasheet (p.19) recommends
    mode = forced, osrs_p = 1x, osrs_t = 1x, filter = off, timing = 1/min, ODR (Hz) = 1/60, BW = full

    thus ctrl_meas can be:
    001  |  001   | 01

    ## config register
    |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |
    |   t_sb[2:0]     |   filter[2:0]   |  X  |  spi3w_en

    t_sb is standby time, which only matters in NORMAL mode, (I think)
    b  |  ms
    000 | 0.5
    001 | 62.5
    010 | 125
    011 | 250
    100 | 500
    101 | 1000
    110 | 2000
    111 | 400

    filter is filter coefficient.
    spi3w_en is spi enable. we are using i2c, so set it to 00.

*/

#ifndef BMP280_I2C_DRIVER_H
#define BMP280_I2C_DRIVER_H

// needed for i2c
#include <stdint.h>

#include "driver/i2c.h"
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include "stdbool.h"

// conversion constant from Pascal to ATM
#define BMP280_CONV_PA_ATM 9.86923267e-6

enum {
    // values
    BMP280_ADDRESS = 0x76,
    BMP280_CHIPID  = 0x58,  // expected chip id
    BMP280_RESET   = 0xB6,

    // registers
    BMP280_REG_DIG_T1    = 0x88,
    BMP280_REG_DIG_T2    = 0x8A,
    BMP280_REG_DIG_T3    = 0x8C,
    BMP280_REG_DIG_P1    = 0x8E,
    BMP280_REG_DIG_P2    = 0x90,
    BMP280_REG_DIG_P3    = 0x92,
    BMP280_REG_DIG_P4    = 0x94,
    BMP280_REG_DIG_P5    = 0x96,
    BMP280_REG_DIG_P6    = 0x98,
    BMP280_REG_DIG_P7    = 0x9A,
    BMP280_REG_DIG_P8    = 0x9C,
    BMP280_REG_DIG_P9    = 0x9E,
    BMP280_REG_CHIPID    = 0xD0,
    BMP280_REG_SOFTRESET = 0xE0,
    BMP280_REG_STATUS    = 0xF3,
    BMP280_REG_CONTROL   = 0xF4,
    BMP280_REG_CONFIG    = 0xF5,
    BMP280_REG_PRESSURE  = 0xF7,

    // i2c timeout
    BMP280_TIMEOUT = 10,

};

// configuration types for oversampling, mode, standby time

// temperature oversampling: 0x(disabled), 1x, 2x, 4x, 8x, 16x
typedef enum {
    BMP280_OVRSMP_TEMP_0X  = 0b00000000,
    BMP280_OVRSMP_TEMP_1X  = 0b00100000,
    BMP280_OVRSMP_TEMP_2X  = 0b01000000,
    BMP280_OVRSMP_TEMP_4X  = 0b01100000,
    BMP280_OVRSMP_TEMP_8X  = 0b10000000,
    BMP280_OVRSMP_TEMP_16X = 0b10100000,
} BMP280_OVRSMP_TEMP;

// pressure oversampling: 0x(disabled), 1x, 2x, 4x, 8x, 16x
typedef enum {
    BMP280_OVRSMP_PRES_0X  = 0b00000000,
    BMP280_OVRSMP_PRES_1X  = 0b00000100,
    BMP280_OVRSMP_PRES_2X  = 0b00001000,
    BMP280_OVRSMP_PRES_4X  = 0b00001100,
    BMP280_OVRSMP_PRES_8X  = 0b00010000,
    BMP280_OVRSMP_PRES_16X = 0b00010100,
} BMP280_OVRSMP_PRES;

// operation mode: sleep, forced or normal
typedef enum {
    BMP280_MODE_SLEEP  = 0b00000000,
    BMP280_MODE_FORCED = 0b00000001,
    BMP280_MODE_NORMAL = 0b00000011,
} BMP280_MODE;

// standby period, in milliseconds
typedef enum {
    BMP280_STANDBY_05   = 0b00000000,
    BMP280_STANDBY_62   = 0b00100000,
    BMP280_STANDBY_125  = 0b01000000,
    BMP280_STANDBY_250  = 0b01100000,
    BMP280_STANDBY_500  = 0b10000000,
    BMP280_STANDBY_1000 = 0b10100000,
    BMP280_STANDBY_2000 = 0b11000000,
    BMP280_STANDBY_4000 = 0b11100000,
} BMP280_STANDBY;

typedef enum {
    BMP280_FILTER_0  = 0b00000000,
    BMP280_FILTER_2  = 0b00000100,
    BMP280_FILTER_4  = 0b00001000,
    BMP280_FILTER_8  = 0b00001100,
    BMP280_FILTER_16 = 0b00010000,
} BMP280_FILTER;

// sensor struct
struct BMP280 {
    // i2c port number
    i2c_port_t i2c_port;

    // another struct to hold compensation parameters
    struct BMP280_COMP {
        uint16_t dig_T1;
        int16_t dig_T2;
        int16_t dig_T3;
        uint16_t dig_P1;
        int16_t dig_P2;
        int16_t dig_P3;
        int16_t dig_P4;
        int16_t dig_P5;
        int16_t dig_P6;
        int16_t dig_P7;
        int16_t dig_P8;
        int16_t dig_P9;
    } comp;

    // global value used for pressure compensation
    int32_t t_fine;

    // to hold the final results of temperature and pressure
    float temperature;
    float pressure;
};

// initializes sensor in default mode, and reads the compensation parameters.
// returns false if init failed.
bool bmp280_init(struct BMP280 *dev, i2c_port_t i2c_port);
void bmp280_config(struct BMP280 *dev, BMP280_OVRSMP_TEMP osrs_t, BMP280_OVRSMP_PRES osrs_p, BMP280_MODE mode, BMP280_FILTER filter, BMP280_STANDBY t_sb);

esp_err_t bmp280_read_data(struct BMP280 *dev);
void bmp280_read_comp(struct BMP280 *dev);

int32_t bmp280_compensate_T(struct BMP280 *dev, int32_t adc_T);
uint32_t bmp280_compensate_P(struct BMP280 *dev, int32_t adc_P);

// low level
uint8_t bmp280_read8(struct BMP280 *dev, uint8_t addr);
uint16_t bmp280_read16(struct BMP280 *dev, uint8_t addr);
esp_err_t bmp280_write8(struct BMP280 *dev, uint8_t addr, uint8_t data);

esp_err_t bmp280_readbuf(struct BMP280 *dev, uint8_t addr, uint8_t *buf, size_t n);

#endif