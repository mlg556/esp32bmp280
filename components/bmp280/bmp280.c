
#include "bmp280.h"

static const char *TAG = "BMP280 Driver";

bool bmp280_init(struct BMP280 *dev, i2c_port_t i2c_port) {
    dev->i2c_port = i2c_port;
    dev->t_fine   = 0;

    uint8_t chipID;

    // check chipID, to see if it matches
    chipID = bmp280_read8(dev, BMP280_REG_CHIPID);

    if (chipID != BMP280_CHIPID) {
        ESP_LOGE(TAG, "Expected chip ID %x, got %x\n", BMP280_CHIPID, chipID);
        return false;
    }

    // Soft reset. needed?
    // BMP280_write8(dev, BMP280_REG_SOFTRESET, BMP280_RESET);

    bmp280_read_comp(dev);

    // default config
    // bmp280_config(dev, BMP280_OVRSMP_TEMP_1X, BMP280_OVRSMP_PRES_1X, BMP280_MODE_NORMAL, BMP280_FILTER_2, BMP280_STANDBY_500);

    return true;
}

// read n bytes from addr into buffer, return esp_err_t.
esp_err_t bmp280_readbuf(struct BMP280 *dev, const uint8_t addr, uint8_t *buf, size_t n) {
    return i2c_master_write_read_device(dev->i2c_port, BMP280_ADDRESS, &addr, 1, buf, n, BMP280_TIMEOUT);  // timeout is 2 ticks
}

uint8_t bmp280_read8(struct BMP280 *dev, const uint8_t addr) {
    uint8_t buf[1];

    // if read error, return 0
    if (bmp280_readbuf(dev, addr, buf, 1) != ESP_OK)
        return 0;

    // return byte
    return buf[0];
}

uint16_t bmp280_read16(struct BMP280 *dev, uint8_t addr) {
    uint8_t buf[2];

    if (bmp280_readbuf(dev, addr, buf, 2) != ESP_OK)
        return 0;

    // combine buf[0] and buf[1] in to an unsigned 16bit
    return (uint16_t)((buf[1] << 8) | buf[0]);
}

// write a byte to given address
esp_err_t bmp280_write8(struct BMP280 *dev, uint8_t addr, uint8_t data) {
    // buffer contains address and data, send 2 bytes
    uint8_t buf[2] = {addr, data};

    // send buffer, 2 bytes
    return i2c_master_write_to_device(dev->i2c_port, BMP280_ADDRESS, buf, 2, BMP280_TIMEOUT);
}

void bmp280_config(struct BMP280 *dev, BMP280_OVRSMP_TEMP osrs_t, BMP280_OVRSMP_PRES osrs_p, BMP280_MODE mode, BMP280_FILTER filter, BMP280_STANDBY t_sb) {
    uint8_t reg_ctrl_meas = osrs_t | osrs_p | mode;
    uint8_t reg_config    = filter | t_sb;

    // write ctrl_meas
    bmp280_write8(dev, BMP280_REG_CONTROL, reg_ctrl_meas);
    // write config
    bmp280_write8(dev, BMP280_REG_CONFIG, reg_config);
}

// read the compensation parameters
void bmp280_read_comp(struct BMP280 *dev) {
    dev->comp.dig_T1 = bmp280_read16(dev, BMP280_REG_DIG_T1);
    dev->comp.dig_T2 = bmp280_read16(dev, BMP280_REG_DIG_T2);
    dev->comp.dig_T3 = bmp280_read16(dev, BMP280_REG_DIG_T3);
    dev->comp.dig_P1 = bmp280_read16(dev, BMP280_REG_DIG_P1);
    dev->comp.dig_P2 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P2);
    dev->comp.dig_P3 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P3);
    dev->comp.dig_P4 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P4);
    dev->comp.dig_P5 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P5);
    dev->comp.dig_P6 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P6);
    dev->comp.dig_P7 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P7);
    dev->comp.dig_P8 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P8);
    dev->comp.dig_P9 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P9);
}

// datasheet recommends burst read for sensor data, since the data can change mid-transmission
// we have to do 6 reads, starting at 0xF7 and ending at 0xFC
// so we first write 0xF7, then read 6 bytes, 6x8 = 48 bits
// both temperature and pressure are unsigned 20 bits, so 4 bits for each are xxxx
esp_err_t bmp280_read_data(struct BMP280 *dev) {
    uint8_t buf[6];

    // do 6 reads into buf_rx from BMP280_REG_PRESSURE
    if (bmp280_readbuf(dev, BMP280_REG_PRESSURE, buf, 6) != ESP_OK)
        return ESP_FAIL;

    // save into 32 bit signed integers
    int32_t adc_P = buf[0] << 12 | buf[1] << 4 | buf[2] >> 4;
    int32_t adc_T = buf[3] << 12 | buf[4] << 4 | buf[5] >> 4;

    // compensate using the acquired parameters
    dev->temperature = ((float)(bmp280_compensate_T(dev, adc_T))) / 100;  // in degrees
    dev->pressure    = ((float)(bmp280_compensate_P(dev, adc_P))) / 256;  // in pascals

    return ESP_OK;
}

// from datasheet p.22, rev.1.1.
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
int32_t bmp280_compensate_T(struct BMP280 *dev, int32_t adc_T) {
    int32_t var1, var2, t_fine;

    var1 = ((((adc_T >> 3) - ((int32_t)dev->comp.dig_T1 << 1))) * (int32_t)dev->comp.dig_T2) >> 11;
    var2 = (((((adc_T >> 4) - (int32_t)dev->comp.dig_T1) * ((adc_T >> 4) - (int32_t)dev->comp.dig_T1)) >> 12) * (int32_t)dev->comp.dig_T3) >> 14;

    t_fine = var1 + var2;

    dev->t_fine = t_fine;  // save t_fine for compensate_P

    return (t_fine * 5 + 128) >> 8;
}

// from datasheet p.22, rev.1.1.
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280_compensate_P(struct BMP280 *dev, int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->comp.dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->comp.dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->comp.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->comp.dig_P3) >> 8) + ((var1 * (int64_t)dev->comp.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->comp.dig_P1) >> 33;
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    p    = 1048576 - adc_P;
    p    = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->comp.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->comp.dig_P8) * p) >> 19;
    p    = ((p + var1 + var2) >> 8) + (((int64_t)dev->comp.dig_P7) << 4);
    return (uint32_t)p;
}