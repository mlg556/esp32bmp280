
#include "bmp280.h"

static const char *TAG = "BMP280 Driver";

bool bmp280_init(BMP280 *dev, i2c_port_t i2c_num) {
    dev->i2c_num = i2c_num;
    dev->t_fine  = 0;
    dev->adc_P   = 0;
    dev->adc_T   = 0;

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

    // ESP_LOGD(
    //     TAG,
    //     "\nT1: %" PRIu16
    //     "-"
    //     "T2: %" PRId16
    //     "-"
    //     "T3: %" PRId16
    //     "-"
    //     "P1: %" PRIu16
    //     "-"
    //     "P2: %" PRId16
    //     "-"
    //     "P3: %" PRId16
    //     "-"
    //     "P4: %" PRId16
    //     "-"
    //     "P5: %" PRId16
    //     "-"
    //     "P6: %" PRId16
    //     "-"
    //     "P7: %" PRId16
    //     "-"
    //     "P8: %" PRId16
    //     "-"
    //     "P9: %" PRId16 "-\n",
    //     dev->dig_T1,
    //     dev->dig_T2,
    //     dev->dig_T3,
    //     dev->dig_P1,
    //     dev->dig_P2,
    //     dev->dig_P3,
    //     dev->dig_P4,
    //     dev->dig_P5,
    //     dev->dig_P6,
    //     dev->dig_P7,
    //     dev->dig_P8,
    //     dev->dig_P9);

    // default config
    bmp280_config(dev, BMP280_OVRSMP_TEMP_1X, BMP280_OVRSMP_PRES_1X, BMP280_MODE_NORMAL, BMP280_FILTER_2, BMP280_STANDBY_500);

    return true;
}

// read n bytes into buffer from addr
esp_err_t bmp280_readbuf(BMP280 *dev, const uint8_t addr, const size_t n) {
    // read into device's buf_rx: receive buffer
    return i2c_master_write_read_device(dev->i2c_num, BMP280_ADDRESS, &addr, 1, dev->buf_rx, n, BMP280_TIMEOUT);  // timeout is 2 ticks
}

uint8_t bmp280_read8(BMP280 *dev, const uint8_t addr) {
    if (bmp280_readbuf(dev, addr, 1) != ESP_OK)
        return 0;

    return dev->buf_rx[0];
}

uint16_t bmp280_read16(BMP280 *dev, uint8_t addr) {
    if (bmp280_readbuf(dev, addr, 2) != ESP_OK)
        return 0;
    // combine buf[0] and buf[1] in to an unsigned 16bit
    return (uint16_t)((dev->buf_rx[1] << 8) | dev->buf_rx[0]);
}

esp_err_t bmp280_write8(BMP280 *dev, uint8_t addr, uint8_t data) {
    const uint8_t num_bytes = 2;
    // write address and data to buffer, send 2 bytes
    dev->buf_tx[0] = addr;
    dev->buf_tx[1] = data;

    // send buffer, 2bytes
    return i2c_master_write_to_device(dev->i2c_num, BMP280_ADDRESS, dev->buf_tx, num_bytes, BMP280_TIMEOUT);
}

void bmp280_config(BMP280 *dev, BMP280_OVRSMP_TEMP osrs_t, BMP280_OVRSMP_PRES osrs_p, BMP280_MODE mode, BMP280_FILTER filter, BMP280_STANDBY t_sb) {
    dev->ctrl_meas = osrs_t | osrs_p | mode;
    dev->config    = filter | t_sb;
    // write ctrl_meas
    bmp280_write8(dev, BMP280_REG_CONTROL, dev->ctrl_meas);
    // write config
    bmp280_write8(dev, BMP280_REG_CONFIG, dev->config);
}

// read the compensation parameters
void bmp280_read_comp(BMP280 *dev) {
    dev->dig_T1 = bmp280_read16(dev, BMP280_REG_DIG_T1);
    dev->dig_T2 = bmp280_read16(dev, BMP280_REG_DIG_T2);
    dev->dig_T3 = bmp280_read16(dev, BMP280_REG_DIG_T3);
    dev->dig_P1 = bmp280_read16(dev, BMP280_REG_DIG_P1);
    dev->dig_P2 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P2);
    dev->dig_P3 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P3);
    dev->dig_P4 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P4);
    dev->dig_P5 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P5);
    dev->dig_P6 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P6);
    dev->dig_P7 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P7);
    dev->dig_P8 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P8);
    dev->dig_P9 = (int16_t)bmp280_read16(dev, BMP280_REG_DIG_P9);
}

// datasheet recommends burst read for sensor data, since the data can change mid-transmission
// we have to do 6 reads, starting at 0xF7 and ending at 0xFC
// so we first write 0xF7, then read 6 bytes, 6x8 = 48 bits
// both temperature and pressure are unsigned 20 bits, so 4 bits for each are xxxx
esp_err_t bmp280_read_data(BMP280 *dev) {
    // do 6 reads into buf_rx from BMP280_REG_PRESSURE
    if (bmp280_readbuf(dev, BMP280_REG_PRESSURE, 6) != ESP_OK)
        return ESP_FAIL;

    // save into 32 bit signed integers
    dev->adc_P = dev->buf_rx[0] << 12 | dev->buf_rx[1] << 4 | dev->buf_rx[2] >> 4;
    dev->adc_T = dev->buf_rx[3] << 12 | dev->buf_rx[4] << 4 | dev->buf_rx[5] >> 4;

    // compensate using the acquired parameters
    dev->temperature = ((float)(bmp280_compensate_T(dev))) / 100;  // in degrees
    dev->pressure    = ((float)(bmp280_compensate_P(dev))) / 256;  // in pascals

    return ESP_OK;
}

// TODO: implement forced
void BMP280_read_forced(BMP280 *dev) {
    // // write ctrl_meas, required for FORCED mode
    // BMP280_write8(dev, BMP280_REG_CONTROL, dev->ctrl_meas);

    // // do 6 reads into dev buffer
    // uint8_t addr = BMP280_REG_PRESSURE;
    // HAL_I2C_Mem_Read(dev->i2c, BMP280_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT, dev->buf, 6, HAL_MAX_DELAY);

    // // save into 32 bit signed integers
    // dev->adc_P = dev->buf[0] << 12 | dev->buf[1] << 4 | dev->buf[2] >> 4;
    // dev->adc_T = dev->buf[3] << 12 | dev->buf[4] << 4 | dev->buf[5] >> 4;
}

// from datasheet p.22, rev.1.1.
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
int32_t bmp280_compensate_T(BMP280 *dev) {
    int32_t var1, var2, t_fine;

    var1 = ((((dev->adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) * (int32_t)dev->dig_T2) >> 11;
    var2 = (((((dev->adc_T >> 4) - (int32_t)dev->dig_T1) * ((dev->adc_T >> 4) - (int32_t)dev->dig_T1)) >> 12) * (int32_t)dev->dig_T3) >> 14;

    t_fine = var1 + var2;

    dev->t_fine = t_fine;  // save t_fine for pressure compensation

    return (t_fine * 5 + 128) >> 8;
}

// from datasheet p.22, rev.1.1.
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280_compensate_P(BMP280 *dev) {
    int64_t var1, var2, p;
    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->dig_P1) >> 33;
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    p    = 1048576 - dev->adc_P;
    p    = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->dig_P8) * p) >> 19;
    p    = ((p + var1 + var2) >> 8) + (((int64_t)dev->dig_P7) << 4);
    return (uint32_t)p;
}