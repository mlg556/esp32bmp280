
#include "bmp280.h"

static const char *TAG = "BMP280 Driver";

bool bmp280_init(BMP280 *bmp, i2c_port_t i2c_num) {
    bmp->i2c_num = i2c_num;
    bmp->t_fine  = 0;
    bmp->adc_P   = 0;
    bmp->adc_T   = 0;

    uint8_t chipID;

    // check chipID, to see if it matches
    chipID = bmp280_read8(bmp, BMP280_REG_CHIPID);

    if (chipID != BMP280_CHIPID) {
        ESP_LOGE(TAG, "Expected chip ID %x, got %x\n", BMP280_CHIPID, chipID);
        return false;
    }

    // Soft reset. needed?
    // bmp280_write8(bmp, BMP280_REG_SOFTRESET, BMP280_RESET);

    bmp280_read_comp(bmp);

    ESP_LOGD(
        TAG,
        "\nT1: %" PRIu16
        "-"
        "T2: %" PRId16
        "-"
        "T3: %" PRId16
        "-"
        "P1: %" PRIu16
        "-"
        "P2: %" PRId16
        "-"
        "P3: %" PRId16
        "-"
        "P4: %" PRId16
        "-"
        "P5: %" PRId16
        "-"
        "P6: %" PRId16
        "-"
        "P7: %" PRId16
        "-"
        "P8: %" PRId16
        "-"
        "P9: %" PRId16 "-\n",
        bmp->dig_T1,
        bmp->dig_T2,
        bmp->dig_T3,
        bmp->dig_P1,
        bmp->dig_P2,
        bmp->dig_P3,
        bmp->dig_P4,
        bmp->dig_P5,
        bmp->dig_P6,
        bmp->dig_P7,
        bmp->dig_P8,
        bmp->dig_P9);

    // default config
    bmp280_config(bmp, BMP280_OVRSMP_TEMP_1X, BMP280_OVRSMP_PRES_1X, BMP280_MODE_NORMAL, BMP280_FILTER_2, BMP280_STANDBY_500);

    return true;
}

void bmp280_config(BMP280 *bmp, BMP280_OVRSMP_TEMP osrs_t, BMP280_OVRSMP_PRES osrs_p, BMP280_MODE mode, BMP280_FILTER filter, BMP280_STANDBY t_sb) {
    bmp->ctrl_meas = osrs_t | osrs_p | mode;
    bmp->config    = filter | t_sb;
    // write ctrl_meas
    bmp280_write8(bmp, BMP280_REG_CONTROL, bmp->ctrl_meas);
    // write config
    bmp280_write8(bmp, BMP280_REG_CONFIG, bmp->config);
}

// read the compensation parameters
void bmp280_read_comp(BMP280 *bmp) {
    bmp->dig_T1 = bmp280_read16(bmp, BMP280_REG_DIG_T1);
    bmp->dig_T2 = bmp280_read16(bmp, BMP280_REG_DIG_T2);
    bmp->dig_T3 = bmp280_read16(bmp, BMP280_REG_DIG_T3);
    bmp->dig_P1 = bmp280_read16(bmp, BMP280_REG_DIG_P1);
    bmp->dig_P2 = (int16_t)bmp280_read16(bmp, BMP280_REG_DIG_P2);
    bmp->dig_P3 = (int16_t)bmp280_read16(bmp, BMP280_REG_DIG_P3);
    bmp->dig_P4 = (int16_t)bmp280_read16(bmp, BMP280_REG_DIG_P4);
    bmp->dig_P5 = (int16_t)bmp280_read16(bmp, BMP280_REG_DIG_P5);
    bmp->dig_P6 = (int16_t)bmp280_read16(bmp, BMP280_REG_DIG_P6);
    bmp->dig_P7 = (int16_t)bmp280_read16(bmp, BMP280_REG_DIG_P7);
    bmp->dig_P8 = (int16_t)bmp280_read16(bmp, BMP280_REG_DIG_P8);
    bmp->dig_P9 = (int16_t)bmp280_read16(bmp, BMP280_REG_DIG_P9);
}

uint8_t bmp280_read8(BMP280 *bmp, uint8_t addr) {
    esp_err_t ret;
    size_t num_bytes = 1;

    bmp->buf_w[0] = addr;

    // attempt i2c read
    ret = i2c_master_write_read_device(bmp->i2c_num, BMP280_ADDRESS, bmp->buf_w, 1, bmp->buf_r, num_bytes, BMP280_TIMEOUT);  // timeout is 2 ticks

    if (ret != ESP_OK)
        return 0;

    return bmp->buf_r[0];
}

uint16_t bmp280_read16(BMP280 *bmp, uint8_t addr) {
    esp_err_t ret;
    size_t num_bytes = 2;

    bmp->buf_w[0] = addr;

    // attempt i2c read
    ret = i2c_master_write_read_device(bmp->i2c_num, BMP280_ADDRESS, bmp->buf_w, 1, bmp->buf_r, num_bytes, BMP280_TIMEOUT);  // timeout is 2 ticks

    if (ret != ESP_OK)
        return 0;

    // combine buf[0] and buf[1] in to an unsigned 16bit
    return (uint16_t)((bmp->buf_r[1] << 8) | bmp->buf_r[0]);
}

void bmp280_write8(BMP280 *bmp, uint8_t addr, uint8_t data) {
    uint8_t num_bytes = 2;
    // write address and data to buffer, send 2 bytes
    bmp->buf_w[0] = addr;
    bmp->buf_w[1] = data;

    // send buffer, 1 byte
    i2c_master_write_to_device(bmp->i2c_num, BMP280_ADDRESS, bmp->buf_w, num_bytes, BMP280_TIMEOUT);
}

// datasheet recommends burst read for sensor data, since the data can change mid-transmission
// we have to do 6 reads, starting at 0xF7 and ending at 0xFC
// so we first write 0xF7, then read 6 bytes, 6x8 = 48 bits
// both temperature and pressure are unsigned 20 bits, so 4 bits for each are xxxx
void bmp280_read_data(BMP280 *bmp) {
    // do 6 reads into bmp buffer
    uint8_t addr     = BMP280_REG_PRESSURE;
    size_t num_bytes = 6;

    bmp->buf_w[0] = addr;

    i2c_master_write_read_device(bmp->i2c_num, BMP280_ADDRESS, bmp->buf_w, 1, bmp->buf_r, num_bytes, BMP280_TIMEOUT);  // timeout is 2 ticks

    // save into 32 bit signed integers
    bmp->adc_P = bmp->buf_r[0] << 12 | bmp->buf_r[1] << 4 | bmp->buf_r[2] >> 4;
    bmp->adc_T = bmp->buf_r[3] << 12 | bmp->buf_r[4] << 4 | bmp->buf_r[5] >> 4;

    // compensate using the acquired parameters
    bmp->temperature = ((float)(bmp280_compensate_T(bmp))) / 100;  // in degrees
    bmp->pressure    = ((float)(bmp280_compensate_P(bmp))) / 256;  // in pascals
}

// TODO: implement forced
void bmp280_read_forced(BMP280 *bmp) {
    // // write ctrl_meas, required for FORCED mode
    // bmp280_write8(bmp, BMP280_REG_CONTROL, bmp->ctrl_meas);

    // // do 6 reads into bmp buffer
    // uint8_t addr = BMP280_REG_PRESSURE;
    // HAL_I2C_Mem_Read(bmp->i2c, BMP280_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT, bmp->buf, 6, HAL_MAX_DELAY);

    // // save into 32 bit signed integers
    // bmp->adc_P = bmp->buf[0] << 12 | bmp->buf[1] << 4 | bmp->buf[2] >> 4;
    // bmp->adc_T = bmp->buf[3] << 12 | bmp->buf[4] << 4 | bmp->buf[5] >> 4;
}

// from datasheet p.22, rev.1.1.
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
int32_t bmp280_compensate_T(BMP280 *bmp) {
    int32_t var1, var2, t_fine;

    var1 = ((((bmp->adc_T >> 3) - ((int32_t)bmp->dig_T1 << 1))) * (int32_t)bmp->dig_T2) >> 11;
    var2 = (((((bmp->adc_T >> 4) - (int32_t)bmp->dig_T1) * ((bmp->adc_T >> 4) - (int32_t)bmp->dig_T1)) >> 12) * (int32_t)bmp->dig_T3) >> 14;

    t_fine = var1 + var2;

    bmp->t_fine = t_fine;  // save t_fine for pressure compensation

    return (t_fine * 5 + 128) >> 8;
}

// from datasheet p.22, rev.1.1.
// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280_compensate_P(BMP280 *bmp) {
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp->dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp->dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp->dig_P3) >> 8) + ((var1 * (int64_t)bmp->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp->dig_P1) >> 33;
    if (var1 == 0) {
        return 0;  // avoid exception caused by division by zero
    }
    p    = 1048576 - bmp->adc_P;
    p    = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp->dig_P8) * p) >> 19;
    p    = ((p + var1 + var2) >> 8) + (((int64_t)bmp->dig_P7) << 4);
    return (uint32_t)p;
}