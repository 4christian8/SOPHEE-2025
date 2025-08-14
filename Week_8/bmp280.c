#include "bmp280.h"

#define BMP280_SPI &hspi3  // Use SPI3 for NUCLEO-F401

extern SPI_HandleTypeDef hspi3;

BMP280_CalibData bmp280_calib;
int32_t t_fine;

void bmp280_write_register(uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { reg & 0x7F, value };
    uint8_t rx[2] = { 0 };

    HAL_GPIO_WritePin(BMP280_SPI_CS_PORT, BMP280_SPI_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(BMP280_SPI, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(BMP280_SPI_CS_PORT, BMP280_SPI_CS_PIN, GPIO_PIN_SET);
}

uint8_t bmp280_read_register(uint8_t reg) {
    uint8_t tx[2] = { reg | 0x80, 0x00 };  // Read command (MSB = 1)
    uint8_t rx[2] = { 0 };

    HAL_GPIO_WritePin(BMP280_SPI_CS_PORT, BMP280_SPI_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(BMP280_SPI, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(BMP280_SPI_CS_PORT, BMP280_SPI_CS_PIN, GPIO_PIN_SET);

    printf("Transmitted: 0x%02X 0x%02X\r\n", tx[0], tx[1]);
    printf("Received: 0x%02X 0x%02X\r\n\n", rx[0], rx[1]);

    return rx[1];  // Data received in second byte
}

void bmp280_read_calibration() {
    uint8_t tx[25] = { BMP280_REG_CALIB | 0x80 };
    uint8_t rx[25] = { 0 };

    HAL_GPIO_WritePin(BMP280_SPI_CS_PORT, BMP280_SPI_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(BMP280_SPI, tx, rx, 25, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(BMP280_SPI_CS_PORT, BMP280_SPI_CS_PIN, GPIO_PIN_SET);

    bmp280_calib.dig_T1 = (rx[2] << 8) | rx[1];
    bmp280_calib.dig_T2 = (rx[4] << 8) | rx[3];
    bmp280_calib.dig_T3 = (rx[6] << 8) | rx[5];
    bmp280_calib.dig_P1 = (rx[8] << 8) | rx[7];
    bmp280_calib.dig_P2 = (rx[10] << 8) | rx[9];
    bmp280_calib.dig_P3 = (rx[12] << 8) | rx[11];
    bmp280_calib.dig_P4 = (rx[14] << 8) | rx[13];
    bmp280_calib.dig_P5 = (rx[16] << 8) | rx[15];
    bmp280_calib.dig_P6 = (rx[18] << 8) | rx[17];
    bmp280_calib.dig_P7 = (rx[20] << 8) | rx[19];
    bmp280_calib.dig_P8 = (rx[22] << 8) | rx[21];
    bmp280_calib.dig_P9 = (rx[24] << 8) | rx[23];
}

void bmp280_init() {
    uint8_t id = bmp280_read_register(BMP280_REG_ID);
    if (id != 0x58) {
        printf("BMP280 not detected!\n");
        return;
    }
    bmp280_write_register(BMP280_REG_RESET, BMP280_RESET_CMD);
    HAL_Delay(10);
    bmp280_read_calibration();
    bmp280_write_register(BMP280_REG_CTRL, 0x27);
    bmp280_write_register(BMP280_REG_CONFIG, 0xA0);
}

void bmp280_read_data(int32_t *temperature, uint32_t *pressure) {
    uint8_t tx[7] = { BMP280_REG_PRESS | 0x80 };
    uint8_t rx[7] = { 0 };

    HAL_GPIO_WritePin(BMP280_SPI_CS_PORT, BMP280_SPI_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(BMP280_SPI, tx, rx, 7, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(BMP280_SPI_CS_PORT, BMP280_SPI_CS_PIN, GPIO_PIN_SET);

    int32_t adc_p = (rx[1] << 12) | (rx[2] << 4) | (rx[3] >> 4);
    int32_t adc_t = (rx[4] << 12) | (rx[5] << 4) | (rx[6] >> 4);

    *temperature = bmp280_compensate_temp(adc_t);
    *pressure = bmp280_compensate_pressure(adc_p);
}

int32_t bmp280_compensate_temp(int32_t adc_T) {
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) * ((int32_t)bmp280_calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1))) >> 12) * ((int32_t)bmp280_calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;  // Returns temperature in 0.01°C
}

uint32_t bmp280_compensate_pressure(int32_t adc_P) {
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_calib.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) * ((int64_t)bmp280_calib.dig_P1)) >> 33;

    if (var1 == 0) return 0;  // Avoid division by zero
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_calib.dig_P7) << 4);
    return (uint32_t)(p / 256);  // Pressure in Pa
}
