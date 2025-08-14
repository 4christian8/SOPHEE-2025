#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>

#define BMP280_SPI_CS_PIN GPIO_PIN_4
#define BMP280_SPI_CS_PORT GPIOA

// Fill in values here
#define BMP280_REG_ID       0xD0 //Register ID
#define BMP280_REG_RESET     //Reset
#define BMP280_REG_CTRL      //Control Measurement
#define BMP280_REG_CONFIG    //Config
#define BMP280_REG_PRESS     //Pressure Measurement (MSB)
#define BMP280_REG_TEMP      //Temperature Measurement (MSB)
#define BMP280_REG_CALIB    0x88 //Calibration Parameters
#define BMP280_RESET_CMD     //CMD sent to the reset register to perform power-on-reset procedure

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_CalibData;

void bmp280_write_register(uint8_t reg, uint8_t value);

uint8_t bmp280_read_register(uint8_t reg);

void bmp280_read_calibration();

void bmp280_init();

void bmp280_read_data(int32_t *temperature, uint32_t *pressure);

int32_t bmp280_compensate_temp(int32_t adc_T);

uint32_t bmp280_compensate_pressure(int32_t adc_P);
