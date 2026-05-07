#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"

#define I2C_PORT    I2C_NUM_0
#define SDA_PIN     8
#define SCL_PIN     9
#define I2C_FREQ    100000

#define BMP180_ADDR 0x77

#define BMP180_REG_ID       0xD0
#define BMP180_REG_CTRL     0xF4
#define BMP180_REG_DATA     0xF6

#define BMP180_CMD_TEMP     0x2E
#define BMP180_CMD_PRESSURE 0x34

static int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
static uint16_t AC4, AC5, AC6;

esp_err_t i2c_init(void)
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &config));
    return i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t bmp180_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_PORT,
        BMP180_ADDR,
        &reg,
        1,
        data,
        len,
        pdMS_TO_TICKS(1000)
    );
}

esp_err_t bmp180_write(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    return i2c_master_write_to_device(
        I2C_PORT,
        BMP180_ADDR,
        data,
        2,
        pdMS_TO_TICKS(1000)
    );
}

uint16_t bmp180_read_u16(uint8_t reg)
{
    uint8_t data[2];
    bmp180_read(reg, data, 2);
    return ((uint16_t)data[0] << 8) | data[1];
}

int16_t bmp180_read_s16(uint8_t reg)
{
    return (int16_t)bmp180_read_u16(reg);
}

void bmp180_read_calibration(void)
{
    AC1 = bmp180_read_s16(0xAA);
    AC2 = bmp180_read_s16(0xAC);
    AC3 = bmp180_read_s16(0xAE);
    AC4 = bmp180_read_u16(0xB0);
    AC5 = bmp180_read_u16(0xB2);
    AC6 = bmp180_read_u16(0xB4);
    B1  = bmp180_read_s16(0xB6);
    B2  = bmp180_read_s16(0xB8);
    MB  = bmp180_read_s16(0xBA);
    MC  = bmp180_read_s16(0xBC);
    MD  = bmp180_read_s16(0xBE);
}

int32_t bmp180_read_raw_temperature(void)
{
    bmp180_write(BMP180_REG_CTRL, BMP180_CMD_TEMP);
    vTaskDelay(pdMS_TO_TICKS(5));

    return bmp180_read_u16(BMP180_REG_DATA);
}

int32_t bmp180_read_raw_pressure(void)
{
    uint8_t data[3];

    bmp180_write(BMP180_REG_CTRL, BMP180_CMD_PRESSURE);
    vTaskDelay(pdMS_TO_TICKS(5));

    bmp180_read(BMP180_REG_DATA, data, 3);

    return ((int32_t)data[0] << 16 | (int32_t)data[1] << 8 | data[2]) >> 8;
}

void bmp180_read_temperature_pressure(float *temperature, int32_t *pressure)
{
    int32_t UT = bmp180_read_raw_temperature();
    int32_t UP = bmp180_read_raw_pressure();

    int32_t X1 = ((UT - (int32_t)AC6) * (int32_t)AC5) >> 15;
    int32_t X2 = ((int32_t)MC << 11) / (X1 + MD);
    int32_t B5 = X1 + X2;

    int32_t temp = (B5 + 8) >> 4;
    *temperature = temp / 10.0;

    int32_t B6 = B5 - 4000;

    X1 = ((int32_t)B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = ((int32_t)AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;

    int32_t B3 = ((((int32_t)AC1 * 4 + X3) + 2) >> 2);

    X1 = ((int32_t)AC3 * B6) >> 13;
    X2 = ((int32_t)B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;

    uint32_t B4 = ((uint32_t)AC4 * (uint32_t)(X3 + 32768)) >> 15;
    uint32_t B7 = ((uint32_t)UP - B3) * 50000;

    int32_t p;

    if (B7 < 0x80000000) {
        p = (B7 * 2) / B4;
    } else {
        p = (B7 / B4) * 2;
    }

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;

    p = p + ((X1 + X2 + 3791) >> 4);

    *pressure = p;
}

void app_main(void)
{
    printf("Iniciando ESP32-S3 com BMP180...\n");

    ESP_ERROR_CHECK(i2c_init());

    uint8_t id = 0;
    esp_err_t ret = bmp180_read(BMP180_REG_ID, &id, 1);

    if (ret != ESP_OK) {
        printf("Erro ao comunicar com BMP180: %s\n", esp_err_to_name(ret));
        return;
    }

    printf("ID do BMP180: 0x%02X\n", id);

    if (id != 0x55) {
        printf("Sensor BMP180 não reconhecido.\n");
        return;
    }

    printf("BMP180 encontrado!\n");

    bmp180_read_calibration();

    while (1) {
        float temperatura = 0.0;
        int32_t pressao = 0;

        bmp180_read_temperature_pressure(&temperatura, &pressao);

        printf("Temperatura: %.2f °C\n", temperatura);
        printf("Pressao: %ld Pa\n", pressao);
        printf("Pressao: %.2f hPa\n", pressao / 100.0);
        printf("-------------------------\n");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}