#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <icm42670.h>

static const char *TAG = "icm42670";

#define INT_SOURCE1 0x2C

#define PORT 0
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_GND)
#define I2C_ADDR ICM42670_I2C_ADDR_GND
#endif
#if defined(CONFIG_EXAMPLE_I2C_ADDRESS_VCC)
#define I2C_ADDR ICM42670_I2C_ADDR_VCC
#endif

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif
#define ALPHA 0.8 
/* Find gpio definitions in sdkconfig */

void icm42670_test(void *pvParameters)
{
    // init device descriptor and device
    icm42670_t dev = { 0 };
    ESP_ERROR_CHECK(
        icm42670_init_desc(&dev, I2C_ADDR, PORT, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(icm42670_init(&dev));

    // enable accelerometer and gyro in low-noise (LN) mode
    ESP_ERROR_CHECK(icm42670_set_gyro_pwr_mode(&dev, ICM42670_GYRO_ENABLE_LN_MODE));
    ESP_ERROR_CHECK(icm42670_set_accel_pwr_mode(&dev, ICM42670_ACCEL_ENABLE_LN_MODE));

    /* OPTIONAL */
    // enable low-pass-filters on accelerometer and gyro
    ESP_ERROR_CHECK(icm42670_set_accel_lpf(&dev, ICM42670_ACCEL_LFP_53HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_lpf(&dev, ICM42670_GYRO_LFP_53HZ));
    // set output data rate (ODR)
    ESP_ERROR_CHECK(icm42670_set_accel_odr(&dev, ICM42670_ACCEL_ODR_200HZ));
    ESP_ERROR_CHECK(icm42670_set_gyro_odr(&dev, ICM42670_GYRO_ODR_200HZ));
    // set full scale range (FSR)
    ESP_ERROR_CHECK(icm42670_set_accel_fsr(&dev, ICM42670_ACCEL_RANGE_16G));
    ESP_ERROR_CHECK(icm42670_set_gyro_fsr(&dev, ICM42670_GYRO_RANGE_2000DPS));

    // read temperature sensor value once
    float temperature;
    ESP_ERROR_CHECK(icm42670_read_temperature(&dev, &temperature));
    ESP_LOGI(TAG, "Temperature reading: %f", temperature);

    int16_t raw_reading;
    uint8_t data_register1;
    uint8_t data_register2;
    uint8_t data_register3;
    uint8_t data_register4;
    uint8_t data_register5;
    uint8_t data_register6;

    /* select which acceleration or gyro value should be read: */
    data_register1 = ICM42670_REG_ACCEL_DATA_X1;
    data_register2= ICM42670_REG_ACCEL_DATA_Y1;
    data_register3= ICM42670_REG_ACCEL_DATA_Z1;
    data_register4= ICM42670_REG_GYRO_DATA_X1;
    data_register5 = ICM42670_REG_GYRO_DATA_Y1;
    data_register6= ICM42670_REG_GYRO_DATA_Z1;
    // ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, INT_SOURCE1, &raw_reading));
    // printf("%d\n", raw_reading);
    int g=0;
    // now poll selected accelerometer or gyro raw value directly from registers
    while (1)
    {
        ESP_ERROR_CHECK(icm42670_read_raw_data(&dev, data_register2, &raw_reading));

        //ESP_LOGI(TAG, "Raw accelerometer / gyro reading: %d", raw_reading);
        // printf("%d %d %d\n",5000,raw_reading,-5000);
        //float accel_y = raw_reading/2048.0;
        g=0.9*g+0.1*raw_reading;
        float lin_accel_y = raw_reading-g;
        printf("%d %.2f %d\n",2000,lin_accel_y,-2000);
        vTaskDelay(pdMS_TO_TICKS(75));
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(icm42670_test, "icm42670_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
