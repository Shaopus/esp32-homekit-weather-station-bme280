#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "iot_sht30.h"
#include "iot_i2c_bus.h"

#define I2C_MASTER_SCL_IO           23          /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           22          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ          100000      /*!< I2C master clock frequency */

static i2c_bus_handle_t i2c_bus = NULL;
static sht30_handle_t sens = NULL;

/**
 * @brief i2c master initialization
 */
static void i2c_bus_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_bus = iot_i2c_bus_create(I2C_MASTER_NUM, &conf);
}

void sht30_init()
{
    i2c_bus_init();
    sens = iot_sht30_create(i2c_bus, sht30_SLAVE_ADDR);
}

void sht30_test_task(void* pvParameters)
{
    int cnt = 0;
    float tp, rh;
    while(1){
        iot_sht30_get_data(sens, &tp, &rh);
        printf("test[%d]: tp: %.02f; rh: %.02f %%\n", cnt++, tp, rh);
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void sht30_test()
{
    sht30_init();
    xTaskCreate(sht30_test_task, "sht30_test_task", 1024 * 2, NULL, 10, NULL);
}