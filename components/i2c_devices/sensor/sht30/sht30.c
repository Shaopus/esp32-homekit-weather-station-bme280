#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "iot_sht30.h"
#include "iot_i2c_bus.h"

#define WRITE_BIT  I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN   0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0               /*!< I2C ack value */
#define NACK_VAL   0x1               /*!< I2C nack value */
#define CMD_FETCH_DATA_H    0x22     /*!<循环采样，参考sht30 datasheet */
#define CMD_FETCH_DATA_L    0x36

static const char* TAG = "sht30";
typedef struct {
    i2c_bus_handle_t bus;
    uint16_t dev_addr;
} sht30_dev_t;

static uint8_t iot_sht30_calcrc(uint8_t *data, uint8_t nbrOfBytes);
static uint8_t iot_sht30_checkcrc(uint8_t *pdata, uint8_t nbrOfBytes, uint8_t checksum);

esp_err_t iot_sht30_init(sht30_handle_t dev)
{
    // check if sensor, i.e. the chip ID is correct
    esp_err_t ret;
    ret = iot_sht30_write_byte(dev,CMD_FETCH_DATA_H,CMD_FETCH_DATA_L);
    return ret;
}

sht30_handle_t iot_sht30_create(i2c_bus_handle_t bus, uint16_t dev_addr)
{
    sht30_dev_t* sensor = (sht30_dev_t*) calloc(1, sizeof(sht30_dev_t));
    sensor->bus = bus;
    sensor->dev_addr = dev_addr;
    return (sht30_handle_t) sensor;
}

esp_err_t iot_sht30_delete(sht30_handle_t sensor, bool del_bus)
{
    sht30_dev_t* sens = (sht30_dev_t*) sensor;
    if(del_bus) {
        iot_i2c_bus_delete(sens->bus);
        sens->bus = NULL;
    }
    free(sens);
    return ESP_OK;
}

esp_err_t iot_sht30_write_byte(sht30_handle_t dev, uint8_t addr, uint8_t data)
{
    //start-device_addr-word_addr-data-stop
    esp_err_t ret;
    sht30_dev_t* device = (sht30_dev_t*) dev;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device->dev_addr << 1) | WRITE_BIT,
    ACK_CHECK_EN);
    i2c_master_write_byte(cmd, addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = iot_i2c_bus_cmd_begin(device->bus, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "SNED WRITE ERROR\n");
        return ESP_FAIL;
    }
    return ret;
}

static uint8_t iot_sht30_calcrc(uint8_t *data, uint8_t nbrOfBytes)
{
	uint8_t bit;        // bit mask
    uint8_t crc = 0xFF; // calculated checksum
    uint8_t byteCtr;    // byte counter
    uint32_t POLYNOMIAL =  0x131;           // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

    // calculates 8-Bit checksum with given polynomial
    for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
        crc ^= (data[byteCtr]);
        for(bit = 8; bit > 0; --bit) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ POLYNOMIAL;
            }  else {
                crc = (crc << 1);
            }
        }
    }
	return crc;
}

static uint8_t iot_sht30_checkcrc(uint8_t *pdata, uint8_t nbrOfBytes, uint8_t checksum)
{
    uint8_t crc;
	crc = iot_sht30_calcrc(pdata, nbrOfBytes);// calculates 8-Bit checksum
    if(crc != checksum) 
    {   
        return 1;           
    }
    return 0;              
}

esp_err_t iot_sht30_get_data(sht30_handle_t sensor, float* tp, float* rh)
{
    sht30_dev_t* sens = (sht30_dev_t*) sensor;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    uint8_t data[6];
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (sens->dev_addr << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, 5, ACK_VAL);
    i2c_master_read_byte(cmd, &data[5], NACK_VAL);
    i2c_master_stop(cmd);
    int ret = iot_i2c_bus_cmd_begin(sens->bus, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(20 / portTICK_PERIOD_MS);

    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "SEND READ ERROR \n");
        return ESP_FAIL;
    }

    if( (!iot_sht30_checkcrc(data,2,data[2])) && (!iot_sht30_checkcrc(data+3,2,data[5])) ){

        if (tp) {
            *tp = (float) ((((data[0] * 256)+ data[1]) * 175) / 65535.0 - 45);
        }
        if (rh) {
            *rh = (float) ((((data[3] * 256) + data[4]) * 100) /65535.0);
        }

    }else{
        ESP_LOGE(TAG, "INVAILD CRC ERROR \n");
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;

}

esp_err_t iot_sht30_get_huminity(sht30_handle_t sensor, float* rh)
{
    return iot_sht30_get_data(sensor, NULL, rh);
}

esp_err_t iot_sht30_get_temperature(sht30_handle_t sensor, float* tp)
{
    return iot_sht30_get_data(sensor, tp, NULL);
}