#ifndef _IOT_SHT30_H_
#define _IOT_SHT30_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/i2c.h"
#include "iot_i2c_bus.h"

#define SHT30_I2C_ADDRESS_DEFAULT   (0x44)
typedef void* sht30_handle_t;

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param bus I2C bus object handle
 * @param dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
esp_err_t iot_sht30_init(sht30_handle_t dev);

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param bus I2C bus object handle
 * @param dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
sht30_handle_t iot_sht30_create(i2c_bus_handle_t bus, uint16_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of sht30
 * @param del_bus Whether to delete the I2C bus
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_sht30_delete(sht30_handle_t sensor, bool del_bus);

/**
 * @brief Write value to one register of sht30
 *
 * @param sensor object handle of sht30
 * @param reg_addr register address
 * @param data register value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_sht30_write_byte(sht30_handle_t dev, uint8_t addr, uint8_t data);

/**
 * @brief read temperature and huminity
 * @param sensor object handle of sht30
 * @tp pointer to accept temperature
 * @rh pointer to accept huminity
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_sht30_get_data(sht30_handle_t sensor, float* tp, float* rh);

/**
 * @brief read huminity
 * @param sensor object handle of sht30
 * @rh pointer to accept huminity
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_sht30_get_huminity(sht30_handle_t sensor, float* rh);

/**
 * @brief read temperature
 * @param sensor object handle of sht30
 * @tp pointer to accept temperature
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t iot_sht30_get_temperature(sht30_handle_t sensor, float* tp);

#ifdef __cplusplus
}
#endif

#endif
