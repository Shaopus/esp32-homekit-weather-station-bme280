#include <stdio.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include "driver/i2c.h"

#include "iot_i2c_bus.h"
#include "iot_bme280.h"

#include <homekit/homekit.h>
#include <homekit/characteristics.h>


#define I2C_MASTER_SCL_IO           22          /*!< gpio number for I2C master clock IO23*/
#define I2C_MASTER_SDA_IO           21          /*!< gpio number for I2C master data  IO22*/
#define I2C_MASTER_NUM              I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0           /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0           /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ          100000      /*!< I2C master clock frequency */

typedef struct {
    float temp;
    float humi;
    float pres;
    float altitude;
}env_t;

static const char *TAG = "HOMEKIT-BME280";
static env_t env ;

static i2c_bus_handle_t i2c_bus = NULL;
static bme280_handle_t bme280_dev = NULL;

void on_wifi_ready();

homekit_characteristic_t temperature = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 0);
homekit_characteristic_t humidity    = HOMEKIT_CHARACTERISTIC_(CURRENT_RELATIVE_HUMIDITY, 0);

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

void bme280_init()
{
    bme280_dev = iot_bme280_create(i2c_bus, BME280_I2C_ADDRESS_DEFAULT);
    ESP_LOGI("BME280:", "iot_bme280_init:%d", iot_bme280_init(bme280_dev));
}

void dev_init(void){
    i2c_bus_init();
    bme280_init();
}

void temperature_sensor_identify(homekit_value_t _value) {
    printf("Temperature sensor identify\n");
}

void temperature_sensor_task(void *_args) {

    while (1) {
        env.pres = iot_bme280_read_pressure(bme280_dev);
        env.temp = iot_bme280_read_temperature(bme280_dev);
        env.humi = iot_bme280_read_humidity(bme280_dev);
        env.altitude = iot_bme280_read_altitude(bme280_dev,10.13);
        if (env.temp) {
            ESP_LOGI(TAG,"T:%.1f, H:%.1f, P:%.1f, A:%.1f", 
                    env.temp,
                    env.humi,
                    env.pres,
                    env.altitude);

            temperature.value.float_value = env.temp;
            humidity.value.float_value = env.humi;

            homekit_characteristic_notify(&temperature, HOMEKIT_FLOAT(env.temp));
            homekit_characteristic_notify(&humidity, HOMEKIT_FLOAT(env.humi));
        } else {
            printf("Couldnt read data from sensor\n");
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void temperature_sensor_init() {
    xTaskCreate(temperature_sensor_task, "Temperatore Sensor", 1024 * 2, NULL, 5,NULL);
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            printf("STA start\n");
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            printf("WiFI ready\n");
            on_wifi_ready();
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            printf("STA disconnected\n");
            esp_wifi_connect();
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_init() {
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "max-inf",
            .password = "maxinf951!",
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_thermostat, .services=(homekit_service_t*[]) {
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "BME280"),
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "ShaoPu"),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "024A2BABF19D"),
            HOMEKIT_CHARACTERISTIC(MODEL, "MyTemperatureSensor"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, temperature_sensor_identify),
            NULL
        }),
        HOMEKIT_SERVICE(TEMPERATURE_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Temperature Sensor"),
            &temperature,
            NULL
        }),
        HOMEKIT_SERVICE(HUMIDITY_SENSOR, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Humidity Sensor"),
            &humidity,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "123-45-666",
    .setupId="1SP0",
};

void on_wifi_ready() {
    homekit_server_init(&config);
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_LOGI(TAG, "Version: %s", PROJECT_VER);

    wifi_init();
    dev_init();
    temperature_sensor_init();
}
