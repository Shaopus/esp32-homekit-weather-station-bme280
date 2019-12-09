# [esp-homekit](https://github.com/maximkulkin/esp-homekit) 在 ESP32 上移植

[esp-homekit](https://github.com/maximkulkin/esp-homekit) 是 `Apple HomeKit` 配件服务器库，可以在 `esp8266` 和 `esp32` 上使用，以下是 [esp-homekit](https://github.com/maximkulkin/esp-homekit)  是基于 [esp-idf](https://github.com/espressif/esp-idf) 在 `esp32` 上的移植步骤。



## 组件

- [esp-homekit](https://github.com/maximkulkin/esp-homekit)
- [esp-wolfssl](https://github.com/maximkulkin/esp-wolfssl)
- [esp-http-parser](https://github.com/maximkulkin/esp-http-parser)



## 步骤

- 在`components`上下载以上组件，并重命名文件夹

  ```shell
  1. git clone --recursive https://github.com/maximkulkin/esp-homekit homekit
  2. git clone --recursive https://github.com/maximkulkin/esp-http-parser http-parser
  3. git clone --recursive https://github.com/maximkulkin/esp-wolfssl wolfssl
  ```

  

- 修改`makefile`文件

  ```shell
  CFLAGS += -DHOMEKIT_SHORT_APPLE_UUIDS
  
  include ./components/component_conf.mk
  ```

  

- 修改 `main` 文件下的 `component.mk`

  ```shell
  COMPONENT_DEPENDS := homekit
  ```

  

- 修改`led demo`，详细见目录下的 `led.c`

  ```c
  ...
  
  #include <homekit/homekit.h>
  #include <homekit/characteristics.h>
  ...
      
  homekit_accessory_t *accessories[] = {
      HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_lightbulb, .services=(homekit_service_t*[]){
          HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
              HOMEKIT_CHARACTERISTIC(NAME, "Sample LED"),
              HOMEKIT_CHARACTERISTIC(MANUFACTURER, "HaPK"),
              HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "025A2BABF19D"),
              HOMEKIT_CHARACTERISTIC(MODEL, "MyLED"),
              HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1"),
              HOMEKIT_CHARACTERISTIC(IDENTIFY, led_identify),
              NULL
          }),
          HOMEKIT_SERVICE(LIGHTBULB, .primary=true, .characteristics=(homekit_characteristic_t*[]){
              HOMEKIT_CHARACTERISTIC(NAME, "Sample LED"),
              HOMEKIT_CHARACTERISTIC(
                  ON, false,
                  .getter=led_on_get,
                  .setter=led_on_set
              ),
              NULL
          }),
          NULL
      }),
      NULL
  };
  
  homekit_server_config_t config = {
      .accessories = accessories,
      .password = "111-11-111",
      .setupId="1SP0",
  };
  
  void on_wifi_ready() {
      homekit_server_init(&config);
  }
  
  ...
  ```

  

## 参考

- 参考 [esp32-homekit-camera](https://github.com/maximkulkin/esp32-homekit-camera) 中的 `makefile` 和 项目目录