/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2025 Jeongyeham
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_lcd_panel_st7789.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "iot_sensor_hub.h"
#include <inttypes.h>
#include <stdio.h>
#include "mqtt5.h"

static const int CONNECTED_BIT = BIT0;
static const int SC_DONE_BIT = BIT1;
static const int NVS_STORED_BIT = BIT2;


/////////////////////////////////////////////////////////////////////////SENSOR_HUB////////////////////////////////////////////////////////////////
#define I2C_MASTER_SCL_IO 33      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 34      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static i2c_bus_handle_t i2c_bus = NULL;
static sensor_handle_t sht4x_handle = NULL;
static sensor_event_handler_instance_t sht4x_handler_handle = NULL;
static void sensor_event_handler(void *handler_args, esp_event_base_t base,
                                 int32_t id, void *event_data);

static const char *sensor_task_TAG = "sensor_hub_task";

static void sensor_event_handler(void *handler_args, esp_event_base_t base,
                                 int32_t id, void *event_data) {
  sensor_data_t *sensor_data = (sensor_data_t *)event_data;

  switch (id) {
  case SENSOR_STARTED:
    ESP_LOGI(sensor_task_TAG, "Timestamp = %llu - %s_0x%x STARTED",
             sensor_data->timestamp, sensor_data->sensor_name,
             sensor_data->sensor_addr);
    break;
  case SENSOR_STOPED:
    ESP_LOGI(sensor_task_TAG, "Timestamp = %llu - %s_0x%x STOPPED",
             sensor_data->timestamp, sensor_data->sensor_name,
             sensor_data->sensor_addr);
    break;
  case SENSOR_HUMI_DATA_READY:
    ESP_LOGI(sensor_task_TAG,
             "Timestamp = %llu - %s_0x%x HUMI_DATA_READY - "
             "humiture=%.2f",
             sensor_data->timestamp, sensor_data->sensor_name,
             sensor_data->sensor_addr, sensor_data->humidity);
    break;
  case SENSOR_TEMP_DATA_READY:
    ESP_LOGI(sensor_task_TAG,
             "Timestamp = %llu - %s_0x%x TEMP_DATA_READY - "
             "temperature=%.2f\n",
             sensor_data->timestamp, sensor_data->sensor_name,
             sensor_data->sensor_addr, sensor_data->temperature);
    break;
  case SENSOR_ACCE_DATA_READY:
    ESP_LOGI(sensor_task_TAG,
             "Timestamp = %llu - %s_0x%x ACCE_DATA_READY - "
             "acce_x=%.2f, acce_y=%.2f, acce_z=%.2f\n",
             sensor_data->timestamp, sensor_data->sensor_name,
             sensor_data->sensor_addr, sensor_data->acce.x, sensor_data->acce.y,
             sensor_data->acce.z);
    break;
  case SENSOR_GYRO_DATA_READY:
    ESP_LOGI(sensor_task_TAG,
             "Timestamp = %llu - %s_0x%x GYRO_DATA_READY - "
             "gyro_x=%.2f, gyro_y=%.2f, gyro_z=%.2f\n",
             sensor_data->timestamp, sensor_data->sensor_name,
             sensor_data->sensor_addr, sensor_data->gyro.x, sensor_data->gyro.y,
             sensor_data->gyro.z);
    break;
  case SENSOR_LIGHT_DATA_READY:
    ESP_LOGI(sensor_task_TAG,
             "Timestamp = %llu - %s_0x%x LIGHT_DATA_READY - "
             "light=%.2f",
             sensor_data->timestamp, sensor_data->sensor_name,
             sensor_data->sensor_addr, sensor_data->light);
    break;
  case SENSOR_RGBW_DATA_READY:
    ESP_LOGI(sensor_task_TAG,
             "Timestamp = %llu - %s_0x%x RGBW_DATA_READY - "
             "r=%.2f, g=%.2f, b=%.2f, w=%.2f\n",
             sensor_data->timestamp, sensor_data->sensor_name,
             sensor_data->sensor_addr, sensor_data->rgbw.r, sensor_data->rgbw.r,
             sensor_data->rgbw.b, sensor_data->rgbw.w);
    break;
  case SENSOR_UV_DATA_READY:
    ESP_LOGI(sensor_task_TAG,
             "Timestamp = %llu - %s_0x%x UV_DATA_READY - "
             "uv=%.2f, uva=%.2f, uvb=%.2f\n",
             sensor_data->timestamp, sensor_data->sensor_name,
             sensor_data->sensor_addr, sensor_data->uv.uv, sensor_data->uv.uva,
             sensor_data->uv.uvb);
    break;
  default:
    ESP_LOGI(sensor_task_TAG, "Timestamp = %" PRIi64 " - event id = %" PRIi32,
             sensor_data->timestamp, id);
    break;
  }
}

void sensor_hub_task(void *pvParameters) {

  const i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);

  const sensor_config_t sht4x_config = {
      .bus = i2c_bus,
      .addr = 0x44,
      .type = HUMITURE_ID,
      .mode = MODE_POLLING,
      .min_delay = 1000,
  };
  iot_sensor_create("sht4x", &sht4x_config, &sht4x_handle);

  sensors_event_loop_create();

  iot_sensor_handler_register(sht4x_handle, sensor_event_handler,
                              &sht4x_handler_handle);

  iot_sensor_start(sht4x_handle);

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////LVGL_UI///////////////////////////////////////////////////////

static const char *lvgl_task_TAG = "lvgl_task";
#define DISP_WIDTH 120
#define DISP_HEIGHT 120

static esp_lcd_panel_io_spi_config_t io_config = {};
static esp_lcd_panel_dev_config_t panel_config = {};

void lvgl_ui_task(void *pvParameters) {

  const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
  esp_err_t err = lvgl_port_init(&lvgl_cfg);

  static lv_disp_t *disp_handle;

  /* LCD IO */
  esp_lcd_panel_io_handle_t io_handle = NULL;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)1,
                                           &io_config, &io_handle));

  /* LCD driver initialization */
  esp_lcd_panel_handle_t lcd_panel_handle;
  ESP_ERROR_CHECK(
      esp_lcd_new_panel_st7789(io_handle, &panel_config, &lcd_panel_handle));

  /* Add LCD screen */
  const lvgl_port_display_cfg_t disp_cfg = {
      .io_handle = io_handle,
      .panel_handle = lcd_panel_handle,
      .buffer_size = DISP_WIDTH * DISP_HEIGHT,
      .double_buffer = true,
      .hres = DISP_WIDTH,
      .vres = DISP_HEIGHT,
      .monochrome = false,
      .color_format = LV_COLOR_FORMAT_RGB565,
      .rotation =
          {
              .swap_xy = false,
              .mirror_x = false,
              .mirror_y = false,
          },
      .flags = {
          .buff_dma = true,
          .swap_bytes = false,
      }};
  disp_handle = lvgl_port_add_disp(&disp_cfg);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////MQTT5///////////////////////////////////////////////////////

static const char *mqtt5_task_TAG = "mqtt5_task";

void mqtt5_start_task(void *pvParameters) {
  EventGroupHandle_t s_wifi_event_group = (EventGroupHandle_t) pvParameters;

  ESP_LOGI(mqtt5_task_TAG, "[APP] Startup..");
  ESP_LOGI(mqtt5_task_TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
  ESP_LOGI(mqtt5_task_TAG, "[APP] IDF version: %s", esp_get_idf_version());

  esp_log_level_set("*", ESP_LOG_INFO);
  esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
  esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
  esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
  esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
  esp_log_level_set("transport", ESP_LOG_VERBOSE);
  esp_log_level_set("outbox", ESP_LOG_VERBOSE);
  while (1) {
    EventBits_t uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, true, false,
                                             portMAX_DELAY);
    if (likely(uxBits & CONNECTED_BIT)) {
      ESP_LOGI(mqtt5_task_TAG, "WiFi Connected to ap");
      mqtt5_app_config();
      break;
    }
    ESP_LOGE(mqtt5_task_TAG, "Waiting Net");
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  vTaskDelete(NULL);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////SMART_CONFIG////////////////////////////////////////////////////////
#include <memory.h>
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include "esp_log.h"



/*NVS_FLASH Configuration */
static const char *NVS_Name_space = "wifi_data";
static const char *NVS_Key = "key_wifi_data";
static nvs_handle_t wifi_nvs_handle;
wifi_config_t *wifi_config_stored;

static const char *TAG = "smart_config";

void smart_config_task(void *pvParameters);

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    EventGroupHandle_t *s_wifi_event_group = (EventGroupHandle_t *) arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (ESP_ERR_WIFI_SSID == esp_wifi_connect()) {
            ESP_ERROR_CHECK(nvs_open(NVS_Name_space, NVS_READWRITE, &wifi_nvs_handle));
            ESP_ERROR_CHECK(nvs_erase_key(wifi_nvs_handle, NVS_Key));
            ESP_ERROR_CHECK(nvs_erase_all(wifi_nvs_handle));
            ESP_ERROR_CHECK(nvs_commit(wifi_nvs_handle));
            nvs_close(wifi_nvs_handle);

        }
        if (pdPASS == xEventGroupClearBitsFromISR(s_wifi_event_group, CONNECTED_BIT)) {
            portYIELD_FROM_ISR(pdTRUE);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        BaseType_t xHigherPriorityTaskWoken = pdFAIL;
        if (pdPASS == xEventGroupSetBitsFromISR(s_wifi_event_group, CONNECTED_BIT, &xHigherPriorityTaskWoken)) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG, "Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *) event_data;

        memset(wifi_config_stored, 0x0, sizeof(wifi_config_t));

        memcpy(wifi_config_stored->sta.ssid, evt->ssid, sizeof(wifi_config_stored->sta.ssid));
        memcpy(wifi_config_stored->sta.password, evt->password, sizeof(wifi_config_stored->sta.password));

        wifi_config_stored->sta.bssid_set = evt->bssid_set;
        if (wifi_config_stored->sta.bssid_set == true) {
            memcpy(wifi_config_stored->sta.bssid, evt->bssid, sizeof(wifi_config_stored->sta.bssid));
        }

        ESP_LOGI(TAG, "SSID:%s", wifi_config_stored->sta.ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", wifi_config_stored->sta.password);


        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, wifi_config_stored));

        ESP_ERROR_CHECK(nvs_open(NVS_Name_space, NVS_READWRITE, &wifi_nvs_handle));
        ESP_ERROR_CHECK(nvs_set_blob(wifi_nvs_handle, NVS_Key, wifi_config_stored, sizeof(wifi_config_t)));
        ESP_ERROR_CHECK(nvs_commit(wifi_nvs_handle));

        nvs_close(wifi_nvs_handle);
        vPortFree(wifi_config_stored);

        ESP_ERROR_CHECK(esp_wifi_connect());
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        ESP_LOGI(TAG, "SC_ACK_DONE");
        BaseType_t xHigherPriorityTaskWoken;
        if (pdPASS == xEventGroupSetBitsFromISR(s_wifi_event_group, SC_DONE_BIT, &xHigherPriorityTaskWoken)) {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void wifi_task(void *pvParameters) {
    EventGroupHandle_t s_wifi_event_group = (EventGroupHandle_t) pvParameters;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, s_wifi_event_group));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, s_wifi_event_group));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, s_wifi_event_group));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(nvs_open(NVS_Name_space, NVS_READWRITE, &wifi_nvs_handle));

    wifi_config_stored = pvPortMalloc(sizeof(wifi_config_t));
    memset(wifi_config_stored, 0x0, sizeof(wifi_config_t));

    uint32_t len = sizeof(wifi_config_t);
    esp_err_t ret = nvs_get_blob(wifi_nvs_handle, NVS_Key, wifi_config_stored, (size_t *) &len);

    if (likely(ret == ESP_OK)) {
        xEventGroupSetBits(s_wifi_event_group, NVS_STORED_BIT);
        nvs_close(wifi_nvs_handle);

        ESP_ERROR_CHECK(esp_wifi_set_config((wifi_interface_t) ESP_IF_WIFI_STA, wifi_config_stored));
        vPortFree(wifi_config_stored);

    } else {
        xEventGroupClearBits(s_wifi_event_group, NVS_STORED_BIT);
        xTaskCreate(smart_config_task, "smart_config_task", 4096, s_wifi_event_group, 3, NULL);
        nvs_close(wifi_nvs_handle);
    }

    ESP_ERROR_CHECK(esp_wifi_start());

    vTaskDelete(NULL);
}

void smart_config_task(void *pvParameters) {
    EventGroupHandle_t s_wifi_event_group = (EventGroupHandle_t) pvParameters;

    ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
    while (1) {
        EventBits_t uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | SC_DONE_BIT, true, false,
                                                 portMAX_DELAY);
        if (likely(uxBits & CONNECTED_BIT)) {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if (likely(uxBits & SC_DONE_BIT)) {
            ESP_LOGI(TAG, "Smart_config Function over");
            esp_smartconfig_stop();
            break;
        }
    }
    vTaskDelete(NULL);
}
