// Copyright 2020-2021 Espressif Systems (Shanghai) PTE LTD
// Copyright 2022 JeongYeham
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory.h>
#include "smart_config.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_smartconfig.h"
#include "esp_log.h"


static const int CONNECTED_BIT = BIT0;
static const int SC_DONE_BIT = BIT1;
static const int NVS_STORED_BIT = BIT2;

/*NVS_FLASH Configuration */
static const char *NVS_Name_space = "wifi_data";
static const char *NVS_Key = "key_wifi_data";
static nvs_handle_t wifi_nvs_handle;
wifi_config_t *wifi_config_stored;

static const char *TAG = "smart_config";

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
