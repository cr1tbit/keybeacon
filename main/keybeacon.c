/*
   This example code is whatever licensed just steal it

   Based on ESP IDF's iBeacon example

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

static const char* DEMO_TAG = "KEYBROADCASTER";

static uint8_t x_row_pins[] = {2,4,16,17,5,18,19,21};
static uint8_t y_row_pins[] = {13,12,14,27,26,25,33};

static uint8_t x_row_len = sizeof(x_row_pins)/sizeof(x_row_pins[0]);
static uint8_t y_row_len = sizeof(y_row_pins)/sizeof(y_row_pins[0]);

void km_init(){
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 0b0;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    for (int i=0;i<x_row_len;i++){
        io_conf.pin_bit_mask |= 1ULL<<x_row_pins[i];
    }
    gpio_config(&io_conf);

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 0b0;
    io_conf.pull_down_en = 1;
    io_conf.pull_up_en = 0;

    for (int i=0;i<y_row_len;i++){
        io_conf.pin_bit_mask  |= 1ULL<<y_row_pins[i];
    }
    gpio_config(&io_conf);
}

void km_set_pin_from_array(uint8_t *pinArray,uint8_t pinArraySize,uint8_t pinNo){
  for (uint8_t i=0;i<pinArraySize; i++){
    if (i == pinNo){
      gpio_set_level(pinArray[i],1);
    }
    else{
      gpio_set_level(pinArray[i],0);
    }
      
  }
}

int8_t km_get_pressed_key(uint8_t *pinArray, uint8_t pinArraySize){
  int8_t pin_pressed = -1;
  for (int i=0;i<pinArraySize; i++){
    uint8_t state = gpio_get_level(pinArray[i]);
    if (state){
        pin_pressed = i;
    }
    //printf("%d",state);
  }
  //printf("|\n\r");
  return pin_pressed;
}

// the loop routine runs over and over again forever:
int8_t get_keyboard_state() {
  for (int x = 0;x<x_row_len;x++){
    //set X pin
    km_set_pin_from_array(x_row_pins,x_row_len,x);
    vTaskDelay(30/portTICK_PERIOD_MS);
    
    //get Y pins
    int8_t key_pressed = km_get_pressed_key(y_row_pins,y_row_len);
    if (key_pressed!=-1){
        return (x*x_row_len)+key_pressed;
    }
  }
  //printf("===========\n\r");
  return -1;
}

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = 0x040,
    .adv_int_max        = 0x040,
    .adv_type           = ADV_TYPE_NONCONN_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:{
        esp_ble_gap_start_advertising(&ble_adv_params);
        break;
    }
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(DEMO_TAG, "Scan start failed: %s", esp_err_to_name(err));
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //adv start complete event to indicate adv start successfully or failed
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(DEMO_TAG, "Adv start failed: %s", esp_err_to_name(err));
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(DEMO_TAG, "Scan stop failed: %s", esp_err_to_name(err));
        }
        else {
            ESP_LOGI(DEMO_TAG, "Stop scan successfully");
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(DEMO_TAG, "Adv stop failed: %s", esp_err_to_name(err));
        }
        else {
            ESP_LOGI(DEMO_TAG, "Stop adv successfully");
        }
        break;

    default:
        break;
    }
}

void ble_beacon_appRegister(void)
{
    esp_err_t status;

    ESP_LOGI(DEMO_TAG, "register callback");

    //register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(DEMO_TAG, "gap register error: %s", esp_err_to_name(status));
        return;
    }
}

void ble_beacon_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    ble_beacon_appRegister();
}


void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    ble_beacon_init();

    km_init();

    uint8_t dummy_data[] = {0x02, //flags section length 
                            0x01, //flags section identifier
                            0x06, //ADV packet flags

                            0x05, //custom data section length
                            0xFF, //custom data identifier
                            0xE5, 0x02, //company identifier (using espressif's ID here) 
                            0xDE, 0xAD // custom payload
                            };

    uint8_t coda = 0;
    
    while (1){
        //replace last payload byte with key gotten from keyboard
        dummy_data[8] = get_keyboard_state();
        //update advertised data
        esp_ble_gap_config_adv_data_raw(&dummy_data, sizeof(dummy_data));
        //vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

