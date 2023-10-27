/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "bt_app_av.h"
#include "bt_app_core.h"
#include "bt_app_led.h"
#include "bt_app_stack.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

/*******************************
 * MAIN ENTRY POINT
 ******************************/

void app_main(void) {
  /* initialize NVS — it is used to store PHY calibration data */
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  /*
   * This example only uses the functions of Classical Bluetooth.
   * So release the controller memory for Bluetooth Low Energy.
   */
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
    ESP_LOGE(BT_AV_TAG, "%s initialize controller failed: %s\n", __func__,
             esp_err_to_name(err));
    return;
  }
  if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
    ESP_LOGE(BT_AV_TAG, "%s enable controller failed: %s\n", __func__,
             esp_err_to_name(err));
    return;
  }
  if ((err = esp_bluedroid_init()) != ESP_OK) {
    ESP_LOGE(BT_AV_TAG, "%s initialize bluedroid failed: %s\n", __func__,
             esp_err_to_name(err));
    return;
  }
  if ((err = esp_bluedroid_enable()) != ESP_OK) {
    ESP_LOGE(BT_AV_TAG, "%s enable bluedroid failed: %s\n", __func__,
             esp_err_to_name(err));
    return;
  }

  bt_stack_init();

  ui_status_task_startup();
  bt_app_task_start_up();
  /* bluetooth device name, connection mode and profile set up */
  bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);
}