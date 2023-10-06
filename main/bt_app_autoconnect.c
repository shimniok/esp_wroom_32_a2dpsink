#include "bt_app_autoconnect.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_a2dp_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h"

////////////////////////////////////
//
// Variables
//
////////////////////////////////////

/* bda for target of autoconnect */
static uint8_t mybda[6];

/* task handle for auto connect */
static TaskHandle_t s_autoconnect_th = NULL;

////////////////////////////////////
//
// Auto Connect Task
//
////////////////////////////////////
static void bt_autoconnect_task(void *arg) {
  while (1) {
    ESP_LOGI("BT_AUTOCONN",
             "attempting to connect to paired device "
             "[%02x:%02x:%02x:%02x:%02x:%02x]... ",
             mybda[0], mybda[1], mybda[2], mybda[3], mybda[4], mybda[5]);

    esp_a2d_sink_connect(mybda);

    vTaskDelay(10000UL / portTICK_PERIOD_MS);  // TODO: should this be one-shot?
  }
}

void bt_autoconnect_task_startup(uint8_t *bda) {
  if (bda) {
    for (int i = 0; i < 6; i++) {
      mybda[i] = bda[i];
    }
  }
  xTaskCreate(bt_autoconnect_task, "BtAutoconn", 2048, NULL, 3,
              &s_autoconnect_th);
}

void bt_autoconnect_task_shutdown(void) {
  if (s_autoconnect_th) {
    vTaskDelete(s_autoconnect_th);
    s_autoconnect_th = NULL;
  }
}
