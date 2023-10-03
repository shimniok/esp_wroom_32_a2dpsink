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
static uint8_t bda[6];

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
             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    esp_a2d_sink_connect(bda);

    vTaskDelay(10000UL / portTICK_PERIOD_MS);
  }
}

void bt_autoconnect_task_startup(uint8_t *bda) {
  xTaskCreate(bt_autoconnect_task, "BtAutoconn", 2048, NULL, 3,
              &s_autoconnect_th);
}

void bt_autoconnect_task_shutdown(void) {
  if (s_autoconnect_th) {
    vTaskDelete(s_autoconnect_th);
    s_autoconnect_th = NULL;
  }
}
