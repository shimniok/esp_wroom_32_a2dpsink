#include "ui_display.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/xtensa_api.h"

////////////////////////////////////
//
// Variables
//
////////////////////////////////////
QueueHandle_t ui_queue;
const TickType_t dtime = 100 / portTICK_PERIOD_MS;
static TaskHandle_t s_status_th = NULL; /* task handle for status task */

////////////////////////////////////
//
// UI Status indicator Task
//
////////////////////////////////////

static void ui_status_task(void *arg) {
  uint8_t blink_pattern[UI_STATUS_COUNT][10] = {
      {1, 1, 1, 0, 0, 0, 0, 0, 0, 0},  // not connected
      {1, 1, 0, 0, 0, 1, 1, 0, 0, 0},  // connecting
      {1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // connected
      {1, 1, 1, 1, 1, 1, 1, 1, 0, 0}   // paused
  };
  ui_status_t msg;
  uint8_t status = 0;

  gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);
  gpio_set_drive_capability(GPIO_NUM_33, GPIO_DRIVE_CAP_3);
  gpio_set_level(GPIO_NUM_33, 1);

  uint16_t t = 0;
  while (1) {
    if (pdTRUE == xQueueReceive(ui_queue, &msg, 0)) {
      if (msg < UI_STATUS_COUNT) {
        status = msg;
      }
    }
    vTaskDelay(dtime);
    gpio_set_level(GPIO_NUM_33, blink_pattern[status][t]);
    t++;
    if (t >= 10) {
      t = 0;
    }
  }
}

void ui_update_status(ui_status_t status) {
  xQueueSend(ui_queue, &status, dtime);
}

void ui_status_task_startup(void) {
  ui_queue = xQueueCreate(10, sizeof(ui_status_t));
  xTaskCreate(ui_status_task, "uistatus", 2048, NULL, 3, &s_status_th);
}
