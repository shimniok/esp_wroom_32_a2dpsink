#ifndef __BT_APP_LED__
#define __BT_APP_LED__

#include <stdint.h>

/**
 * UI status messages and count of statuses
 */
#define UI_STATUS_COUNT 4
#define UI_STATUS_NOT_CONNECTED 0
#define UI_STATUS_CONNECTING 1
#define UI_STATUS_CONNECTED 2
#define UI_STATUS_PLAYING UI_STATUS_CONNECTED
#define UI_STATUS_PAUSED 3

typedef uint8_t ui_status_t;

/**
 * @brief  update ui status task with new status
 */
void ui_update_status(ui_status_t status);

/**
 * @brief  start up the status diplay task
 */
void ui_status_task_startup(void);

#endif