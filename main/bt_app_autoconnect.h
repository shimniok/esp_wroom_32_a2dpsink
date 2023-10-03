#ifndef __BT_APP_AUTOCONNECT_H__
#define __BT_APP_AUTOCONNECT_H__

#include <stdint.h>

/**
 * @brief  start up the auto connect task
 * @param bda target address to autoconnect to
 */
void bt_autoconnect_task_startup(uint8_t *bda);

/**
 * @brief  shut down the auto connect task
 */
void bt_autoconnect_task_shutdown(void);

#endif