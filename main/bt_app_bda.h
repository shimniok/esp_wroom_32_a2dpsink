#ifndef __BT_APP_BDA_H__
#define __BT_APP_BDA_H__

#include <stdbool.h>
#include <stdint.h>

/* Application NVS storage of paired BDA */
#define BT_APP_PART "nvs"
#define BT_APP_NS "bt_app_bda"
#define BT_APP_BDA_KEY "bda"
#define BT_APP_BDA_LEN 6

#define BT_BDA_TAG "BT_BDA"

/**
 * @brief  read saved bda from nvs
 */
bool nvs_read_bda(uint8_t *bda);

/**
 * @brief  update bda to nvs if it has changed
 */
void nvs_update_bda(uint8_t *bda);

#endif