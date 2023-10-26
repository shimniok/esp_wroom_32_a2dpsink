#ifndef BT_APP_GAP_H_
#define BT_APP_GAP_H_

#define BT_GAP_TAG "BT_GAP"

#include "esp_gap_bt_api.h"

/* GAP callback function */
void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

#endif  // BT_APP_GAP_H_
