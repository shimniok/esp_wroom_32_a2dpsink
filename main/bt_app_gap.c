#include "bt_app_gap.h"

#include "esp_log.h"

/*******************************
 * STATIC FUNCTION DEFINITIONS
 ******************************/
void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  uint8_t *bda = NULL;

  switch (event) {
    /* when authentication completed, this event comes */
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
      if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
        ESP_LOGI(BT_GAP_TAG, "authentication success: %s",
                 param->auth_cmpl.device_name);
        esp_log_buffer_hex(BT_GAP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
      } else {
        ESP_LOGE(BT_GAP_TAG, "authentication failed, status: %d",
                 param->auth_cmpl.stat);
      }
      break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* when Security Simple Pairing user confirmation requested, this event
     * comes */
    case ESP_BT_GAP_CFM_REQ_EVT:
      ESP_LOGI(
          BT_GAP_TAG,
          "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %" PRIu32,
          param->cfm_req.num_val);
      esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
      break;
    /* when Security Simple Pairing passkey notified, this event comes */
    case ESP_BT_GAP_KEY_NOTIF_EVT:
      ESP_LOGI(BT_GAP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey: %" PRIu32,
               param->key_notif.passkey);
      break;
    /* when Security Simple Pairing passkey requested, this event comes */
    case ESP_BT_GAP_KEY_REQ_EVT:
      ESP_LOGI(BT_GAP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
      break;
#endif

    /* when GAP mode changed, this event comes */
    case ESP_BT_GAP_MODE_CHG_EVT:
      ESP_LOGI(BT_GAP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode: %d",
               param->mode_chg.mode);
      break;
    /* when ACL connection completed, this event comes */
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
      bda = (uint8_t *)param->acl_conn_cmpl_stat.bda;
      ESP_LOGI(BT_GAP_TAG,
               "ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT Connected to "
               "[%02x:%02x:%02x:%02x:%02x:%02x], status: 0x%x",
               bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
               param->acl_conn_cmpl_stat.stat);
      break;
    /* when ACL disconnection completed, this event comes */
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
      bda = (uint8_t *)param->acl_disconn_cmpl_stat.bda;
      ESP_LOGI(BT_GAP_TAG,
               "ESP_BT_GAP_ACL_DISC_CMPL_STAT_EVT Disconnected from "
               "[%02x:%02x:%02x:%02x:%02x:%02x], reason: 0x%x",
               bda[0], bda[1], bda[2], bda[3], bda[4], bda[5],
               param->acl_disconn_cmpl_stat.reason);
      break;
    /* others */
    default: {
      ESP_LOGI(BT_GAP_TAG, "event: %d", event);
      break;
    }
  }
}
