#include "bt_app_stack.h"

#include "bt_app_av.h"
#include "bt_app_core.h"
#include "bt_app_display.h"
#include "bt_app_gap.h"
#include "bt_app_stack.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_log.h"
#include "esp_system.h"

/* device name */
#define LOCAL_DEVICE_NAME "ESP_SPEAKER"

void bt_stack_init(void) {
#if (CONFIG_BT_SSP_ENABLED == true)
  /* set default parameters for Secure Simple Pairing */
  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
  esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

  /* set default parameters for Legacy Pairing (use fixed pin code 1234) */
  esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
  esp_bt_pin_code_t pin_code;
  pin_code[0] = '1';
  pin_code[1] = '2';
  pin_code[2] = '3';
  pin_code[3] = '4';
  esp_bt_gap_set_pin(pin_type, 4, pin_code);
}

void bt_av_hdl_stack_evt(uint16_t event, void *p_param) {
  ESP_LOGD(BT_STACK_TAG, "%s event: %d", __func__, event);

  switch (event) {
    /* when do the stack up, this event comes */
    case BT_APP_EVT_STACK_UP: {
      esp_bt_dev_set_device_name(LOCAL_DEVICE_NAME);
      esp_bt_gap_register_callback(bt_app_gap_cb);

      // Initialize avrcp controller
      assert(esp_avrc_ct_init() == ESP_OK);
      // Register controller callback
      esp_avrc_ct_register_callback(bt_app_rc_ct_cb);

      // Initialize avrcp target
      assert(esp_avrc_tg_init() == ESP_OK);
      // Register target callback
      esp_avrc_tg_register_callback(bt_app_rc_tg_cb);

      // Set target capabilities
      esp_avrc_rn_evt_cap_mask_t evt_set = {0};
      esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set,
                                         ESP_AVRC_RN_VOLUME_CHANGE);
      assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK);

      assert(esp_a2d_sink_init() == ESP_OK);
      esp_a2d_register_callback(&bt_app_a2d_cb);
      esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);

      /* Get the default value of the delay value */
      esp_a2d_sink_get_delay_value();

      /* set discoverable and connectable mode, wait to be connected */
      esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
      break;
    }
    /* others */
    default:
      ESP_LOGE(BT_STACK_TAG, "%s unhandled event: %d", __func__, event);
      break;
  }
}
