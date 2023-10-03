/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "bt_app_av.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "bt_app_autoconnect.h"
#include "bt_app_core.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "ui_display.h"
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
#include "driver/dac_continuous.h"
#else
#include "driver/i2s_std.h"
#endif

#include "sys/lock.h"

/* AVRCP used transaction labels */
#define APP_RC_CT_TL_GET_CAPS (0)
#define APP_RC_CT_TL_GET_META_DATA (1)
#define APP_RC_CT_TL_RN_TRACK_CHANGE (2)
#define APP_RC_CT_TL_RN_PLAYBACK_CHANGE (3)
#define APP_RC_CT_TL_RN_PLAY_POS_CHANGE (4)

/* Application layer causes delay value */
#define APP_DELAY_VALUE 50  // 5ms

/* Application NVS storage of paired BDA */
#define BT_APP_PART "nvs"
#define BT_APP_NS "bt_app_bda"
#define BT_APP_BDA_KEY "bda"
#define BT_APP_BDA_LEN 6

/*******************************
 * STATIC FUNCTION DECLARATIONS
 ******************************/

/* volume control conversion */
// static uint8_t vol_to_log(uint8_t volume);
/* allocate new meta buffer */
static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param);
/* handler for new track is loaded */
static void bt_av_new_track(void);
/* handler for track status change */
static void bt_av_playback_changed(void);
/* handler for track playing position change */
static void bt_av_play_pos_changed(void);
/* notification event handler */
static void bt_av_notify_evt_handler(uint8_t event_id,
                                     esp_avrc_rn_param_t *event_parameter);
/* installation for i2s */
static void bt_i2s_driver_install(void);
/* uninstallation for i2s */
static void bt_i2s_driver_uninstall(void);
/* set volume by remote controller */
static void volume_set_by_controller(uint8_t volume);
/* notify the target we've pressed play */
static void ct_press_play();
/* a2dp event handler */
static void bt_av_hdl_a2d_evt(uint16_t event, void *p_param);
/* avrc controller event handler */
static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param);
/* avrc target event handler */
static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param);
/* autoconnect callback */
void bt_connect_cb(void);
/* compare two bdas for equality */
bool bda_equal(uint8_t *bda1, uint8_t *bda2);
/* read bda from nvs */
bool nvs_read_bda(uint8_t *bda);
/* write bda to nvs */
void nvs_update_bda(uint8_t *bda);

/*******************************
 * STATIC VARIABLE DEFINITIONS
 ******************************/

static uint32_t s_pkt_cnt = 0; /* count for audio packet */
static esp_a2d_audio_state_t s_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;
/* audio stream datapath state */
static const char *s_a2d_conn_state_str[] = {"Disconnected", "Connecting",
                                             "Connected", "Disconnecting"};
/* connection state in string */
static const char *s_a2d_audio_state_str[] = {"Suspended", "Stopped",
                                              "Started"};
/* audio stream datapath state in string */
static esp_avrc_rn_evt_cap_mask_t s_avrc_peer_rn_cap;
/* AVRC target notification capability bit mask */
static _lock_t s_volume_lock;
// static TaskHandle_t s_vcs_task_hdl = NULL;    /* handle for volume change
// simulation task */
static uint8_t s_volume = 0; /* local volume value */
static bool s_volume_notify; /* notify volume change or not */
static bool s_play_notify;
#ifndef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
i2s_chan_handle_t tx_chan = NULL;
#else
dac_continuous_handle_t tx_chan;
#endif

/********************************
 * STATIC FUNCTION DEFINITIONS
 *******************************/

// convert linear 0-100 volume to logarithmic
// static uint8_t vol_to_log(uint8_t volume)
// {
//     if (volume < 80) {
//         return volume / 4;
//     } else {
//         return 100;
//     }
// }

static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param) {
  esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
  uint8_t *attr_text = (uint8_t *)malloc(rc->meta_rsp.attr_length + 1);

  memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
  attr_text[rc->meta_rsp.attr_length] = 0;
  rc->meta_rsp.attr_text = attr_text;
}

////////////////////////////////////
//
// AV
//
////////////////////////////////////

static void bt_av_new_track(void) {
  /* request metadata */
  uint8_t attr_mask = ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST |
                      ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_GENRE;
  esp_avrc_ct_send_metadata_cmd(APP_RC_CT_TL_GET_META_DATA, attr_mask);

  /* register notification if peer (controller) supports the event_id */
  if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST,
                                         &s_avrc_peer_rn_cap,
                                         ESP_AVRC_RN_TRACK_CHANGE)) {
    esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_TRACK_CHANGE,
                                               ESP_AVRC_RN_TRACK_CHANGE, 0);
  }
}

static void bt_av_playback_changed(void) {
  /* register notification if peer (controller) support the event_id */
  if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST,
                                         &s_avrc_peer_rn_cap,
                                         ESP_AVRC_RN_PLAY_STATUS_CHANGE)) {
    esp_avrc_ct_send_register_notification_cmd(
        APP_RC_CT_TL_RN_PLAYBACK_CHANGE, ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
  }
}

static void bt_av_play_pos_changed(void) {
  /* register notification if peer (controller) support the event_id */
  if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST,
                                         &s_avrc_peer_rn_cap,
                                         ESP_AVRC_RN_PLAY_POS_CHANGED)) {
    esp_avrc_ct_send_register_notification_cmd(
        APP_RC_CT_TL_RN_PLAY_POS_CHANGE, ESP_AVRC_RN_PLAY_POS_CHANGED, 10);
  }
}

////////////////////////////////////
//
// AV Event Handler
//
////////////////////////////////////

static void bt_av_notify_evt_handler(uint8_t event_id,
                                     esp_avrc_rn_param_t *event_parameter) {
  switch (event_id) {
    /* when new track is loaded, this event comes */
    case ESP_AVRC_RN_TRACK_CHANGE:
      bt_av_new_track();
      break;
    /* when track status changed, this event comes */
    case ESP_AVRC_RN_PLAY_STATUS_CHANGE:
      ESP_LOGI(BT_AV_TAG, "Playback status changed: 0x%x",
               event_parameter->playback);
      bt_av_playback_changed();
      if (event_parameter->playback == ESP_AVRC_PLAYBACK_PAUSED) {
        ui_update_status(UI_STATUS_PAUSED);
      } else if (event_parameter->playback == ESP_AVRC_PLAYBACK_PLAYING) {
        ui_update_status(UI_STATUS_PLAYING);
      }
      break;
    /* when track playing position changed, this event comes */
    case ESP_AVRC_RN_PLAY_POS_CHANGED:
      ESP_LOGI(BT_AV_TAG, "Play position changed: %" PRIu32 "-ms",
               event_parameter->play_pos);
      bt_av_play_pos_changed();
      break;
    /* others */
    default:
      ESP_LOGI(BT_AV_TAG, "unhandled event: %d", event_id);
      break;
  }
}

////////////////////////////////////
//
// I2S
//
////////////////////////////////////
void bt_i2s_driver_install(void) {
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
  dac_continuous_config_t cont_cfg = {
      .chan_mask = DAC_CHANNEL_MASK_ALL,
      .desc_num = 8,
      .buf_size = 2048,
      .freq_hz = 44100,
      .offset = 127,
      .clk_src = DAC_DIGI_CLK_SRC_DEFAULT,  // Using APLL as clock source to get
                                            // a wider frequency range
      .chan_mode = DAC_CHANNEL_MODE_ALTER,
  };
  /* Allocate continuous channels */
  ESP_ERROR_CHECK(dac_continuous_new_channels(&cont_cfg, &tx_chan));
  /* Enable the continuous channels */
  ESP_ERROR_CHECK(dac_continuous_enable(tx_chan));
#else
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
      .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                  I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = I2S_GPIO_UNUSED,
              .bclk = CONFIG_EXAMPLE_I2S_BCK_PIN,
              .ws = CONFIG_EXAMPLE_I2S_LRCK_PIN,
              .dout = CONFIG_EXAMPLE_I2S_DATA_PIN,
              .din = I2S_GPIO_UNUSED,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };
  /* enable I2S */
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
#endif
}

void bt_i2s_driver_uninstall(void) {
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
  ESP_ERROR_CHECK(dac_continuous_disable(tx_chan));
  ESP_ERROR_CHECK(dac_continuous_del_channels(tx_chan));
#else
  ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));
  ESP_ERROR_CHECK(i2s_del_channel(tx_chan));
#endif
}

////////////////////////////////////
//
// Controller Actions
//
////////////////////////////////////
static void volume_set_by_controller(uint8_t volume) {
  ESP_LOGI(BT_RC_TG_TAG,
           "Volume is set by remote controller to: %" PRIu32 "%% (%d)",
           (uint32_t)volume * 100 / 0x7f, volume);
  /* set the volume in protection of lock */
  _lock_acquire(&s_volume_lock);
  s_volume = volume;
  _lock_release(&s_volume_lock);
}

static void ct_press_play() {
  // Attempt to start playing
  ESP_LOGI(BT_RC_TG_TAG, ">>>>>>>>>>> Attempting to start playing");
  esp_avrc_ct_send_passthrough_cmd(0, ESP_AVRC_PT_CMD_PLAY,
                                   ESP_AVRC_PT_CMD_STATE_PRESSED);
  esp_avrc_ct_send_passthrough_cmd(1, ESP_AVRC_PT_CMD_PLAY,
                                   ESP_AVRC_PT_CMD_STATE_RELEASED);
}

// TODO: button press to erase pairing

////////////////////////////////////
//
// A2D Event Handler
//
////////////////////////////////////
static void bt_av_hdl_a2d_evt(uint16_t event, void *p_param) {
  ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

  esp_a2d_cb_param_t *a2d = NULL;

  switch (event) {
    /* when connection state changed, this event comes */
    case ESP_A2D_CONNECTION_STATE_EVT: {
      a2d = (esp_a2d_cb_param_t *)(p_param);
      uint8_t *bda = a2d->conn_stat.remote_bda;
      ESP_LOGI(BT_AV_TAG,
               "A2DP connection state: %s, [%02x:%02x:%02x:%02x:%02x:%02x]",
               s_a2d_conn_state_str[a2d->conn_stat.state], bda[0], bda[1],
               bda[2], bda[3], bda[4], bda[5]);
      if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
        ui_update_status(UI_STATUS_NOT_CONNECTED);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                 ESP_BT_GENERAL_DISCOVERABLE);
        bt_i2s_driver_uninstall();
        bt_i2s_task_shut_down();
        // auto connect but only on first boot?
        // begin polling in attempt to connect?
      } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        ui_update_status(UI_STATUS_CONNECTED);
        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE,
                                 ESP_BT_NON_DISCOVERABLE);
        bt_i2s_task_start_up();
        bt_autoconnect_task_shutdown();
        nvs_update_bda(bda);
      } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
        ui_update_status(UI_STATUS_CONNECTING);
        bt_i2s_driver_install();
      }
      break;
    }
    /* when audio stream transmission state changed, this event comes */
    case ESP_A2D_AUDIO_STATE_EVT: {
      a2d = (esp_a2d_cb_param_t *)(p_param);
      ESP_LOGI(BT_AV_TAG, "A2DP audio state: %s",
               s_a2d_audio_state_str[a2d->audio_stat.state]);
      s_audio_state = a2d->audio_stat.state;
      if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
        s_pkt_cnt = 0;
      }
      break;
    }
    /* when audio codec is configured, this event comes */
    case ESP_A2D_AUDIO_CFG_EVT: {
      a2d = (esp_a2d_cb_param_t *)(p_param);
      ESP_LOGI(BT_AV_TAG, "A2DP audio stream configuration, codec type: %d",
               a2d->audio_cfg.mcc.type);
      /* for now only SBC stream is supported */
      if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
        int sample_rate = 16000;
        int ch_count = 2;
        char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
        if (oct0 & (0x01 << 6)) {
          sample_rate = 32000;
        } else if (oct0 & (0x01 << 5)) {
          sample_rate = 44100;
        } else if (oct0 & (0x01 << 4)) {
          sample_rate = 48000;
        }

        if (oct0 & (0x01 << 3)) {
          ch_count = 1;
        }
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
        dac_continuous_disable(tx_chan);
        dac_continuous_del_channels(tx_chan);
        dac_continuous_config_t cont_cfg = {
            .chan_mask = DAC_CHANNEL_MASK_ALL,
            .desc_num = 8,
            .buf_size = 2048,
            .freq_hz = sample_rate,
            .offset = 127,
            .clk_src =
                DAC_DIGI_CLK_SRC_DEFAULT,  // Using APLL as clock source to
                                           // get a wider frequency range
            .chan_mode = (ch_count == 1) ? DAC_CHANNEL_MODE_SIMUL
                                         : DAC_CHANNEL_MODE_ALTER,
        };
        /* Allocate continuous channels */
        dac_continuous_new_channels(&cont_cfg, &tx_chan);
        /* Enable the continuous channels */
        dac_continuous_enable(tx_chan);
#else
        i2s_channel_disable(tx_chan);
        i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
        i2s_std_slot_config_t slot_cfg =
            I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, ch_count);
        slot_cfg.bit_shift = true;  // required for PCM5102 I2S format
        i2s_channel_reconfig_std_clock(tx_chan, &clk_cfg);
        i2s_channel_reconfig_std_slot(tx_chan, &slot_cfg);
        i2s_channel_enable(tx_chan);
#endif
        ESP_LOGI(BT_AV_TAG, "Configure audio player: %x-%x-%x-%x",
                 a2d->audio_cfg.mcc.cie.sbc[0], a2d->audio_cfg.mcc.cie.sbc[1],
                 a2d->audio_cfg.mcc.cie.sbc[2], a2d->audio_cfg.mcc.cie.sbc[3]);
        ESP_LOGI(BT_AV_TAG, "Audio player configured, sample rate: %d",
                 sample_rate);
      }
      break;
    }
    /* when a2dp init or deinit completed, this event comes */
    case ESP_A2D_PROF_STATE_EVT: {
      a2d = (esp_a2d_cb_param_t *)(p_param);
      if (ESP_A2D_INIT_SUCCESS == a2d->a2d_prof_stat.init_state) {
        ESP_LOGI(BT_AV_TAG, "A2DP PROF STATE: Init Complete");
        uint8_t bda[6];
        if (nvs_read_bda(bda)) {
          bt_autoconnect_task_startup(bda);
        }
      } else {
        ESP_LOGI(BT_AV_TAG, "A2DP PROF STATE: Deinit Complete");
      }
      break;
    }
    /* When protocol service capabilities configured, this event comes */
    case ESP_A2D_SNK_PSC_CFG_EVT: {
      a2d = (esp_a2d_cb_param_t *)(p_param);
      ESP_LOGI(BT_AV_TAG, "protocol service capabilities configured: 0x%x ",
               a2d->a2d_psc_cfg_stat.psc_mask);
      if (a2d->a2d_psc_cfg_stat.psc_mask & ESP_A2D_PSC_DELAY_RPT) {
        ESP_LOGI(BT_AV_TAG, "Peer device support delay reporting");
      } else {
        ESP_LOGI(BT_AV_TAG, "Peer device unsupport delay reporting");
      }
      break;
    }
    /* when set delay value completed, this event comes */
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT: {
      a2d = (esp_a2d_cb_param_t *)(p_param);
      if (ESP_A2D_SET_INVALID_PARAMS ==
          a2d->a2d_set_delay_value_stat.set_state) {
        ESP_LOGI(BT_AV_TAG, "Set delay report value: fail");
      } else {
        ESP_LOGI(BT_AV_TAG,
                 "Set delay report value: success, delay_value: %u * 1/10 ms",
                 a2d->a2d_set_delay_value_stat.delay_value);
      }
      break;
    }
    /* when get delay value completed, this event comes */
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
      a2d = (esp_a2d_cb_param_t *)(p_param);
      ESP_LOGI(BT_AV_TAG, "Get delay report value: delay_value: %u * 1/10 ms",
               a2d->a2d_get_delay_value_stat.delay_value);
      /* Default delay value plus delay caused by application layer */
      esp_a2d_sink_set_delay_value(a2d->a2d_get_delay_value_stat.delay_value +
                                   APP_DELAY_VALUE);
      break;
    }
    /* others */
    default:
      ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
      break;
  }
}

////////////////////////////////////
//
// Controler Event Handler
//
////////////////////////////////////
static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param) {
  ESP_LOGD(BT_RC_CT_TAG, "%s event: %d", __func__, event);

  esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);

  switch (event) {
    /* when connection state changed, this event comes */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
      uint8_t *bda = rc->conn_stat.remote_bda;
      ESP_LOGI(
          BT_RC_CT_TAG,
          "AVRC conn_state event: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
          rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4],
          bda[5]);

      if (rc->conn_stat.connected) {
        /* get remote supported event_ids of peer AVRCP Target */
        esp_avrc_ct_send_get_rn_capabilities_cmd(APP_RC_CT_TL_GET_CAPS);
        /* start playing as soon as we've connected */
        ct_press_play();
      } else {
        /* clear peer notification capability record */
        s_avrc_peer_rn_cap.bits = 0;
      }
      break;
    }
    /* when passthrough responsed, this event comes */
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
      ESP_LOGI(BT_RC_CT_TAG,
               "AVRC passthrough rsp: key_code 0x%x, key_state %d, rsp_code %d",
               rc->psth_rsp.key_code, rc->psth_rsp.key_state,
               rc->psth_rsp.rsp_code);
      break;
    }
    /* when metadata responsed, this event comes */
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
      ESP_LOGI(BT_RC_CT_TAG, "AVRC metadata rsp: attribute id 0x%x, %s",
               rc->meta_rsp.attr_id, rc->meta_rsp.attr_text);
      free(rc->meta_rsp.attr_text);
      break;
    }
    /* when notified, this event comes */
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
      ESP_LOGI(BT_RC_CT_TAG, "AVRC event notification: %d",
               rc->change_ntf.event_id);
      bt_av_notify_evt_handler(rc->change_ntf.event_id,
                               &rc->change_ntf.event_parameter);
      break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
      ESP_LOGI(BT_RC_CT_TAG, "AVRC remote features %" PRIx32 ", TG features %x",
               rc->rmt_feats.feat_mask, rc->rmt_feats.tg_feat_flag);
      // list target features

      // List feature flags
      uint16_t f = rc->rmt_feats.tg_feat_flag;
      if (f & ESP_AVRC_FEAT_FLAG_CAT1) {
        ESP_LOGI(BT_RC_CT_TAG, "  - Category1");
      }
      if (f & ESP_AVRC_FEAT_FLAG_CAT2) {
        ESP_LOGI(BT_RC_CT_TAG, "  - Category2");
      }
      if (f & ESP_AVRC_FEAT_FLAG_CAT3) {
        ESP_LOGI(BT_RC_CT_TAG, "  - Category3");
      }
      if (f & ESP_AVRC_FEAT_FLAG_CAT4) {
        ESP_LOGI(BT_RC_CT_TAG, "  - Category4");
      }
      if (f & ESP_AVRC_FEAT_FLAG_BROWSING) {
        ESP_LOGI(BT_RC_CT_TAG, "  - Browsing");
      }
      if (f & ESP_AVRC_FEAT_FLAG_COVER_ART_GET_IMAGE_PROP) {
        ESP_LOGI(BT_RC_CT_TAG, "  - Cover Art Get Image Prop");
      }
      if (f & ESP_AVRC_FEAT_FLAG_COVER_ART_GET_IMAGE) {
        ESP_LOGI(BT_RC_CT_TAG, "  - Cover Art Get Image");
      }
      if (f & ESP_AVRC_FEAT_FLAG_COVER_ART_GET_LINKED_THUMBNAIL) {
        ESP_LOGI(BT_RC_CT_TAG, "  - Cover Art Get Linked Thumbnail");
      }

      // List feature mask
      f = rc->rmt_feats.feat_mask;
      if (f & ESP_AVRC_FEAT_RCTG) {
        ESP_LOGI(BT_RC_CT_TAG, "  - target");
      }
      if (f & ESP_AVRC_FEAT_RCCT) {
        ESP_LOGI(BT_RC_CT_TAG, "  - controller");
      }
      if (f & ESP_AVRC_FEAT_VENDOR) {
        ESP_LOGI(BT_RC_CT_TAG, "  - vendor dependent commands");
      }
      if (f & ESP_AVRC_FEAT_BROWSE) {
        ESP_LOGI(BT_RC_CT_TAG, "  - uses browsing channel");
      }
      if (f & ESP_AVRC_FEAT_META_DATA) {
        ESP_LOGI(BT_RC_CT_TAG, "  - metadata transfer command");
      }
      if (f & ESP_AVRC_FEAT_ADV_CTRL) {
        ESP_LOGI(BT_RC_CT_TAG, "  - advanced control command");
      }
      break;
    }
    /* when notification capability of peer device got, this event comes */
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
      ESP_LOGI(BT_RC_CT_TAG, "remote rn_cap: count %d, bitmask 0x%x",
               rc->get_rn_caps_rsp.cap_count, rc->get_rn_caps_rsp.evt_set.bits);
      s_avrc_peer_rn_cap.bits = rc->get_rn_caps_rsp.evt_set.bits;
      bt_av_new_track();
      bt_av_playback_changed();
      bt_av_play_pos_changed();
      break;
    }
    case ESP_AVRC_CT_PLAY_STATUS_RSP_EVT: {
      /* attempting to see if we get this event after pressing play*/
      ESP_LOGI(BT_RC_CT_TAG, "play status change response event");
      break;
    }
    /* others */
    default:
      ESP_LOGE(BT_RC_CT_TAG, "%s unhandled event: %d", __func__, event);
      break;
  }
}

////////////////////////////////////
//
// Target Event Handler
//
////////////////////////////////////
static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param) {
  ESP_LOGD(BT_RC_TG_TAG, "%s event: %d", __func__, event);

  esp_avrc_tg_cb_param_t *rc = (esp_avrc_tg_cb_param_t *)(p_param);

  switch (event) {
    /* when connection state changed, this event comes */
    case ESP_AVRC_TG_CONNECTION_STATE_EVT: {
      uint8_t *bda = rc->conn_stat.remote_bda;
      ESP_LOGI(BT_RC_TG_TAG,
               "AVRC conn_state evt: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
               rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4],
               bda[5]);
      break;
    }
    /* when passthrough commanded, this event comes */
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT: {
      ESP_LOGI(BT_RC_TG_TAG,
               "AVRC passthrough cmd: key_code 0x%x, key_state %d",
               rc->psth_cmd.key_code, rc->psth_cmd.key_state);
      break;
    }
    /* when absolute volume command from remote device set, this event comes */
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT: {
      ESP_LOGI(BT_RC_TG_TAG, "AVRC set absolute volume: %d%%",
               (uint8_t)rc->set_abs_vol.volume * 100 / 0x7f);
      volume_set_by_controller(rc->set_abs_vol.volume);
      break;
    }
    /* when notification registered, this event comes */
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT: {
      ESP_LOGI(BT_RC_TG_TAG,
               ">>>> AVRC register event notification: %d, param: 0x%" PRIx32,
               rc->reg_ntf.event_id, rc->reg_ntf.event_parameter);
      if (rc->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
        s_volume_notify = true;
        esp_avrc_rn_param_t rn_param;
        rn_param.volume = s_volume;
        ESP_LOGI(BT_RC_TG_TAG, "esp_avrc_tg_send_rn_rsp: VOLUME: %d",
                 rn_param.volume);
        esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE,
                                ESP_AVRC_RN_RSP_INTERIM, &rn_param);
      } else if (rc->reg_ntf.event_id == ESP_AVRC_RN_PLAY_STATUS_CHANGE) {
        s_play_notify = true;
        esp_avrc_rn_param_t rn_param;
        rn_param.playback = ESP_AVRC_PLAYBACK_PLAYING;
        esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_PLAY_STATUS_CHANGE,
                                ESP_AVRC_RN_RSP_INTERIM, &rn_param);
      }
      break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT: {
      ESP_LOGI(BT_RC_TG_TAG,
               "AVRC remote features: %" PRIx32 ", CT features: %x",
               rc->rmt_feats.feat_mask, rc->rmt_feats.ct_feat_flag);
      break;
    }
    /* others */
    default:
      ESP_LOGE(BT_RC_TG_TAG, "%s unhandled event: %d", __func__, event);
      break;
  }
}

/********************************
 * EXTERNAL FUNCTION DEFINITIONS
 *******************************/

////////////////////////////////////
//
// A2D Callback
//
////////////////////////////////////

void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
  switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_PROF_STATE_EVT:
    case ESP_A2D_SNK_PSC_CFG_EVT:
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
      bt_app_work_dispatch(bt_av_hdl_a2d_evt, event, param,
                           sizeof(esp_a2d_cb_param_t), NULL);
      break;
    }
    default:
      ESP_LOGE(BT_AV_TAG, "Invalid A2DP event: %d", event);
      break;
  }
}

////////////////////////////////////
//
// A2D Data Callback
//
////////////////////////////////////
void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len) {
  write_ringbuf(data, len);

  /* log the number every 100 packets */
  if (++s_pkt_cnt % 100 == 0) {
    ESP_LOGI(BT_AV_TAG, "Audio packet count: %" PRIu32, s_pkt_cnt);
  }
}

////////////////////////////////////
//
// AVRC Controller Callback
//
////////////////////////////////////
void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event,
                     esp_avrc_ct_cb_param_t *param) {
  switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
      bt_app_alloc_meta_buffer(param);
      /* fall through */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
      bt_app_work_dispatch(bt_av_hdl_avrc_ct_evt, event, param,
                           sizeof(esp_avrc_ct_cb_param_t), NULL);
      break;
    }
    default:
      ESP_LOGE(BT_RC_CT_TAG, "Invalid AVRC event: %d", event);
      break;
  }
}

////////////////////////////////////
//
// AVRC Target Callback
//
////////////////////////////////////
void bt_app_rc_tg_cb(esp_avrc_tg_cb_event_t event,
                     esp_avrc_tg_cb_param_t *param) {
  switch (event) {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
    case ESP_AVRC_TG_SET_PLAYER_APP_VALUE_EVT:
      bt_app_work_dispatch(bt_av_hdl_avrc_tg_evt, event, param,
                           sizeof(esp_avrc_tg_cb_param_t), NULL);
      break;
    default:
      ESP_LOGE(BT_RC_TG_TAG, "Invalid AVRC event: %d", event);
      break;
  }
}

// Callback to automatically connect to previously paired device
void bt_connect_cb(void) {
  // uint8_t bda[6];

  // if (nvs_read_bda(bda)) {
  //   ESP_LOGI(BT_APP_CORE_TAG,
  //            "attempting to connect to paired device "
  //            "[%02x:%02x:%02x:%02x:%02x:%02x]... ",
  //            bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

  //   esp_a2d_sink_connect(bda);
  // }
}

/* compare two bdas for equality */
bool bda_equal(uint8_t *bda1, uint8_t *bda2) {
  uint8_t match = 0;
  for (int i = 0; i < BT_APP_BDA_LEN; i++) {
    if (bda1[i] == bda2[i]) {
      match++;
    }
  }
  return match == BT_APP_BDA_LEN;
}

/* read saved bda */
bool nvs_read_bda(uint8_t *bda) {
  nvs_handle_t my_handle;
  esp_err_t err;
  size_t len;
  bool found = false;

  err = nvs_open(BT_APP_NS, NVS_READONLY, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(BT_AV_TAG, "Cannot open NVS to read %s\n", BT_APP_NS);
  } else {
    err = nvs_get_blob(my_handle, BT_APP_BDA_KEY, (void *)bda, &len);
    if (err == ESP_OK && len == BT_APP_BDA_LEN) {
      found = true;
    }
    nvs_close(my_handle);
  }

  return found;
}

/* update saved bda if necessary */
void nvs_update_bda(uint8_t *bda) {
  nvs_handle_t my_handle;
  esp_err_t err;
  uint8_t saved_bda[BT_APP_BDA_LEN];

  if (bda == NULL) return;

  /* only save if the supplied bda is different from the saved one */
  nvs_read_bda(saved_bda);
  if (!bda_equal(bda, saved_bda)) {
    err = nvs_open(BT_APP_NS, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
      ESP_LOGE(BT_AV_TAG, "Cannot open NVS to write %s\n", BT_APP_NS);
    } else {
      err =
          nvs_set_blob(my_handle, BT_APP_BDA_KEY, (void *)bda, BT_APP_BDA_LEN);
      if (err != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "Error writing BDA: %d\n", err);
      }
      ESP_LOGI(BT_AV_TAG, "New bda written");
      nvs_close(my_handle);
    }
  }
}