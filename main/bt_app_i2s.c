#include "bt_app_i2s.h"

#include <driver/i2s_std.h>
#include <esp_log.h>
#include <freertos/ringbuf.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define RINGBUF_HIGHEST_WATER_LEVEL (32 * 1024)
#define RINGBUF_PREFETCH_WATER_LEVEL (20 * 1024)

/*******************************
 * STATIC VARIABLE DEFINITIONS
 ******************************/
static RingbufHandle_t s_ringbuf_i2s = NULL; /* handle of ringbuffer for I2S */
static TaskHandle_t s_bt_i2s_task_handle = NULL; /* handle of I2S task */
static SemaphoreHandle_t s_i2s_write_semaphore = NULL;
static uint16_t ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
i2s_chan_handle_t tx_chan = NULL;

/*******************************
 * STATIC FUNCTION DECLARATIONS
 ******************************/
static void bt_i2s_task_handler(void *arg);

/*******************************
 * FUNCTION DEFINITIONS
 ******************************/

/**
 * I2S task handler
 */
static void bt_i2s_task_handler(void *arg) {
  uint8_t *data = NULL;
  size_t item_size = 0;
  /**
   * The total length of DMA buffer of I2S is:
   * `dma_frame_num * dma_desc_num * i2s_channel_num * i2s_data_bit_width / 8`.
   * Transmit `dma_frame_num * dma_desc_num` bytes to DMA is trade-off.
   */
  const size_t item_size_upto = 240 * 6;
  size_t bytes_written = 0;

  for (;;) {
    if (pdTRUE == xSemaphoreTake(s_i2s_write_semaphore, portMAX_DELAY)) {
      for (;;) {
        item_size = 0;
        /* receive data from ringbuffer and write it to I2S DMA transmit
         * buffer
         */
        data = (uint8_t *)xRingbufferReceiveUpTo(s_ringbuf_i2s, &item_size,
                                                 (TickType_t)pdMS_TO_TICKS(20),
                                                 item_size_upto);
        if (item_size == 0) {
          ESP_LOGI(I2S_TAG,
                   "ringbuffer underflowed! mode changed: "
                   "RINGBUFFER_MODE_PREFETCHING");
          ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
          break;
        }

#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
        dac_continuous_write(tx_chan, data, item_size, &bytes_written, -1);
#else
        i2s_channel_write(tx_chan, data, item_size, &bytes_written,
                          portMAX_DELAY);
#endif
        vRingbufferReturnItem(s_ringbuf_i2s, (void *)data);
      }
    }
  }
}

/********************************
 * EXTERNAL FUNCTION DEFINITIONS
 *******************************/

/**
 * i2s config
 */
void bt_i2s_config(int sample_rate, int ch_count) {
  i2s_channel_disable(tx_chan);
  i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
  i2s_std_slot_config_t slot_cfg =
      I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, ch_count);
  slot_cfg.bit_shift = true;  // required for PCM5102 I2S format
  i2s_channel_reconfig_std_clock(tx_chan, &clk_cfg);
  i2s_channel_reconfig_std_slot(tx_chan, &slot_cfg);
  i2s_channel_enable(tx_chan);
}

/**
 * enable I2S driver
 */
void bt_i2s_driver_install(void) {
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
}

/**
 * disable I2S driver
 */
void bt_i2s_driver_uninstall(void) {
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
  ESP_ERROR_CHECK(dac_continuous_disable(tx_chan));
  ESP_ERROR_CHECK(dac_continuous_del_channels(tx_chan));
#else
  ESP_ERROR_CHECK(i2s_channel_disable(tx_chan));
  ESP_ERROR_CHECK(i2s_del_channel(tx_chan));
#endif
}

/**
 * I2S task start up
 */
void bt_i2s_task_start_up(void) {
  ESP_LOGI(I2S_TAG,
           "ringbuffer data empty! mode changed: RINGBUFFER_MODE_PREFETCHING");
  ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
  if ((s_i2s_write_semaphore = xSemaphoreCreateBinary()) == NULL) {
    ESP_LOGE(I2S_TAG, "%s, Semaphore create failed", __func__);
    return;
  }
  if ((s_ringbuf_i2s = xRingbufferCreate(RINGBUF_HIGHEST_WATER_LEVEL,
                                         RINGBUF_TYPE_BYTEBUF)) == NULL) {
    ESP_LOGE(I2S_TAG, "%s, ringbuffer create failed", __func__);
    return;
  }
  xTaskCreate(bt_i2s_task_handler, "BtI2STask", 2048, NULL,
              configMAX_PRIORITIES - 3, &s_bt_i2s_task_handle);
}

/**
 * I2S task shut down
 */
void bt_i2s_task_shut_down(void) {
  if (s_bt_i2s_task_handle) {
    vTaskDelete(s_bt_i2s_task_handle);
    s_bt_i2s_task_handle = NULL;
  }
  if (s_ringbuf_i2s) {
    vRingbufferDelete(s_ringbuf_i2s);
    s_ringbuf_i2s = NULL;
  }
  if (s_i2s_write_semaphore) {
    vSemaphoreDelete(s_i2s_write_semaphore);
    s_i2s_write_semaphore = NULL;
  }
}

/**
 * write ringbuf
 */

size_t write_ringbuf(const uint8_t *data, size_t size) {
  size_t item_size = 0;
  BaseType_t done = pdFALSE;

  if (ringbuffer_mode == RINGBUFFER_MODE_DROPPING) {
    ESP_LOGW(I2S_TAG, "ringbuffer is full, drop this packet!");
    vRingbufferGetInfo(s_ringbuf_i2s, NULL, NULL, NULL, NULL, &item_size);
    if (item_size <= RINGBUF_PREFETCH_WATER_LEVEL) {
      ESP_LOGI(I2S_TAG,
               "ringbuffer data decreased! mode changed: "
               "RINGBUFFER_MODE_PROCESSING");
      ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
    }
    return 0;
  }

  done = xRingbufferSend(s_ringbuf_i2s, (void *)data, size, (TickType_t)0);

  if (!done) {
    ESP_LOGW(I2S_TAG,
             "ringbuffer overflowed, ready to decrease data! mode changed: "
             "RINGBUFFER_MODE_DROPPING");
    ringbuffer_mode = RINGBUFFER_MODE_DROPPING;
  }

  if (ringbuffer_mode == RINGBUFFER_MODE_PREFETCHING) {
    vRingbufferGetInfo(s_ringbuf_i2s, NULL, NULL, NULL, NULL, &item_size);
    if (item_size >= RINGBUF_PREFETCH_WATER_LEVEL) {
      ESP_LOGI(I2S_TAG,
               "ringbuffer data increased! mode changed: "
               "RINGBUFFER_MODE_PROCESSING");
      ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
      if (pdFALSE == xSemaphoreGive(s_i2s_write_semaphore)) {
        ESP_LOGE(I2S_TAG, "semphore give failed");
      }
    }
  }

  return done ? size : 0;
}