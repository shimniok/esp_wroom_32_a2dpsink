#ifndef __I2S_H__
#define __I2S_H__

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* log tag */
#define I2S_TAG "I2S"

enum {
  RINGBUFFER_MODE_PROCESSING,  /* ringbuffer is buffering incoming audio data,
                                  I2S is working */
  RINGBUFFER_MODE_PREFETCHING, /* ringbuffer is buffering incoming audio data,
                                  I2S is waiting */
  RINGBUFFER_MODE_DROPPING /* ringbuffer is not buffering (dropping) incoming
                              audio data, I2S is working */
};

/**
 * @brief  start up the is task
 */
void bt_i2s_task_start_up(void);

/**
 * @brief  shut down the I2S task
 */
void bt_i2s_task_shut_down(void);

/**
 * @brief  write data to ringbuffer
 *
 * @param [in] data  pointer to data stream
 * @param [in] size  data length in byte
 *
 * @return size if writteen ringbuffer successfully, 0 others
 */
size_t write_ringbuf(const uint8_t *data, size_t size);

#endif