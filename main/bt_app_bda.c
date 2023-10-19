#include "bt_app_bda.h"

#include "esp_log.h"
#include "nvs.h"

/*******************************
 * STATIC FUNCTION DECLARATIONS
 ******************************/
static bool bda_equal(uint8_t *bda1, uint8_t *bda2);

/* compare two bdas for equality */
static bool bda_equal(uint8_t *bda1, uint8_t *bda2) {
  uint8_t match = 0;
  for (int i = 0; i < BT_APP_BDA_LEN; i++) {
    if (bda1[i] == bda2[i]) {
      match++;
    }
  }
  return match == BT_APP_BDA_LEN;
}

/**
 * read saved bda
 */
bool nvs_read_bda(uint8_t *bda) {
  nvs_handle_t my_handle;
  esp_err_t err;
  size_t len;
  bool found = false;

  err = nvs_open(BT_APP_NS, NVS_READONLY, &my_handle);
  if (err != ESP_OK) {
    ESP_LOGE(BT_BDA_TAG, "Cannot open NVS to read %s\n", BT_APP_NS);
  } else {
    err = nvs_get_blob(my_handle, BT_APP_BDA_KEY, (void *)bda, &len);
    if (err == ESP_OK && len == BT_APP_BDA_LEN) {
      found = true;
    }
    nvs_close(my_handle);
  }

  return found;
}

/**
 *  update saved bda if necessary
 */
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
      ESP_LOGE(BT_BDA_TAG, "Cannot open NVS to write %s\n", BT_APP_NS);
    } else {
      err =
          nvs_set_blob(my_handle, BT_APP_BDA_KEY, (void *)bda, BT_APP_BDA_LEN);
      if (err != ESP_OK) {
        ESP_LOGE(BT_BDA_TAG, "Error writing BDA: %d\n", err);
      }
      ESP_LOGI(BT_BDA_TAG, "New bda written");
      nvs_close(my_handle);
    }
  }
}