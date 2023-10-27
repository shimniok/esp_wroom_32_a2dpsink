#ifndef BT_APP_STACK_H_
#define BT_APP_STACK_H_

#include <stdint.h>

#define BT_STACK_TAG "BT_STK"

/* event for stack up */
#define BT_APP_EVT_STACK_UP 0

/* initialize bt stack */
void bt_stack_init(void);

/* handler for bluetooth stack enabled events */
void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

#endif  // BT_APP_STACK_H_