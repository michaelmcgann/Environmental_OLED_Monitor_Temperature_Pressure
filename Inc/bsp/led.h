

#ifndef BSP_LED_H_
#define BSP_LED_H_

#include <stdbool.h>
#include "utils/em_status.h"

typedef enum {
	LED_HEARTBEAT,
	LED_WARNING,
	LED_COUNT
} led_id_t;

void led_init  ( void );
em_status_t led_set   ( led_id_t id, bool on );
em_status_t led_toggle( led_id_t id );


#endif /* BSP_LED_H_ */
