

#ifndef APP_HEARTBEAT_H_
#define APP_HEARTBEAT_H_

#include "bsp/led.h"
#include <stdint.h>

em_status_t heartbeat_init( led_id_t led_id, uint32_t period_ms );
void heartbeat_tick( void );


#endif /* APP_HEARTBEAT_H_ */
