

#include "app/heartbeat.h"
#include "utils/em_status.h"
#include "bsp/timebase.h"

static led_id_t s_led;
static uint32_t s_period_ms;
static uint32_t s_last_ms;

em_status_t heartbeat_init( led_id_t led_id, uint32_t period_ms ) {

	s_led = led_id;
	s_period_ms = period_ms;
	s_last_ms = time_ms();
	return led_set( led_id , 0 );

}


void heartbeat_tick( void ) {

	if ( time_elapsed( &s_last_ms, s_period_ms ) ) led_toggle( s_led );

}
