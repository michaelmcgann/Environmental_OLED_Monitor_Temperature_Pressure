

#include <stdint.h>
#include "utils/log.h"
#include "bsp/led.h"
#include "bsp/timebase.h"
#include "app/heartbeat.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void) {

	SystemCoreClockUpdate();

	log_init();
	led_init();
	timebase_init();
	heartbeat_init( LED_HEARTBEAT, 500 );

	const uint8_t boot[] = "BOOT\r\n";
	const size_t size = sizeof(boot) - 1u;
	log_write( boot, size );

	uint32_t last = 0;

	while (1) {

		heartbeat_tick();

		if ( time_elapsed( &last, 500u ) ) {
			const uint8_t msg[] = "I own Leeds\r\n";
			log_write( msg, sizeof( msg ) - 1 );
		}


	}


}
