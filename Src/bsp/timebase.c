

#include "bsp/timebase.h"


static volatile uint32_t g_ms;

em_status_t timebase_init( void ) {

	g_ms = 0;

	uint8_t failed = SysTick_Config( SystemCoreClock / 1000u );
	if ( failed ) return EM_E_PARAM;
	else return EM_OK;

}

void timebase_isr_1ms( void ) {
	g_ms++;
}

uint32_t time_ms( void ) {
	return g_ms;
}

bool time_elapsed( uint32_t *last, uint32_t period_ms ) {

	if ( ( last == NULL ) || ( period_ms == 0u ) ) return false;

	uint32_t now = time_ms();

	uint32_t elapsed = ( uint32_t ) ( now - *last );

	if ( elapsed >= period_ms ) {
		*last += period_ms;
		return true;
	}

	return false;

}

