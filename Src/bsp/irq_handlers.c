#include "bsp/timebase.h"

void SysTick_Handler( void ) {

	timebase_isr_1ms();

}

