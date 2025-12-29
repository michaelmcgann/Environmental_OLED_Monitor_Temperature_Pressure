


#ifndef BSP_TIMEBASE_H_
#define BSP_TIMEBASE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32f4xx.h"
#include "utils/em_status.h"

em_status_t timebase_init( void );
uint32_t time_ms( void );
void timebase_isr_1ms( void );

/*
 * Checks if period amount of time has past since last successful check
 */
bool time_elapsed( uint32_t *last, uint32_t period_ms );



#endif /* BSP_TIMEBASE_H_ */
