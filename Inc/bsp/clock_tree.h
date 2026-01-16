

/**
 * Notes:
 * HCLK  = AHB  - SystemCoreClock
 * PCLK1 = APB1 - peripheral clock
 * PCLK2 = APB2 - peripheral clock
 */

#ifndef BSP_CLOCK_TREE_H_
#define BSP_CLOCK_TREE_H_

#include <stdint.h>

// Get HCLK - AHB clock, in Hz
uint32_t clock_hclk_hz( void );

// Get PCLK1 - APB1, in Hz
uint32_t clock_pclk1_hz( void );

// Get PCLK2 - APB2, in Hz
uint32_t clock_pclk2_hz( void );

// APB1 timer clock, in Hz
uint32_t clock_apb1_timer_hz( void );

// APB2 timer clock, in Hz
uint32_t clock_apb2_timer_hz( void );


#endif /* BSP_CLOCK_TREE_H_ */
