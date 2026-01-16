
#include "bsp/clock_tree.h"
#include "stm32f4xx.h"


// APB prescaler decode table
static const uint8_t s_apb_presc_table[8] = {
		1, 1, 1, 1, 2, 4, 8, 16
};

// Returns APB1 prescaler divisor
static uint32_t apb1_divisor( void ) {
	uint32_t ppre1 = ( ( ( RCC->CFGR & RCC_CFGR_PPRE1_Msk ) >> RCC_CFGR_PPRE1_Pos ) );
	return ( ( uint32_t ) s_apb_presc_table[ppre1] );
}

static uint32_t apb2_divisor( void ) {

	uint32_t ppre2 = ( ( RCC->CFGR & RCC_CFGR_PPRE2_Msk ) >> RCC_CFGR_PPRE2_Pos );
	return ( ( uint32_t ) s_apb_presc_table[ppre2] );
}

// Get HCLK - AHB clock, in Hz
uint32_t clock_hclk_hz( void ) {
	return ( SystemCoreClock );
}

// Get PCLK1 - APB1, in Hz
uint32_t clock_pclk1_hz( void ) {
	uint32_t hclk = clock_hclk_hz();
	return ( hclk / apb1_divisor() );
}

// Get PCLK2 - APB2, in Hz
uint32_t clock_pclk2_hz( void ) {
	uint32_t hclk = clock_hclk_hz();
	return ( hclk / apb2_divisor() );
}

// APB1 timer clock, in Hz
uint32_t clock_apb1_timer_hz( void ) {
	uint32_t divisor = apb1_divisor();
	uint32_t pclk1 = clock_pclk1_hz();
	return ( ( divisor == 1u ) ? pclk1 : ( 2u * pclk1 ) );
}

// APB2 timer clock, in Hz
uint32_t clock_apb2_timer_hz( void ) {
	uint32_t divisor = apb2_divisor();
	uint32_t pclk2 = clock_pclk2_hz();
	return ( ( divisor == 1u ) ? pclk2 : ( 2u * pclk2 ) );
}



