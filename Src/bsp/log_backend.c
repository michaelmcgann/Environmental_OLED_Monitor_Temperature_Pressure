
#include "bsp/log_backend.h"
#include "bsp/clock_tree.h"
#include "stm32f4xx.h"

#ifndef LOG_UART_BAUD
#define LOG_UART_BAUD 115200u
#endif

#ifndef LOG_UART_TX_TIMEOUT_LOOPS
#define LOG_UART_TX_TIMEOUT_LOOPS 1000000u
#endif

#define LOG_UART           USART2
#define LOG_UART_AF        7u

#define LOG_UART_GPIO_PORT GPIOA
#define LOG_UART_TX_PIN    2u
#define LOG_UART_RX_PIN    3u

// Clock enables
#define LOG_UART_GPIO_CLK_EN() ( RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN )
#define LOG_UART_CLK_EN()      ( RCC->APB1ENR |= RCC_APB1ENR_USART2EN )

static void log_uart_gpio_init( void );

static void log_uart_periph_init( uint32_t baud );

static em_status_t log_uart_write_byte_blocking( uint8_t b );

static em_status_t log_uart_write_blocking( const uint8_t *data, size_t len );

static void log_uart_gpio_init( void ) {

	// enable GPIO clock
	LOG_UART_GPIO_CLK_EN();

	// Set PA2 (TX) and PA3 (RX) to Alternate Function mode
	LOG_UART_GPIO_PORT->MODER &= ~( 3u << ( 2u * LOG_UART_TX_PIN ) );
	LOG_UART_GPIO_PORT->MODER |=  ( 2u << ( 2u * LOG_UART_TX_PIN ) );

	LOG_UART_GPIO_PORT->MODER &= ~( 3u << ( 2u * LOG_UART_RX_PIN ) );
	LOG_UART_GPIO_PORT->MODER |=  ( 2u << ( 2u * LOG_UART_RX_PIN ) );

	// Output type -> push-pull (OTYPER = 0)
	LOG_UART_GPIO_PORT->OTYPER &= ~( 1u << LOG_UART_TX_PIN );
	LOG_UART_GPIO_PORT->OTYPER &= ~( 1u << LOG_UART_RX_PIN );

	// Speed -> high -> OSPEED = 10
	LOG_UART_GPIO_PORT->OSPEEDR &= ~( 3u << ( 2u * LOG_UART_TX_PIN ) );
	LOG_UART_GPIO_PORT->OSPEEDR |=  ( 2u << ( 2u * LOG_UART_TX_PIN ) );

	LOG_UART_GPIO_PORT->OSPEEDR &= ~( 3u << ( 2u * LOG_UART_RX_PIN ) );
	LOG_UART_GPIO_PORT->OSPEEDR |=  ( 2u << ( 2u * LOG_UART_RX_PIN ) );

	/*
	 * pull-ups: TX -> no pull up (00), RX -> pull-up (01)
	 */
	LOG_UART_GPIO_PORT->PUPDR &= ~( 3u << ( 2u * LOG_UART_TX_PIN ) );
	LOG_UART_GPIO_PORT->PUPDR &= ~( 3u << ( 2u * LOG_UART_RX_PIN ) );
	LOG_UART_GPIO_PORT->PUPDR |=  ( 1u << ( 2u * LOG_UART_RX_PIN ) );

	// Select AF7 for USART2 on PA2/PA3
	LOG_UART_GPIO_PORT->AFR[0] &= ~( 0xFu        << ( 4u * LOG_UART_TX_PIN ) );
	LOG_UART_GPIO_PORT->AFR[0] |=  ( LOG_UART_AF << ( 4u * LOG_UART_TX_PIN ) );

	LOG_UART_GPIO_PORT->AFR[0] &= ~( 0xFu        << ( 4u * LOG_UART_RX_PIN ) );
	LOG_UART_GPIO_PORT->AFR[0] |=  ( LOG_UART_AF << ( 4u * LOG_UART_RX_PIN ) );

}

static uint32_t log_get_pclk_hz( void ) {
	return ( clock_pclk1_hz() );
}

static void log_uart_periph_init( uint32_t baud ) {

	// Enable USART clock on APB1
	LOG_UART_CLK_EN();

	// Disable UART before config
	LOG_UART->CR1 &= ~( USART_CR1_UE );

	// Configure basic frame: 8 data bits, no parity, 1 stop bit 8N1
	LOG_UART->CR1 &= ~( USART_CR1_M | USART_CR1_PCE );
	LOG_UART->CR2 &= ~( USART_CR2_STOP );
	LOG_UART->CR3 &= ~( USART_CR3_CTSE | USART_CR3_RTSE );

	// Over sampling by 16 - default
	LOG_UART->CR1 &= ~( USART_CR1_OVER8 );

	uint32_t pclk = log_get_pclk_hz();
	uint32_t div = ( pclk + ( baud / 2u ) ) / baud;
	uint32_t mant = div / 16u;
	uint32_t frac = div % 16u;
	LOG_UART->BRR = ( mant << 4 ) | ( frac & 0xFu );

	// Enable transmitter
	LOG_UART->CR1 |= USART_CR1_TE | USART_CR1_RE;

	// Enable UART
	LOG_UART->CR1 |= USART_CR1_UE;

}

static em_status_t log_uart_write_byte_blocking( uint8_t b ) {

	uint32_t timeout = LOG_UART_TX_TIMEOUT_LOOPS;

	// Wait for TXE (data reg empty)
	while ( ( LOG_UART->SR & USART_SR_TXE ) == 0u ) {

		if ( --timeout == 0u ) return EM_E_TIMEOUT;

	}

	// Write byte
	LOG_UART->DR = (uint16_t) b;
	return EM_OK;

}

static em_status_t log_uart_write_blocking( const uint8_t *data, size_t len ) {

	if ( (len > 0u) && ( data == NULL ) ) return EM_E_NULL;

	for ( size_t i = 0; i < len; i++ ) {
		em_status_t status = log_uart_write_byte_blocking( data[i] );
		if ( status != EM_OK ) return status;
	}

	uint32_t timeout = LOG_UART_TX_TIMEOUT_LOOPS;
	while ( ( LOG_UART->SR & USART_SR_TC ) == 0u ) {
		if ( --timeout == 0u ) return EM_E_TIMEOUT;
	}

	return EM_OK;
}

em_status_t log_backend_init( void ) {
	log_uart_gpio_init();
	log_uart_periph_init(LOG_UART_BAUD);
	return EM_OK;
}

em_status_t log_backend_write( const uint8_t *data, size_t len) {

	if ( (len >0u) && ( data == NULL ) ) return EM_E_NULL;

	if ( len == 0u ) return EM_OK;

	return log_uart_write_blocking( data, len );
}



