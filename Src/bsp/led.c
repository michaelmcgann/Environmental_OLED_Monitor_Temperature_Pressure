

#include "bsp/led.h"
#include "stm32f4xx.h"
#include <stdint.h>
#include "utils/em_status.h"

#define LED_GPIO_CLK_EN()   ( RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN )
#define LED_GPIO_READBACK() do { (void) RCC->AHB1ENR; } while (0)

#define LED_HEARTBEAT_PIN 12
#define LED_WARNING_PIN   14

#define LED_GPIO_PORT GPIOD

static void led_config_output( GPIO_TypeDef *port, uint32_t pin );

typedef struct {
	GPIO_TypeDef *port;
	uint32_t pin;
} led_hw_t;

static const led_hw_t s_led_map[LED_COUNT] = {
		[LED_HEARTBEAT] = { .port = LED_GPIO_PORT, .pin = LED_HEARTBEAT_PIN },
		[LED_WARNING]   = { .port = LED_GPIO_PORT, .pin = LED_WARNING_PIN }
};


void led_init( void ) {

	LED_GPIO_CLK_EN();
	LED_GPIO_READBACK();

	for ( led_id_t id = 0; id < LED_COUNT; id++ ) {

		GPIO_TypeDef *port = s_led_map[id].port;
		uint32_t pin = s_led_map[id].pin;
		led_config_output( port, pin );

		// set default to off
		led_set( id, false );
	}
}

static void led_config_output( GPIO_TypeDef *port, uint32_t pin ) {

	port->MODER &= ~( 3u << ( 2u * pin ) );
	port->MODER |=  ( 1u << ( 2u * pin ) );

	port->OTYPER &= ~( 1u << pin );

	port->OSPEEDR &= ~( 3u << ( 2u * pin ) );

	port->PUPDR &= ~( 3u << ( 2u * pin ) );

}

em_status_t led_set( led_id_t id, bool on ) {
	
	if ( id >= LED_COUNT ) return EM_E_PARAM;

	uint32_t pin = s_led_map[id].pin;
	GPIO_TypeDef *port = s_led_map[id].port;
	uint32_t mask = 1u << pin;
	
	if ( on ) port->BSRR = mask;
	else      port->BSRR = (mask << 16u);

	return EM_OK;
}

em_status_t led_toggle( led_id_t id ) {
	
	if ( id >= LED_COUNT ) return EM_E_PARAM;

	uint32_t pin = s_led_map[id].pin;
	GPIO_TypeDef *port = s_led_map[id].port;

	uint32_t mask = 1u << pin;
	
	if ( ( port->ODR & mask ) != 0 ) port->BSRR = mask << 16u;
	else port->BSRR = mask;
	
	return EM_OK;

}


