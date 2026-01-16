
#include "bsp/i2c_gpio_init.h"
#include "stm32f4xx.h"

#define SDA_PIN 7u
#define SCL_PIN 6u

#define TWO_BIT_CLEAR  ( ( 3u << ( SDA_PIN * 2u ) ) | ( 3u << ( SCL_PIN * 2u ) ) )
#define FOUR_BIT_CLEAR ( ( 0xFu << ( SDA_PIN * 4u ) ) | ( 0xFu << ( SCL_PIN * 4u ) ) )

#define MODER_SET    ( ( 2u << ( SDA_PIN * 2u ) ) | ( 2u << ( SCL_PIN * 2u ) ) )
#define OTYPER_SET   ( ( 1u << SDA_PIN ) | ( 1u << SCL_PIN ) )
#define OSPEEDR_SET  ( ( 2u << ( SDA_PIN * 2u ) ) | ( 2u << ( SCL_PIN * 2u ) ) )
#define AFR_SET      ( ( 4u << ( SDA_PIN * 4u ) ) | ( 4u << ( SCL_PIN * 4u ) ) )

void i2c1_gpio_init( void ) {

	// Enable GPIOB clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	( void ) RCC->AHB1ENR;

	// PB6 & PB7 to alternate function mode -> 10b
	GPIOB->MODER &= ~( TWO_BIT_CLEAR );
	GPIOB->MODER |=  ( MODER_SET);

	// Set pins to open drain needed for I2C
	GPIOB->OTYPER |= ( OTYPER_SET );

	// Output speed, set high, better suited for breadboard
	GPIOB->OSPEEDR &= ~( TWO_BIT_CLEAR );
	GPIOB->OSPEEDR |= ( OSPEEDR_SET );

	// No pull-up pull-down, already on modules -> set to 00
	GPIOB->PUPDR &= ~( TWO_BIT_CLEAR );

	// Select AF4 for I2C1 on PB6 and PB7
	GPIOB->AFR[0] &= ~( FOUR_BIT_CLEAR );
	GPIOB->AFR[0] |=   ( AFR_SET );

}

