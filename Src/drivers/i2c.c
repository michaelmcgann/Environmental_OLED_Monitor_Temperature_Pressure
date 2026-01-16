#include "drivers/i2c.h"
#include "bsp/timebase.h"
#include "bsp/i2c_gpio_init.h"
#include "bsp/clock_tree.h"
#include "tests/i2c_tests.h"


///////////////////////////////////////////////////////
///// INTERNAL HELPERS AND SET-UP
///////////////////////////////////////////////////////

#define I2C_DIR_WRITE 0u
#define I2C_DIR_READ  1u

static const uint8_t MINIMUM_ADDR = 0x08u;
static const uint8_t MAXIMUM_ADDR = 0x77u;

static i2c_status_t i2c_map_error_and_clear( I2C_TypeDef *i2c ) {

	uint32_t sr1 = i2c->SR1;

	if ( sr1 & I2C_SR1_ARLO ) { i2c->SR1 &= ~I2C_SR1_ARLO; return I2C_ERR_ARB_LOST; }
	if ( sr1 & I2C_SR1_BERR ) { i2c->SR1 &= ~I2C_SR1_BERR; return I2C_ERR_BUS_ERROR; }
	if ( sr1 & I2C_SR1_OVR )  { i2c->SR1 &= ~I2C_SR1_OVR;  return I2C_ERR_OVERRUN; }

	// caller to check
	if ( sr1 & I2C_SR1_AF ) {}

	return I2C_OK;

}

static i2c_status_t i2c_wait_flag_set( i2c_handle_t *h, volatile uint32_t *reg, uint32_t mask ) {

	uint32_t start = time_ms();
	while ( ( ( *reg ) & mask ) == 0u ) {

		i2c_status_t e = i2c_map_error_and_clear( h->instance );
		if ( e != I2C_OK ) { h->last_error = e; return ( e ); }

		if ( ( time_ms() - start ) >= h->timeout_ms ) {
			h->last_error = I2C_ERR_TIMEOUT;
			return ( I2C_ERR_TIMEOUT );
		}
	}
	return I2C_OK;
}

static i2c_status_t i2c_wait_flag_clear( i2c_handle_t *h, volatile uint32_t *reg, uint32_t mask ) {

	uint32_t start = time_ms();
	while ( ( ( *reg ) & mask ) != 0u ) {

		i2c_status_t e = i2c_map_error_and_clear( h->instance );
		if ( e != I2C_OK ) { h->last_error = e; return ( e ); }

		if ( ( time_ms() - start ) >= h->timeout_ms ) {
			h->last_error = I2C_ERR_TIMEOUT;
			return ( I2C_ERR_TIMEOUT );
		}
	}
	return I2C_OK;
}

// Clear ADDR
static inline void i2c_clear_addr( I2C_TypeDef *i2c ) {
	( void ) i2c->SR1;
	( void ) i2c->SR2;
}

static void i2c_clear_stop_if_stuck( I2C_TypeDef * i2c ) {

	if ( i2c->CR1 & I2C_CR1_STOP ) {
		i2c->CR1 &= ~( I2C_CR1_STOP );
	}
}

static i2c_status_t i2c_stop_and_wait_busy_clear( i2c_handle_t *h ) {
	I2C_TypeDef *i2c = h->instance;

	// Request STOP
	i2c->CR1 |= I2C_CR1_STOP;

	// Wait for bus not busy)
	i2c_status_t st = i2c_wait_flag_clear( h, &i2c->SR2, I2C_SR2_BUSY );

	i2c_clear_stop_if_stuck( i2c );

	return st;
}

static void i2c1_peripheral_reset( void ) {
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
	(void) RCC->APB1RSTR;
	RCC->APB1RSTR &= ~( RCC_APB1RSTR_I2C1RST );
}

static void i2c_gpio_init( void ) {
	i2c1_gpio_init();
}

static inline i2c_status_t i2c_fail( i2c_handle_t *h, i2c_status_t e ) {
	if ( h ) h->last_error = e;
	return e;
}


///////////////////////////////////////////////////////
///// PUBLIC API
///////////////////////////////////////////////////////

i2c_status_t i2c_init_default( i2c_handle_t *h, I2C_TypeDef *instance ) {
	uint32_t     pclk1   = clock_pclk1_hz();
	uint32_t     bus_hz  = 100000u;
	uint32_t     timeout = 200u;
	return ( i2c_init( h, instance, pclk1, bus_hz, timeout ) );
}

i2c_status_t i2c_init( i2c_handle_t *h, I2C_TypeDef *instance, uint32_t pclk1_hz, uint32_t bus_hz, uint32_t timeout_ms ) {

	if ( !( h ) || pclk1_hz == 0u || bus_hz == 0u || timeout_ms == 0u ) return i2c_fail( h, I2C_ERR_BAD_PARAM );

	if ( instance != I2C1 ) return i2c_fail( h, I2C_ERR_UNSUPPORTED );

	h->instance   = instance;
	h->pclk1_hz   = pclk1_hz;
	h->bus_hz     = bus_hz;
	h->timeout_ms = timeout_ms;
	h->last_error = I2C_OK;

	// GPIO set up, SCL and SDA pins
	i2c_gpio_init();

	// Enable I2C1 clock
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	( void ) RCC->APB1ENR;

	RCC->APB1RSTR |=    RCC_APB1RSTR_I2C1RST;
	RCC->APB1RSTR &= ~( RCC_APB1RSTR_I2C1RST );

	// Disabled peripheral before configuring
	instance->CR1 &= ~( I2C_CR1_PE );

	// CR2.FREQ expects APB1 frequency in MHz
	uint32_t pclk1_mhz = pclk1_hz / 1000000u;
	if ( ( pclk1_mhz < 2u ) || ( pclk1_mhz > 50u) ) return i2c_fail( h, I2C_ERR_BAD_PARAM );

	instance->CR2 = ( instance->CR2 & ~I2C_CR2_FREQ_Msk ) | ( ( pclk1_mhz << I2C_CR2_FREQ_Pos ) & I2C_CR2_FREQ_Msk );

	// Set up standard mode and put duty to known state
	instance->CCR &= ~( ( I2C_CCR_FS ) | ( I2C_CCR_DUTY ) );

	// calculate CCR
	uint32_t ccr = pclk1_hz / ( 2u * bus_hz );
	if ( ccr < 4u ) ccr = 4u;
	instance->CCR = ( ( instance->CCR & ~I2C_CCR_CCR_Msk ) | ( ( ccr << I2C_CCR_CCR_Pos ) & I2C_CCR_CCR_Msk ));

	// TRISE for standard mode -> Pclk1 in MHz + 1
	instance->TRISE = ( ( ( pclk1_mhz + 1u ) << I2C_TRISE_TRISE_Pos ) & I2C_TRISE_TRISE_Msk );

	// Enable peripheral
	instance->CR1 |= I2C_CR1_PE;

	// verify bus not stuck
	i2c_status_t st = i2c_wait_flag_clear( h, &instance->SR2, I2C_SR2_BUSY );

	return ( st );
}


i2c_status_t i2c_probe( i2c_handle_t *h, uint8_t addr7 ) {

	if ( !( h ) || !( h->instance ) || ( addr7 < MINIMUM_ADDR ) || ( addr7 > MAXIMUM_ADDR ) )
		return i2c_fail( h, I2C_ERR_BAD_PARAM );

	I2C_TypeDef *i2c = h->instance;

	// If STOP stuck, clear
	i2c_clear_stop_if_stuck( i2c );

	// Wait for bus to be idle, if stuck, reset peripheral once and retry
	i2c_status_t st = i2c_wait_flag_clear( h, &i2c->SR2, I2C_SR2_BUSY );
	if ( st != I2C_OK ) {
		i2c1_peripheral_reset();
		(void) i2c_init( h, h->instance, h->pclk1_hz, h->bus_hz, h->timeout_ms );

		st = i2c_wait_flag_clear( h, &i2c->SR2, I2C_SR2_BUSY );
		if ( st != I2C_OK ) return st;
	}

//	i2c_status_t st = i2c_wait_flag_clear( h, &i2c->SR2, I2C_SR2_BUSY );
//	if ( st != I2C_OK ) return st;

	// Generate START
	i2c->CR1 |= I2C_CR1_START;
	st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_SB );
	if ( st != I2C_OK ) return st;

	// Send address on bus in write mode
	i2c->DR = ( uint8_t ) ( ( addr7 << 1 ) | I2C_DIR_WRITE ) ;

	// Wait for ACK or NACK (ADDR or AF)
	uint32_t start = time_ms();
	while ( ( ( i2c->SR1 & I2C_SR1_ADDR ) == 0u ) ) {

		// Check for NACK
		if ( i2c->SR1 & I2C_SR1_AF ) {

			(void) i2c_stop_and_wait_busy_clear( h );

			// Clear AF flag
			i2c->SR1 &= ~( I2C_SR1_AF );

			h->last_error = I2C_ERR_NACK_ADDR;
			return ( I2C_ERR_NACK_ADDR );
		} // END IF NACK

		// Check for errors
		i2c_status_t e = i2c_map_error_and_clear( i2c );
		if ( e != I2C_OK ) { h->last_error = e; return e; }

		// Check for timeout
		if ( ( time_ms() - start ) >= h->timeout_ms ) {

			(void) i2c_stop_and_wait_busy_clear( h );

			h->last_error = I2C_ERR_TIMEOUT;
			return I2C_ERR_TIMEOUT;
		}

	} // END WHILE ADDR

	// ADDR ACK'd, clear ADDR and generate STOP
	i2c_clear_addr( i2c );
	i2c_status_t end = i2c_stop_and_wait_busy_clear( h );

	if ( end != I2C_OK ) return i2c_fail( h, end );

	h->last_error = I2C_OK;
	return I2C_OK;

} // END i2c_probe


i2c_status_t i2c_scan( i2c_handle_t *h, uint8_t *out_addrs, size_t max_addrs, size_t *out_count ) {

	// Check for relevant null pointers
	if ( !( h ) || !( out_count ) ) { h->last_error = I2C_ERR_BAD_PARAM; return I2C_ERR_BAD_PARAM; }

	size_t count = 0;

	// Loop over possible addresses
	for ( uint8_t addr = MINIMUM_ADDR; addr <= MAXIMUM_ADDR; addr++ ) {

		if ( i2c_probe( h, addr ) == I2C_OK ) {

			if ( out_addrs && ( count < max_addrs ) ) {
				out_addrs[count] = addr;
			} // END IF OUT_ADDRESSES ARRAY EXISTS
			count++;
		} // END IF I2C_PROBE

	} // END FOR LOOP OVER ADDRESSES

	*out_count = count;
	return ( I2C_OK );

} // END I2C SCAN


const char* i2c_status_str( i2c_status_t s ) {

    switch (s)
    {
        case I2C_OK:            return "I2C_OK";
        case I2C_ERR_BAD_PARAM: return "I2C_ERR_BAD_PARAM";
        case I2C_ERR_UNSUPPORTED:return "I2C_ERR_UNSUPPORTED";
        case I2C_ERR_TIMEOUT:   return "I2C_ERR_TIMEOUT";
        case I2C_ERR_BUS_BUSY:  return "I2C_ERR_BUS_BUSY";
        case I2C_ERR_NACK_ADDR: return "I2C_ERR_NACK_ADDR";
        case I2C_ERR_NACK_DATA: return "I2C_ERR_NACK_DATA";
        case I2C_ERR_ARB_LOST:  return "I2C_ERR_ARB_LOST";
        case I2C_ERR_BUS_ERROR: return "I2C_ERR_BUS_ERROR";
        case I2C_ERR_OVERRUN:   return "I2C_ERR_OVERRUN";
        default:                return "I2C_ERR_UNKNOWN";
    }

}

i2c_status_t i2c_mem_read( i2c_handle_t *h, uint8_t addr7, uint8_t reg, uint8_t *data, size_t len ) {

	if ( !(h) || !(h->instance) || !(data) ||
		(len == 0u) || (addr7 < MINIMUM_ADDR)
		|| (addr7 > MAXIMUM_ADDR) )  {

		return ( i2c_fail( h, I2C_ERR_BAD_PARAM ) );
	}

	I2C_TypeDef *i2c = h->instance;

	// Ensure STOP bit not stuck and put peripheral into known state
	i2c->CR1 |= I2C_CR1_ACK;
	i2c->CR1 &= ~( I2C_CR1_POS );
	i2c_clear_stop_if_stuck( i2c );

	// Ensure bus is idle, if stuck reset and re-init once -> Only one master, using superloop -> no contention
	i2c_status_t st = i2c_wait_flag_clear( h, &i2c->SR2, I2C_SR2_BUSY );
	if ( st != I2C_OK ) {
		i2c1_peripheral_reset();
		(void) i2c_init( h, h->instance, h->pclk1_hz, h->bus_hz, h->timeout_ms );

		st = i2c_wait_flag_clear( h, &i2c->SR2, I2C_SR2_BUSY );
		if ( st != I2C_OK ) return i2c_fail( h, st );
	}

	// Phase A - write register index
	// Transaction: START -> SLA+W -> reg
	// No STOP as want repeated-start for read phase

	// START condition
	i2c->CR1 |= I2C_CR1_START;
	st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_SB );
	if ( st != I2C_OK ) return i2c_fail( h, st );

	// send slave address with WRITE
	i2c->DR = (uint8_t) ( ( addr7 << 1 ) | I2C_DIR_WRITE );

	// Wait for ADDR ACK or AF NACK
	{
		uint32_t t0 = time_ms();
		while ( ( i2c->SR1 & I2C_SR1_ADDR ) == 0u ) {

			if ( i2c->SR1 & I2C_SR1_AF ) {

				// NACK on address
				(void) i2c_stop_and_wait_busy_clear( h );
				i2c->SR1 &= ~( I2C_SR1_AF );
				i2c_nack_mem_read_one( addr7 );
				return ( i2c_fail( h, I2C_ERR_NACK_ADDR ) );
			} // END IF NACK

			// Check for errors
			i2c_status_t e = i2c_map_error_and_clear( i2c );
			if ( e != I2C_OK ) return ( i2c_fail( h, e ) );

			// Check for timeout
			if ( ( time_ms() - t0 ) >= h->timeout_ms ) {
				(void) i2c_stop_and_wait_busy_clear( h );
				return ( i2c_fail( h, I2C_ERR_TIMEOUT ) );
			}

		} // END WHILE BLOCK WAITING FOR ADDR
	} // END ADDR or AF BLOCK

	// Making to this point means ACK, hence need to clear ADDR
	i2c_clear_addr( i2c );

	// Wait for TXE to be set (DR empty)
	st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_TXE );
	if ( st != I2C_OK ) {
		(void) i2c_stop_and_wait_busy_clear( h );
		return ( i2c_fail( h, st ) );
	}

	// TXE empty and no errors, put reg in DR
	i2c->DR = reg;

	// Wait for byte transfer flag to be set
	st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_BTF );
	if ( st != I2C_OK ) {
		(void) i2c_stop_and_wait_busy_clear( h );
		return ( i2c_fail( h, st ) );
	}

	// LOG - byte transfer sent for register reg in device at bus address addr

	// Phase B - repeated start read
	// Transaction - restart -> SLA+R -> receive N bytes -> STOP

	// Generate repeated start
	i2c->CR1 |= I2C_CR1_START;
	st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_SB );
	if ( st != I2C_OK ) {
		(void) i2c_stop_and_wait_busy_clear( h );
		return ( i2c_fail( h, st ) );
	}

	// Send slave address in read direction
	i2c->DR = ( ( (uint8_t) ( addr7 << 1 ) ) | I2C_DIR_READ );


	// CASE 1 - Reading exactly one byte

	if ( len == 1u ) {

		// For 1 byte, disable ACK before clearing ADDR
		// Clear ADDR
		// Set STOP
		// Wait RXNE, read the byte

		// Disable ACK
		i2c->CR1 &= ~( I2C_CR1_ACK );

		// Wait for ADDR or AF
		uint32_t t0 = time_ms();
		while ( ( i2c->SR1 & I2C_SR1_ADDR ) == 0u ) {

			// Check for NACK
			if ( i2c->SR1 & I2C_SR1_AF ) {
				(void) i2c_stop_and_wait_busy_clear( h );
				i2c->SR1 &= ~( I2C_SR1_AF );
				i2c->CR1 |= I2C_CR1_ACK; // restore default
				return i2c_fail( h, I2C_ERR_NACK_ADDR );
			}

			// Check for errors
			i2c_status_t e = i2c_map_error_and_clear( i2c );
			if ( e != I2C_OK ) {
				(void) i2c_stop_and_wait_busy_clear( h );
				i2c->CR1 |= I2C_CR1_ACK; // Restore default
				return ( i2c_fail( h, e ) );
			}

			// Check for timeout
			if ( ( time_ms() - t0 ) >= h->timeout_ms ) {
				(void) i2c_stop_and_wait_busy_clear( h );
				i2c->CR1 |= I2C_CR1_ACK;
				return ( i2c_fail( h, I2C_ERR_TIMEOUT ) );
			}

		} // END WHILE LOOP FOR ADDR

		// ADDR success, clear ADDR & generate STOP
		i2c_clear_addr( i2c );
		i2c->CR1 |= I2C_CR1_STOP;

		// Wait for RXNE flag to be set
		st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_RXNE );
		if ( st != I2C_OK ) {
			(void) i2c_stop_and_wait_busy_clear( h );
			i2c->CR1 |= I2C_CR1_ACK;
			return ( i2c_fail( h, st ) );
		}

		// RXNE flag set -> data ready to be read
		data[0] = (uint8_t) i2c->DR;

		// Restore ACK for future transaction
		i2c->CR1 |= I2C_CR1_ACK;

		// Make sure bus becomes idle
		st = i2c_wait_flag_clear( h, &i2c->SR2, I2C_SR2_BUSY );
		i2c_clear_stop_if_stuck( i2c );
		if ( st != I2C_OK ) return i2c_fail( h, st );

		// End of transaction -> data read success
		return i2c_fail( h, I2C_OK );

	} // END IF LEN == 1

	// Case 2 -> Reading exactly 2 bytes

	if ( len == 2u ) {

		// For 2 bytes:
		// Set POS = 1
		// Disable ACK
		// Wait ADDR then clear it
		// Wait BTF
		// Read 2  bytes from DR

		// Enable POS & disable ACK
		i2c->CR1 |= I2C_CR1_POS;
		i2c->CR1 &= ~( I2C_CR1_ACK );

		// Wait for ADDR or AF
		{

			uint32_t t0 = time_ms();
			while ( ( i2c->SR1 & I2C_SR1_ADDR ) == 0u ) {

				// Check for NACK
				if ( i2c->SR1 & I2C_SR1_AF ) {
					(void) i2c_stop_and_wait_busy_clear( h );
					i2c->SR1 &= ~( I2C_SR1_AF );
					i2c->CR1 |= I2C_CR1_ACK;
					i2c->CR1 &= ~( I2C_CR1_POS );
					return ( i2c_fail( h, I2C_ERR_NACK_ADDR ) );
				}

				// Check for errors
				i2c_status_t e = i2c_map_error_and_clear( i2c );
				if ( e != I2C_OK ) {
					(void) i2c_stop_and_wait_busy_clear( h );
					i2c->CR1 |= I2C_CR1_ACK;
					i2c->CR1 &= ~( I2C_CR1_POS );
					return ( i2c_fail( h, e ) );
				}

				// Check for timeout
				if ( ( time_ms() - t0 ) >= h->timeout_ms ) {
					(void) i2c_stop_and_wait_busy_clear( h );
					i2c->CR1 |= I2C_CR1_ACK;
					i2c->CR1 &= ~( I2C_CR1_POS );
					return ( i2c_fail( h, I2C_ERR_TIMEOUT ) );
				}

			} // END WHILE LOOP FOR ADDR


		} // END ADDR/AF BLOCK

		// ADDR success, clear then wait for BTF
		i2c_clear_addr( i2c );
		st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_BTF );
		if ( st != I2C_OK ) {
			(void) i2c_stop_and_wait_busy_clear( h );
			i2c->CR1 |= I2C_CR1_ACK;
			i2c->CR1 &= ~( I2C_CR1_POS );
			return ( i2c_fail( h, st ) );
		}

		// Request STOP (Both bytes in DR and shift register)
		i2c->CR1 |= I2C_CR1_STOP;

		// Read data
		data[0] = (uint8_t) i2c->DR;
		data[1] = (uint8_t) i2c->DR;

		// Restore defaults (POS = 0, ACK = 1
		i2c->CR1 |= I2C_CR1_ACK;
		i2c->CR1 &= ~( I2C_CR1_POS );

		// Wait for busy flag clear & clear stop if stuck
		st = i2c_wait_flag_clear( h, &i2c->SR2, I2C_SR2_BUSY );
		i2c_clear_stop_if_stuck( i2c );
		if ( st != I2C_OK ) return i2c_fail( h, st );

		// Transaction successful -> return success code
		return ( i2c_fail( h, I2C_OK ) );

	} // END IF LEN == 2

	// CASE 3 -> Reading 3 or more bytes

	{

		// For N > 2:
		// ACK Enabled, POS cleared
		// Wait for ADDR and clear it
		// Read bytes until 3 remain
		// Then wait BTF, clear ACK, read N-2
		// Wait BTF, set STOP, read N-1
		//Wait RXNE, read N

		// Clear POS, enable ACK
		i2c->CR1 &= ~( I2C_CR1_POS );
		i2c->CR1 |= I2C_CR1_ACK;

		// Wait for ADDR or AF
		{
			uint32_t t0 = time_ms();
			while ( ( i2c->SR1 & I2C_SR1_ADDR ) == 0u ) {

				// Check for AF
				if ( i2c->SR1 & I2C_SR1_AF ) {
					(void) i2c_stop_and_wait_busy_clear( h );
					i2c->SR1 &= ~( I2C_SR1_AF );
					return ( i2c_fail( h, I2C_ERR_TIMEOUT ) );
				}

				// Check for errors
				i2c_status_t e = i2c_map_error_and_clear( i2c );
				if ( e != I2C_OK ) {
					(void) i2c_stop_and_wait_busy_clear( h );
					return ( i2c_fail( h, e ) );
				}

				// Check for timeout
				if ( ( time_ms() - t0 ) >= h->timeout_ms ) {
					(void) i2c_stop_and_wait_busy_clear( h );
					return ( i2c_fail( h, I2C_ERR_TIMEOUT ) );
				}

			} // END WHILE LOOP FOR ADDR

		} // END ADDR/AF BLOCK

		// ADDR success -> proceed to read data

		// Clear ADDR
		i2c_clear_addr( i2c );

		size_t remaining = len;
		size_t idx       = 0;

		// Read until last 3 bytes
		while( remaining > 3u ) {

			// Check if data waiting to be read in DR
			st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_RXNE );
			if ( st != I2C_OK ) {
				(void) i2c_stop_and_wait_busy_clear( h );
				return ( i2c_fail( h, st ) );
			}

			// Read data, increment idx, decrement remaining
			data[idx++] = (uint8_t) i2c->DR;
			remaining--;

		} // END WHILE ABOVE 3 BYTES

		// At this stage, bytes remaining will be 3
		// Need special ending sequence

		// Wait for BTF to be set -> meaning DR and shift reg full
		st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_BTF );
		if ( st != I2C_OK ) {
			(void) i2c_stop_and_wait_busy_clear( h );
			return ( i2c_fail( h, st ) );
		}

		// Clear ACK, DR byte and shift reg byte already ACKed, so slave will send last byte, then receive NACK
		i2c->CR1 &= ~( I2C_CR1_ACK );

		// Read N-2 byte, increment idx, decrement remaining
		data[idx++] = (uint8_t) i2c->DR;
		remaining--;

		// Wait for BTF -> meaning final 2 bytes are in DR and shift register
		st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_BTF );
		if ( st != I2C_OK ) {
			(void) i2c_stop_and_wait_busy_clear( h );
			i2c->CR1 |= I2C_CR1_ACK;
			return ( i2c_fail( h, st ) );
		}

		// Request STOP to ensure transaction ends
		i2c->CR1 |= I2C_CR1_STOP;

		// Read N-1 byte, increment idx, decrement remaining
		data[idx++] = (uint32_t) i2c->DR;
		remaining--;

		// Last byte remaining, wait for RXNE
		st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_RXNE );
		if ( st != I2C_OK ) {
			(void) i2c_stop_and_wait_busy_clear( h );
			i2c->CR1 |= I2C_CR1_ACK;
			return ( i2c_fail( h, st ) );
		}

		// Read N,increment idx, decrement remaining (in case future code changes and for debugging)
		data[idx++] = (uint8_t) i2c->DR;
		remaining--;

		// restore default ACK
		i2c->CR1 |= I2C_CR1_ACK;

		// Ensure bus becomes idle
		st = i2c_wait_flag_clear( h, &i2c->SR2, I2C_SR2_BUSY );
		i2c_clear_stop_if_stuck( i2c );
		if ( st != I2C_OK ) return i2c_fail( h, st );

		// Transaction success, return OK
		return ( i2c_fail( h, I2C_OK ) );


	} // END 3 OR MORE BYTES BLOCK

} // END MEM READ

i2c_status_t i2c_mem_write( i2c_handle_t *h, uint8_t addr7, uint8_t reg, const uint8_t *data, size_t len ) {

	// Validate parameters
	if ( !(h) || !(h->instance) || ( addr7 < MINIMUM_ADDR ) || ( addr7 > MAXIMUM_ADDR ) ) {
		return ( i2c_fail( h, I2C_ERR_BAD_PARAM ) );
	}

	// If len > 0, then data must be non-null
	if ( ( len > 0u ) && ( data == NULL ) ) {
		return ( i2c_fail( h, I2C_ERR_BAD_PARAM ) );
	}

	I2C_TypeDef *i2c = h->instance;

	// Put peripheral in known state
	i2c->CR1 |= I2C_CR1_ACK;
	i2c->CR1 &= ~( I2C_CR1_POS );
	i2c_clear_stop_if_stuck( i2c );

	// Ensure the bus is idle
	// If stuck, reset once.
	i2c_status_t st = i2c_wait_flag_clear( h, &i2c->SR2, I2C_SR2_BUSY );
	if ( st != I2C_OK ) {

		i2c1_peripheral_reset();
		(void) i2c_init( h, h->instance, h->pclk1_hz, h->bus_hz, h->timeout_ms );

		st = i2c_wait_flag_clear( h, &i2c->SR2, I2C_SR2_BUSY );
		if ( st != I2C_OK ) return i2c_fail( h, st );

	}

	// PHASE A: START + Address write

	// Generate START & wait for START bit = 1
	i2c->CR1 |= I2C_CR1_START;
	st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_SB );
	if ( st != I2C_OK ) return i2c_fail( h, st );

	// Load address onto bus with write direction
	i2c->DR = ( (uint8_t) ( ( addr7 << 1 ) | I2C_DIR_WRITE ) );

	// Wait for ADDR ACK or AF NACK
	{

		uint32_t t0 = time_ms();
		while ( ( i2c->SR1 & I2C_SR1_ADDR ) == 0u ) {

			// Check for NACK
			if ( i2c->SR1 & I2C_SR1_AF ) {
				// STOP and ensure left idle and clear AF
				(void) i2c_stop_and_wait_busy_clear( h );
				i2c->SR1 &= ~( I2C_SR1_AF );
				return ( i2c_fail( h, I2C_ERR_NACK_ADDR ) );
			}

			// Check for errors
			i2c_status_t e = i2c_map_error_and_clear( i2c );
			if ( e != I2C_OK ) {
				// STOP and ensure left idle
				(void) i2c_stop_and_wait_busy_clear( h );
				return ( i2c_fail( h, e ) );
			}

			// Check for timeout
			if ( ( time_ms() - t0 ) >= h->timeout_ms ) {
				// STOP and ensure left idle
				(void) i2c_stop_and_wait_busy_clear( h );
				return (i2c_fail( h, I2C_ERR_TIMEOUT )  );
			}

		} // END WHILE LOOP FOR ADDR

	} // END ADDR/AF BLOCK

	// ADDR Success -> Clear ADDR
	i2c_clear_addr( i2c );

	// PHASE B: Send Register to slave

	// Wait for TXE -> transmit buffer empty: DR empty
	st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_TXE );
	if ( st != I2C_OK ) {
		(void) i2c_stop_and_wait_busy_clear( h );
		return ( i2c_fail( h, st ) );
	}

	// Ready for transmit, put reg in DR
	i2c->DR = (uint8_t) reg;

	// Wait for reg byte to fully leave shift register. TXE is only for DR. BTF means byte full sent
	// from DR and shift register

	{

		uint32_t t0 = time_ms();
		while ( ( i2c->SR1 & I2C_SR1_BTF ) == 0u ) {

			// Check for NACK (Some devices can NACK invalid registers
			if ( i2c->SR1 & I2C_SR1_AF ) {
				(void) i2c_stop_and_wait_busy_clear( h );
				i2c->SR1 &= ~( I2C_SR1_AF );
				return ( i2c_fail( h, I2C_ERR_NACK_ADDR ) );
			}

			// Check for errors and clear
			i2c_status_t e = i2c_map_error_and_clear( i2c );
			if ( e != I2C_OK ) {
				(void) i2c_stop_and_wait_busy_clear( h );
				return ( i2c_fail( h, e ) );
			}

			// Check for timeout
			if ( ( time_ms() - t0 ) >= h->timeout_ms ) {
				(void) i2c_stop_and_wait_busy_clear( h );
				return ( i2c_fail( h, I2C_ERR_TIMEOUT ) );
			}
		} // END WHILE LOOP FOR BTF

	} // END BTF BLOCK

	// At this point, device address ACKed, BTF set -> pointer in device now
	// pointing to correct register ready to be written to

	// Need to start moving bytes out into DR using a loop with len
	for ( size_t i = 0; i < len; i++ ) {

		// Wait for TXE = 1 to write byte into DR
		st = i2c_wait_flag_set( h, &i2c->SR1, I2C_SR1_TXE );
		if ( st != I2C_OK ) {
			(void) i2c_stop_and_wait_busy_clear( h );
			return ( i2c_fail( h, st ) );
		}

		// TXE set -> DR ready
		i2c->DR = data[i];

		// Byte starts shifting out -> wait for BTF while checking for NACK/errors/timeouts
		{

			uint32_t t0 = time_ms();
			while( ( i2c->SR1 & I2C_SR1_BTF ) == 0u ) {

				// Check for NACK
				if ( i2c->SR1 & I2C_SR1_AF ) {
					// Slaved NACKed, usually means write not permitted or wrong mode
					(void) i2c_stop_and_wait_busy_clear( h );
					i2c->SR1 &= ~( I2C_SR1_AF );
					return ( i2c_fail( h, I2C_ERR_NACK_DATA ) );
				}

				// Check for errors
				i2c_status_t e = i2c_map_error_and_clear( i2c );
				if ( e != I2C_OK )  {
					(void) i2c_stop_and_wait_busy_clear( h );
					return i2c_fail( h, e );
				}

				// Check for timeout
				if ( ( time_ms() - t0 ) >= h->timeout_ms ) {
					(void) i2c_stop_and_wait_busy_clear( h );
					return ( i2c_fail( h, I2C_ERR_TIMEOUT ) );
				}
			} // END WHILE BTF LOOP
		} // END BTF BLOCK
	} // END FOR LOOP WRITING DATA

	// PHASE D: STOP + Bus idle

	st = i2c_stop_and_wait_busy_clear( h );
	if ( st != I2C_OK ) return i2c_fail( h, st );

	return ( i2c_fail( h, I2C_OK ) );


}





