

#ifndef DRIVERS_I2C_H_
#define DRIVERS_I2C_H_


#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "utils/em_status.h"
#include "stm32f4xx.h"

typedef enum
{
    I2C_OK = 0,

    /* Parameter / configuration errors */
    I2C_ERR_BAD_PARAM      = -1,
    I2C_ERR_UNSUPPORTED    = -2,

    /* Runtime / bus errors */
    I2C_ERR_TIMEOUT        = -10,
    I2C_ERR_BUS_BUSY       = -11,
    I2C_ERR_NACK_ADDR      = -12,
    I2C_ERR_NACK_DATA      = -13,
    I2C_ERR_ARB_LOST       = -14,
    I2C_ERR_BUS_ERROR      = -15,
    I2C_ERR_OVERRUN        = -16,
} i2c_status_t;


// Standard speed more suitable for breadboards
typedef enum {
	I2C_SPEED_STANDARD = 100000u,
	IC2_SPEED_FAST = 400000u
} i2c_speed_t;

typedef struct {
	I2C_TypeDef *instance;
	uint32_t pclk1_hz;
	uint32_t bus_hz;
	uint32_t timeout_ms;
	i2c_status_t last_error;

} i2c_handle_t;

const char* i2c_status_str( i2c_status_t s );

i2c_status_t i2c_init_default( i2c_handle_t *h, I2C_TypeDef *instance );

i2c_status_t i2c_init( i2c_handle_t *h, I2C_TypeDef *instance, uint32_t pclk1_hz, uint32_t bus_hz, uint32_t timeout_ms );

static inline void i2c_set_timeout( i2c_handle_t *h, uint32_t timeout_ms ) {
	if ( h ) { h->timeout_ms = timeout_ms; }
}

// Probe a device, check if ACKs
i2c_status_t i2c_probe( i2c_handle_t *h, uint8_t addr7 );

// Write raw bytes to slave - no internal reg address
i2c_status_t i2c_write( i2c_handle_t *h, uint8_t addr7, const uint8_t *data, size_t len );

// Read raw bytes from a slave - no internal reg address
i2c_status_t i2c_read( i2c_handle_t *h, uint8_t addr7, uint8_t *data, size_t len );

// Write to device register
i2c_status_t i2c_mem_write( i2c_handle_t *h, uint8_t addr7, uint8_t reg, const uint8_t *data, size_t len );

i2c_status_t i2c_mem_read( i2c_handle_t *h, uint8_t addr7, uint8_t reg, uint8_t *data, size_t len );

// Scan the bus and collect ACKing addresses
i2c_status_t i2c_scan( i2c_handle_t *h, uint8_t *out_addrs, size_t max_addrs, size_t *out_count );

const char* i2c_status_str( i2c_status_t s );

#endif /* DRIVERS_I2C_H_ */
