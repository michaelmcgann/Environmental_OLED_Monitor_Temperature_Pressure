#include "drivers/i2c.h"

#ifndef TESTS_I2C_TESTS_H_
#define TESTS_I2C_TESTS_H_

void i2c_test_current_status( i2c_handle_t *h );

void i2c_test_for_acks_0x3C_0x76( i2c_handle_t *h );

void i2c_test_read_bmp280_id_register( i2c_handle_t *h );

void i2c_nack_mem_read_one( uint8_t addr );

void i2c_flag_block();



#endif /* TESTS_I2C_TESTS_H_ */
