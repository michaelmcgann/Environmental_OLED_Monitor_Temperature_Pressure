
#include <string.h>

#include "tests/i2c_tests.h"
#include "utils/log.h"

#include "utils/hex.h"

////////////////////////////////
///// Useful data for tests
////////////////////////////////

static const char*  s;
static char         cc[2];

static const uint8_t EOL[]    = "\r\n";
static const uint8_t PREFIX[] = "I2C ADDRESS/DATA: 0x";
static const uint8_t SPACE[]  = " ";

////////////////////////////////
///// Tests
////////////////////////////////

void i2c_test_current_status( i2c_handle_t *h ) {

	i2c_handle_t *h_i2c1 = h;
	s = i2c_status_str( h_i2c1->last_error );
	log_write( ( const uint8_t * ) s, strlen(s) );
	log_write( EOL, sizeof(EOL) - 1u );
	log_write( EOL, sizeof(EOL) - 1u );
}

void i2c_test_for_acks_0x3C_0x76( i2c_handle_t *h ) {

	i2c_status_t status = i2c_probe( h, (uint8_t) 0x3C );
	s = i2c_status_str( status );
	log_write( ( const uint8_t * ) s, strlen(s) );
	log_write( SPACE, sizeof(SPACE) - 1u );

	hex_u8_to_ascii( ( uint8_t ) 0x3C, cc );
	log_write( PREFIX, sizeof(PREFIX) - 1u );
	log_write( ( const uint8_t *) cc, 2u );
	log_write( SPACE, sizeof(SPACE) - 1u );

	log_write( EOL, sizeof(EOL) - 1u );

	status = i2c_probe( h, (uint8_t) 0x76 );
	s = i2c_status_str( status );
	log_write( ( const uint8_t * ) s, strlen(s) );
	log_write( SPACE, sizeof(SPACE) - 1u );

	hex_u8_to_ascii( ( uint8_t ) 0x76, cc );
	log_write( PREFIX, sizeof(PREFIX) - 1u );
	log_write( ( const uint8_t *) cc, 2u );
	log_write( SPACE, sizeof(SPACE) - 1u );

	log_write( EOL, sizeof(EOL) - 1u );
	log_write( EOL, sizeof(EOL) - 1u );


}

void i2c_test_read_bmp280_id_register( i2c_handle_t *h ) {

	uint8_t data[1];
	uint8_t addr7 = 0x76;
	size_t len = 1u;
	uint8_t reg = 0xD0;
	// i2c_status_t i2c_mem_read( i2c_handle_t *h, uint8_t addr7, uint8_t reg, uint8_t *data, size_t len );
	i2c_status_t st = i2c_mem_read( h, addr7, reg, data, len );

	uint8_t ID = data[0];

	s = i2c_status_str( st );
	log_write( (const uint8_t *) s, strlen(s) );
	log_write( EOL, sizeof(EOL) - 1u );
	s = "ID for BMP280: ";
	log_write( (const uint8_t *) s, strlen(s) );

	log_write( PREFIX, sizeof(PREFIX) - 1u );
	hex_u8_to_ascii( ( uint8_t ) ID, cc );
	log_write( (const uint8_t *) cc, 2u );

	log_write( EOL, sizeof(EOL) - 1u );
	log_write( EOL, sizeof(EOL) - 1u );

}



////////////////////////////////
///// Debug
////////////////////////////////

void i2c_nack_mem_read_one( uint8_t addr ) {

	hex_u8_to_ascii( addr, cc );
	s = "NACK on mem_read: 1";
	log_write( (const uint8_t *) s, strlen(s) );
	log_write( EOL, sizeof(EOL) - 1u );

	s = "Address: ";
	log_write( (const uint8_t *) s, strlen(s) );
	log_write( (const uint8_t *) cc, 2u );

}

void i2c_flag_block() {

	s = "error in this block.\r\n";
	log_write( (const uint8_t *) s, strlen( s ) );
	log_write( EOL, sizeof(EOL) - 1u );
	log_write( EOL, sizeof(EOL) - 1u );


}


