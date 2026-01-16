
#include "tests/bmp280_tests.h"
#include "utils/log.h"
#include <string.h>
#include "utils/hex.h"
#include "utils/bmp280_format.h"


static const char* s;
static const uint8_t EOL[]    = "\r\n";
static const uint8_t PREFIX[] = "0x";
static char cc[2];
static const char* dig_P1_prefix = "dig_P1 value: ";


void bmp280_test_status( bmp280_t *dev ) {

	s = bmp280_status_str( dev->last_error );
	log_write( (const uint8_t *) s, strlen( s ) );
	log_write( EOL, sizeof(EOL) - 1u );
	log_write( EOL, sizeof(EOL) - 1u );


}

void bmp280_test_dig_P1_not_zero( bmp280_t *dev ) {

	log_write( (const uint8_t *) dig_P1_prefix, strlen( dig_P1_prefix ) );

	uint16_t dig_P1 = dev->calib.dig_P1;
	uint8_t dig_P1_MSB = (uint8_t) ( ( dig_P1 >> 8 ) & 0xFFu );
	uint8_t dig_P1_LSB = (uint8_t) ( ( dig_P1 & 0xFFu ) );

	hex_u8_to_ascii( dig_P1_MSB, cc );

	log_write( PREFIX, sizeof(PREFIX) - 1u );
	log_write( (const uint8_t *) cc, 2u );

	hex_u8_to_ascii( dig_P1_LSB, cc );
	log_write( (const uint8_t *) cc, 2u );

	log_write( EOL, sizeof(EOL) - 1u );
	log_write( EOL, sizeof(EOL) - 1u );

}

void bmp280_read_osrs_p_reg( bmp280_t *dev ) {

	uint8_t reg = 0xF4;
	uint8_t data;
	size_t len = 1u;

	i2c_status_t st = i2c_mem_read( dev->i2c, dev->addr7, reg, &data, len ); // should be 0x24

	s = "Inside bmp280_read_osrs_p_reg\r\n ";
	log_write( (const uint8_t *) s, strlen( s ) );

	log_write( EOL, sizeof(EOL) - 1u );
	s = i2c_status_str( st );
	log_write( (const uint8_t *) s, strlen( s ) );
	log_write( EOL, sizeof(EOL) - 1u );

	s = "register value should be 0x24\n\r";
	log_write( (const uint8_t *) s, strlen( s ) );

	s = "Actual value: ";
	log_write( (const uint8_t *) s, strlen( s ) );

	hex_u8_to_ascii( data, cc );

	log_write( PREFIX, sizeof(PREFIX) - 1u );
	log_write( (const uint8_t *) cc, 2u );
	log_write( EOL, sizeof(EOL) - 1u );

	s = "end of test.\n\r";
	log_write( (const uint8_t *) s, strlen( s ) );
	log_write( EOL, sizeof(EOL) - 1u );


}

void bmp280_error_in_block( bmp280_t *dev ) {
	s = "Error happened in flagged block.\r\n";
	log_write( (const uint8_t *) s, strlen( s ) );

	s = "I2C handle status: ";
	log_write( (const uint8_t *) s, strlen( s ) );
	s = i2c_status_str( dev->i2c->last_error );
	log_write( (const uint8_t *) s, strlen( s ) );
	log_write( EOL, sizeof(EOL) - 1u );

	s = "I2C last BMP280 status: ";
	log_write( (const uint8_t *) s, strlen( s ) );
	s = i2c_status_str( dev->last_i2c_error );
	log_write( (const uint8_t *) s, strlen( s ) );
	log_write( EOL, sizeof(EOL) - 1u );

	s = "BMP280 last status: ";
	log_write( (const uint8_t *) s, strlen( s ) );
	s = bmp280_status_str( dev->last_error );
	log_write( (const uint8_t *) s, strlen( s ) );
	log_write( EOL, sizeof(EOL) - 1u );
	log_write( EOL, sizeof(EOL) - 1u );

}

void bmp280_test_read_raw( bmp280_t *dev ) {

	s = "Testing read raw values...\r\n";
	log_write( (const uint8_t *)s, strlen( s ) );

	bmp280_raw_t raw;

	s = "Forcing trigger...\r\n";
	log_write( (const uint8_t *)s, strlen( s ) );
	bmp280_status_t st = bmp280_trigger_forced( dev );
	s = bmp280_status_str( st );
	log_write( (const uint8_t *)s, strlen( s ) );

	s = "Waiting for measuring flag to clear...\r\n";
	log_write( (const uint8_t *)s, strlen( s ) );
	st = bmp280_wait_measuring_clear( dev, 100 );
	s = bmp280_status_str( st );
	log_write( (const uint8_t *)s, strlen( s ) );

	s = "Reading raw values...\r\n";
	log_write( (const uint8_t *)s, strlen( s ) );
	st = bmp280_read_raw( dev, &raw );
	s = bmp280_status_str( st );
	log_write( (const uint8_t *)s, strlen( s ) );
	log_write( EOL, sizeof(EOL) - 1u );

	if ( st == BMP280_OK ) {

		char raw_array[5];

		s = "Temp raw (adc_T) 0x";
		log_write( (const uint8_t *)s, strlen( s ) );
		hex_u20_to_ascii( (uint32_t) raw.adc_T , raw_array );
		log_write( (const uint8_t *) raw_array, sizeof(raw_array) );

		log_write( EOL, sizeof(EOL) - 1u );
		log_write( EOL, sizeof(EOL) - 1u );

		s = "Pressure raw (adc_P) 0x";
		log_write( (const uint8_t *)s, strlen( s ) );
		hex_u20_to_ascii( (uint32_t) raw.adc_P , raw_array );
		log_write( (const uint8_t *) raw_array, sizeof(raw_array) );

		log_write( EOL, sizeof(EOL) - 1u );
		log_write( EOL, sizeof(EOL) - 1u );

	}
}

void bmp280_test_compensate( bmp280_t *dev ) {

	s = "Testing compensate...\n\r";
	log_write( (const uint8_t *)s, strlen( s ) );

	bmp280_sample_t sample;
	uint32_t timeout_ms = 100u;

	bmp280_status_t st = bmp280_read_forced_blocking( dev, &sample, timeout_ms );

	if ( st == BMP280_OK ) {

		char data[48];
		bmp280_sample_to_ascii( &sample, data, sizeof(data) );
		log_write( (const uint8_t *) data, strlen( data ) );
		log_write( EOL, sizeof(EOL) - 1u );
		log_write( EOL, sizeof(EOL) - 1u );

	} else {
		s = "Compensate test error.\n\r";
		log_write( (const uint8_t *)s, strlen( s ) );
		s = bmp280_status_str( st );
		log_write( (const uint8_t *)s, strlen( s ) );
		log_write( EOL, sizeof(EOL) - 1u );
		log_write( EOL, sizeof(EOL) - 1u );
	}

}



const char* bmp280_status_str( bmp280_status_t s ) {

	switch ( s ) {

		case BMP280_OK:              return "BMP280_OK";
		case BMP280_ERR_BAD_PARAM:   return "BMP280_ERR_BAD_PARAM";
		case BMP280_ERR_I2C:         return "BMP280_ERR_I2C";
		case BMP280_ERR_BAD_CHIP_ID: return "BMP280_ERR_BAD_CHIP_ID";
		case BMP280_ERR_TIMEOUT:     return "BMP280_ERR_TIMEOUT";
		case BMP280_ERR_NOT_READY:   return "BMP280_ERR_NOT_READY";
		default:                     return "BMP280_ERR_UNKNOWN";

	}
}
