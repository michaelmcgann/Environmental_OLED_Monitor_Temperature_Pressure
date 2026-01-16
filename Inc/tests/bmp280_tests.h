
#include "drivers/bmp280.h"

#ifndef TESTS_BMP280_TESTS_H_
#define TESTS_BMP280_TESTS_H_


const char* bmp280_status_str( bmp280_status_t s );

void bmp280_test_status( bmp280_t *dev );

void bmp280_test_dig_P1_not_zero( bmp280_t *dev );

void bmp280_read_osrs_p_reg( bmp280_t *dev );

void bmp280_error_in_block( bmp280_t *dev);

void bmp280_test_read_raw( bmp280_t *dev );

void bmp280_test_compensate( bmp280_t *dev );


#endif /* TESTS_BMP280_TESTS_H_ */
