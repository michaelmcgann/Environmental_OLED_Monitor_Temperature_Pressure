

#include <stdint.h>

#include "utils/log.h"
#include "utils/hex.h"
#include "bsp/led.h"
#include "bsp/timebase.h"
#include "app/heartbeat.h"
#include "drivers/i2c.h"
#include "drivers/bmp280.h"
#include "tests/i2c_tests.h"
#include "tests/bmp280_tests.h"
#include "drivers/ssd1306.h"
#include "tests/ssd1306_tests.h"
#include "app/UI_handler.h"

#include <string.h>


#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void) {

	///////////////////////////////////
	///// INIT
	///////////////////////////////////
	SystemCoreClockUpdate();

	log_init();
	led_init();
	timebase_init();
	heartbeat_init( LED_HEARTBEAT, 500 );

	i2c_handle_t h_i2c1;
	i2c_init_default( &h_i2c1, I2C1 );

	bmp280_t bmp280;
	bmp280_init( &bmp280, &h_i2c1, 0x76, time_ms );
	bmp280_test_status( &bmp280 );

	uint8_t fb[(128 * 64) / 8];
	ssd1306_t ssd1306;
	memset( &ssd1306, 0, sizeof(ssd1306) );
	ssd1306_init( &ssd1306, &h_i2c1, 0x3C, (uint8_t) 128u, (uint8_t) 64u, fb, sizeof( fb ), time_ms );


	///////////////////////////////////
	///// UART TEST
	///////////////////////////////////

	const uint8_t boot[] = "BOOT\r\n";
	const size_t size = sizeof(boot) - 1u;
	log_write( boot, size );

	///////////////////////////////////
	///// I2C TESTS
	///////////////////////////////////

	i2c_test_current_status( &h_i2c1 );
	i2c_test_for_acks_0x3C_0x76( &h_i2c1 );

	///////////////////////////////////
	///// SSD1306 Test
	///////////////////////////////////

	ssd1306_test_turn_on( &ssd1306 );
//	ssd1306_test_full_white( &ssd1306 );

	ui_refresh( &bmp280, &ssd1306 );

	///////////////////////////////////
	///// INFINITE WHILE LOOP
	///////////////////////////////////

	uint32_t i2c_last_test_time = time_ms();

	while (1) {

		heartbeat_tick();

		if ( time_elapsed( &i2c_last_test_time, 500 ) ) {

//			i2c_test_for_acks_0x3C_0x76( &h_i2c1 );
//
//			bmp280_test_status( &bmp280 );
//
//			i2c_test_current_status( &h_i2c1 );
//
//			ssd1306_test_status( &ssd1306 );
//
//			bmp280_test_compensate( &bmp280 );

			ui_refresh( &bmp280, &ssd1306 );

		}
	}
}
