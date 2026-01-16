
#include "app/UI_handler.h"
#include "utils/bmp280_format.h"

const uint32_t  timeout_ms = 100u;
bmp280_sample_t sample;
char            temp_str[16];
char            press_str[20];

void ui_refresh( bmp280_t *bmp280, ssd1306_t *ssd1306 ) {

	bmp280_status_t st = bmp280_read_forced_blocking( bmp280, &sample, timeout_ms );


	if ( ( st == BMP280_OK ) && bmp280_sample_to_ascii_split( &sample, temp_str, sizeof(temp_str), press_str, sizeof(press_str) ) ) {
		ssd1306_clear( ssd1306 );
		ssd1306_draw_str( ssd1306, 0, 0, temp_str, 1 );
		ssd1306_draw_str( ssd1306, 0, 16, press_str, 1 );
		ssd1306_flush( ssd1306 );
	}


}



