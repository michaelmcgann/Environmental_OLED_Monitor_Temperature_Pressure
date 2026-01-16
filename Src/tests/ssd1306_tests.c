
#include <string.h>

#include "tests/ssd1306_tests.h"
#include "utils/log.h"

static const char* s;
static const uint8_t EOL[]    = "\r\n";
static const uint8_t PREFIX[] = "0x";

static void log_str( const char *s ) {

    if ( !s ) return;
    (void) log_write( (const uint8_t*)s, strlen(s) );
}

void ssd1306_test_status( ssd1306_t *dev ) {

	s = ssd1306_status_str( dev->last_error );
	log_write( (const uint8_t *) s, strlen( s ) );
	log_write( EOL, sizeof(EOL) - 1u );
	log_write( EOL, sizeof(EOL) - 1u );


}

void ssd1306_test_turn_on( ssd1306_t *dev ) {

    if ( !dev ) {
        log_str("SSD1306_TEST: oled=NULL\r\n");
        return;
    }

    // 1) Force display ON (sends 0xAF)
    ssd1306_status_t st = ssd1306_display_on( dev, 1u );
    if ( st != SSD1306_OK ) {
        log_str("SSD1306_TEST: display_on FAILED\r\n");
        return;
    }

    // 2) Clear framebuffer and flush it (blank screen)
    st = ssd1306_clear_and_flush( dev );
    if ( st != SSD1306_OK ) {
        log_str("SSD1306_TEST: clear_and_flush FAILED\r\n");
        return;
    }

    // 3) Draw something so you can visually confirm it woke up
    (void) ssd1306_draw_str( dev, 0u, 0u, "YH IM NICKED", 1u );
    st = ssd1306_flush( dev );
    if ( st != SSD1306_OK ) {
        log_str("SSD1306_TEST: flush FAILED\r\n");
        return;
    }

    log_str("SSD1306_TEST: OK\r\n");
}

void ssd1306_test_full_white( ssd1306_t *dev ) {

    if ( !dev ) return;

    // Force display ON
    (void) ssd1306_display_on( dev, 1u );

    // Fill framebuffer with 0xFF (all pixels ON)
    (void) ssd1306_fill( dev, 0xFFu );
    (void) ssd1306_flush( dev );
}
