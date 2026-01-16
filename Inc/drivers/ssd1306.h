
#include <stdint.h>
#include <stddef.h>

#include "drivers/i2c.h"

#ifndef DRIVERS_SSD1306_H_
#define DRIVERS_SSD1306_H_

/////////////////////////////////////////////////////
////// CONSTANTS (I2C address and control bytes)
/////////////////////////////////////////////////////

#define SSD1306_I2C_ADDR        (0x3Cu)


#define SSD1306_CTRL_CMD        (0x00u)   // Control byte: following bytes are commands
#define SSD1306_CTRL_DATA       (0x40u)   // Control byte: following bytes are display data

/////////////////////////////////////////////////////
////// TYPES
/////////////////////////////////////////////////////

typedef enum {

    SSD1306_OK = 0,
    SSD1306_ERR_BAD_PARAM,
    SSD1306_ERR_UNSUPPORTED,
    SSD1306_ERR_NOT_INIT,
    SSD1306_ERR_I2C,
    SSD1306_ERR_TIMEOUT,

} ssd1306_status_t;

// Time function pointer
typedef uint32_t (*ssd1306_time_ms_fn_t)(void);

typedef struct {

    uint8_t         width;       // glyph width in pixels
    uint8_t         height;      // glyph height in pixels
    uint8_t         first_char;  // ASCII start
    uint8_t         last_char;   // ASCII end
    const uint8_t  *data;        // glyph bitmap data

} ssd1306_font_t;

typedef struct {

	// Bus
	i2c_handle_t *i2c;
	uint8_t       addr7;

	// Geometry
    uint8_t       width;
    uint8_t       height;
    uint8_t       pages;

    // Framebuffer
    uint8_t      *fb;
	size_t       fb_len;

	// Options
	uint8_t       use_external_vcc;  // 0 = charge pump, 1 = external VCC
	uint8_t       flip_180;         // 1 = rotate display 180 degrees
	uint8_t       inited;

	// Text
	const ssd1306_font_t *font; // Optional - if NULL driver will use internal default

	// Status
    ssd1306_status_t      last_error;
    i2c_status_t          last_i2c_error;

    // Timing
    ssd1306_time_ms_fn_t  time_ms;


} ssd1306_t;


/////////////////////////////////////////////////////
////// CONFIG / INIT
/////////////////////////////////////////////////////

/*
 * Initialize the display driver and program the SSD1306 into a known state.
 *
 * framebuffer must point to a buffer of size (width * height / 8).
 * Example for 128x64: 128 * 64 / 8 = 1024 bytes.
 */
ssd1306_status_t ssd1306_init( ssd1306_t *dev,
							   i2c_handle_t *i2c,
							   uint8_t addr7,
							   uint8_t width,
							   uint8_t height,
							   uint8_t *framebuffer,
							   size_t framebuffer_len,
							   ssd1306_time_ms_fn_t time_ms
								);


/////////////////////////////////////////////////////
////// FRAMEBUFFER OPS (no I2C)
/////////////////////////////////////////////////////

// Clear framebuffer to 0 (black).
ssd1306_status_t ssd1306_clear( ssd1306_t *dev );

// Fill framebuffer: 0x00=black, 0xFF=white. For testing.
ssd1306_status_t ssd1306_fill( ssd1306_t *dev, uint8_t pattern );

// Pixel primitives. out-of-range coords are ignored.
ssd1306_status_t ssd1306_draw_pixel( ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t on );
ssd1306_status_t ssd1306_draw_hline( ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t w, uint8_t on );
ssd1306_status_t ssd1306_draw_vline( ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t h, uint8_t on );

/////////////////////////////////////////////////////
////// TEXT (renders into framebuffer)
/////////////////////////////////////////////////////

void ssd1306_set_font( ssd1306_t *dev, const ssd1306_font_t *font );
const ssd1306_font_t* ssd1306_default_font( void );

ssd1306_status_t ssd1306_draw_char( ssd1306_t *dev, uint8_t x, uint8_t y, char c, uint8_t on );
ssd1306_status_t ssd1306_draw_str( ssd1306_t *dev, uint8_t x, uint8_t y, const char *s, uint8_t on );

/////////////////////////////////////////////////////
////// DISPLAY UPDATE (I2C)
/////////////////////////////////////////////////////

ssd1306_status_t ssd1306_display_on( ssd1306_t *dev, uint8_t on );

ssd1306_status_t ssd1306_flush( ssd1306_t *dev );

/* Convenience: clear framebuffer then flush */
ssd1306_status_t ssd1306_clear_and_flush( ssd1306_t *dev );

/////////////////////////////////////////////////////
////// DEBUG
/////////////////////////////////////////////////////

const char* ssd1306_status_str( ssd1306_status_t s );


#endif /* DRIVERS_SSD1306_H_ */
