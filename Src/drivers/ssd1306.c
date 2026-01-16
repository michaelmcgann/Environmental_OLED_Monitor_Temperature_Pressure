
#include "drivers/ssd1306.h"

#include <string.h>

/////////////////////////////////////////////////////
////// INTERNAL HELPERS
/////////////////////////////////////////////////////

#define SSD1306_MAX_WIDTH   ( 128u )
#define SSD1306_MAX_HEIGHT  ( 64u )

// Safe I2C data chunking. Keeps transactions small
#define SSD1306_TX_CHUNK       16u

// SSD1306 commands used in init/flush
#define SSD1306_CMD_DISPLAY_OFF            0xAEu
#define SSD1306_CMD_DISPLAY_ON             0xAFu
#define SSD1306_CMD_SET_DISPLAY_CLOCK_DIV  0xD5u
#define SSD1306_CMD_SET_MULTIPLEX          0xA8u
#define SSD1306_CMD_SET_DISPLAY_OFFSET     0xD3u
#define SSD1306_CMD_SET_START_LINE         0x40u
#define SSD1306_CMD_CHARGE_PUMP            0x8Du
#define SSD1306_CMD_MEMORY_MODE            0x20u
#define SSD1306_CMD_SEG_REMAP_0            0xA0u
#define SSD1306_CMD_SEG_REMAP_1            0xA1u
#define SSD1306_CMD_COM_SCAN_INC           0xC0u
#define SSD1306_CMD_COM_SCAN_DEC           0xC8u
#define SSD1306_CMD_SET_COM_PINS           0xDAu
#define SSD1306_CMD_SET_CONTRAST           0x81u
#define SSD1306_CMD_SET_PRECHARGE          0xD9u
#define SSD1306_CMD_SET_VCOM_DETECT        0xDBu
#define SSD1306_CMD_ENTIRE_DISPLAY_RESUME  0xA4u
#define SSD1306_CMD_NORMAL_DISPLAY         0xA6u
#define SSD1306_CMD_INVERT_DISPLAY         0xA7u
#define SSD1306_CMD_DEACTIVATE_SCROLL      0x2Eu

#define SSD1306_CMD_COLUMN_ADDR            0x21u
#define SSD1306_CMD_PAGE_ADDR              0x22u


static ssd1306_status_t ssd1306_fail( ssd1306_t *dev, ssd1306_status_t e, i2c_status_t i2c_e ) {

	if ( dev ) {
		dev->last_i2c_error = i2c_e;
		dev->last_error     = e;
	}

	return ( e );
}

static size_t ssd1306_required_fb_len( ssd1306_t *dev ) {

	if ( !dev ) return 0u;
	return ( (size_t) dev->width * (size_t) dev->pages );
}

static ssd1306_status_t ssd1306_write_cmd( ssd1306_t *dev, const uint8_t *cmd, size_t len ) {

    if ( !dev || !dev->i2c || !cmd || len == 0u )
        return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

    i2c_status_t st = i2c_mem_write( dev->i2c, dev->addr7, SSD1306_CTRL_CMD, cmd, len );
    if ( st != I2C_OK ) return ssd1306_fail( dev, SSD1306_ERR_I2C, st );

    return ( ssd1306_fail( dev, SSD1306_OK, I2C_OK ) );
}

static ssd1306_status_t ssd1306_write_data_chunked( ssd1306_t *dev, const uint8_t *data, size_t len ) {

	if ( !dev || !dev->i2c || !data || len == 0u ) {
		return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );
	}

	size_t idx = 0u;
	while ( idx < len ) {

		size_t n = len - idx;
		if ( n > SSD1306_TX_CHUNK ) n = SSD1306_TX_CHUNK;

		i2c_status_t st = i2c_mem_write( dev->i2c, dev->addr7, SSD1306_CTRL_DATA, &data[idx], n );
        if ( st != I2C_OK ) return ssd1306_fail( dev, SSD1306_ERR_I2C, st );

        idx += n;
	}

	return ( ssd1306_fail( dev, SSD1306_OK, I2C_OK ) );
}

static ssd1306_status_t ssd1306_set_window_full( ssd1306_t *dev ) {

	if ( !dev ) return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

	uint8_t cmd[6];

	// Column address: 0 .. width - 1
	cmd[0] = SSD1306_CMD_COLUMN_ADDR;
	cmd[1] = 0u;
	cmd[2] = (uint8_t) ( dev->width - 1u );

	// Page address: 0 .. pages - 1
	cmd[3] = SSD1306_CMD_PAGE_ADDR;
	cmd[4] = 0u;
	cmd[5] = (uint8_t) ( dev->pages - 1u );

	return ( ssd1306_write_cmd( dev, cmd, sizeof( cmd ) ) );
}


/////////////////////////////////////////////////////
////// DEFAULT FONT
/////////////////////////////////////////////////////

#define FONT_FIRST 32u
#define FONT_LAST  90u
#define FONT_W     5u
#define FONT_H     7u

static const uint8_t s_font_5x7[( FONT_LAST - FONT_FIRST +1u ) * FONT_W] = {

	    // 32 ' ' (space)
	    0x00,0x00,0x00,0x00,0x00,

	    // 33 '!'  (blank)
	    0x00,0x00,0x00,0x00,0x00,
	    // 34 '"'
	    0x00,0x00,0x00,0x00,0x00,
	    // 35 '#'
	    0x00,0x00,0x00,0x00,0x00,
	    // 36 '$'
	    0x00,0x00,0x00,0x00,0x00,
	    // 37 '%'
	    0x00,0x00,0x00,0x00,0x00,
	    // 38 '&'
	    0x00,0x00,0x00,0x00,0x00,
	    // 39 '''
	    0x00,0x00,0x00,0x00,0x00,
	    // 40 '('
	    0x00,0x00,0x00,0x00,0x00,
	    // 41 ')'
	    0x00,0x00,0x00,0x00,0x00,
	    // 42 '*'
	    0x00,0x00,0x00,0x00,0x00,
	    // 43 '+'
	    0x00,0x00,0x00,0x00,0x00,
	    // 44 ','
	    0x00,0x00,0x00,0x00,0x00,

	    // 45 '-'  (dash)
	    0x08,0x08,0x08,0x08,0x08,

	    // 46 '.'  (dot)
	    0x00,0x60,0x60,0x00,0x00,

	    // 47 '/'  (slash)
	    0x20,0x10,0x08,0x04,0x02,

	    // 48 '0'
	    0x3E,0x51,0x49,0x45,0x3E,
	    // 49 '1'
	    0x00,0x42,0x7F,0x40,0x00,
	    // 50 '2'
	    0x62,0x51,0x49,0x49,0x46,
	    // 51 '3'
	    0x22,0x49,0x49,0x49,0x36,
	    // 52 '4'
	    0x18,0x14,0x12,0x7F,0x10,
	    // 53 '5'
	    0x2F,0x49,0x49,0x49,0x31,
	    // 54 '6'
	    0x3E,0x49,0x49,0x49,0x32,
	    // 55 '7'
	    0x01,0x71,0x09,0x05,0x03,
	    // 56 '8'
	    0x36,0x49,0x49,0x49,0x36,
	    // 57 '9'
	    0x26,0x49,0x49,0x49,0x3E,

	    // 58 ':'  (colon)
	    0x00,0x36,0x36,0x00,0x00,

	    // 59 ';' blank
	    0x00,0x00,0x00,0x00,0x00,
	    // 60 '<' blank
	    0x00,0x00,0x00,0x00,0x00,
	    // 61 '='  (equals)
	    0x14,0x14,0x14,0x14,0x14,

	    // 62 '>' blank
	    0x00,0x00,0x00,0x00,0x00,
	    // 63 '?' blank
	    0x00,0x00,0x00,0x00,0x00,
	    // 64 '@' blank
	    0x00,0x00,0x00,0x00,0x00,

	    // 65 'A'
	    0x7E,0x11,0x11,0x11,0x7E,
	    // 66 'B'
	    0x7F,0x49,0x49,0x49,0x36,
	    // 67 'C'
	    0x3E,0x41,0x41,0x41,0x22,
	    // 68 'D'
	    0x7F,0x41,0x41,0x22,0x1C,
	    // 69 'E'
	    0x7F,0x49,0x49,0x49,0x41,
	    // 70 'F'
	    0x7F,0x09,0x09,0x09,0x01,
	    // 71 'G'
	    0x3E,0x41,0x49,0x49,0x7A,
	    // 72 'H'
	    0x7F,0x08,0x08,0x08,0x7F,
	    // 73 'I'
	    0x00,0x41,0x7F,0x41,0x00,
	    // 74 'J'
	    0x20,0x40,0x41,0x3F,0x01,
	    // 75 'K'
	    0x7F,0x08,0x14,0x22,0x41,
	    // 76 'L'
	    0x7F,0x40,0x40,0x40,0x40,
	    // 77 'M'
	    0x7F,0x02,0x0C,0x02,0x7F,
	    // 78 'N'
	    0x7F,0x04,0x08,0x10,0x7F,
	    // 79 'O'
	    0x3E,0x41,0x41,0x41,0x3E,
	    // 80 'P'
	    0x7F,0x09,0x09,0x09,0x06,
	    // 81 'Q'
	    0x3E,0x41,0x51,0x21,0x5E,
	    // 82 'R'
	    0x7F,0x09,0x19,0x29,0x46,
	    // 83 'S'
	    0x46,0x49,0x49,0x49,0x31,
	    // 84 'T'
	    0x01,0x01,0x7F,0x01,0x01,
	    // 85 'U'
	    0x3F,0x40,0x40,0x40,0x3F,
	    // 86 'V'
	    0x1F,0x20,0x40,0x20,0x1F,
	    // 87 'W'
	    0x7F,0x20,0x18,0x20,0x7F,
	    // 88 'X'
	    0x63,0x14,0x08,0x14,0x63,
	    // 89 'Y'
	    0x07,0x08,0x70,0x08,0x07,
	    // 90 'Z'
	    0x61,0x51,0x49,0x45,0x43

};

static const ssd1306_font_t s_default_font = {

		.width      = FONT_W,
		.height     = FONT_H,
		.first_char = FONT_FIRST,
		.last_char  = FONT_LAST,
		.data       = s_font_5x7

};

const ssd1306_font_t* ssd1306_default_font( void ) {
	return ( &s_default_font) ;
}

/////////////////////////////////////////////////////
////// PUBLIC API
/////////////////////////////////////////////////////

ssd1306_status_t ssd1306_init( ssd1306_t *dev,
							   i2c_handle_t *i2c,
							   uint8_t addr7,
							   uint8_t width,
							   uint8_t height,
							   uint8_t *framebuffer,
							   size_t framebuffer_len,
							   ssd1306_time_ms_fn_t time_ms) {

	if ( !dev || !i2c || !framebuffer )
		return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

	// Preserve options if set before init
	uint8_t use_ext = dev->use_external_vcc;
	uint8_t flip    = dev->flip_180;

	memset( dev, 0, sizeof(*dev) );

	dev->use_external_vcc = use_ext ? 1u : 0u;
	dev->flip_180         = flip ? 1u : 0u;

    if ( width == 0u || height == 0u )        return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );
    if ( width > SSD1306_MAX_WIDTH )          return ssd1306_fail( dev, SSD1306_ERR_UNSUPPORTED, I2C_ERR_BAD_PARAM );
    if ( (height != 32u) && (height != 64u) ) return ssd1306_fail( dev, SSD1306_ERR_UNSUPPORTED, I2C_ERR_BAD_PARAM );

    uint8_t pages  = (uint8_t) ( height / 8u );
    size_t req_len = (size_t) width * (size_t) pages;

    if ( framebuffer_len != req_len )
        return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

    dev->i2c     = i2c;
    dev->addr7   = addr7;
    dev->width   = width;
    dev->height  = height;
    dev->pages   = pages;
    dev->fb      = framebuffer;
    dev->time_ms = time_ms;
    dev->font    = ssd1306_default_font();

    dev->last_error = SSD1306_OK;
    dev->last_i2c_error = I2C_OK;

    uint8_t multiplex = (uint8_t) ( height - 1u );
    uint8_t compins   = (uint8_t) ( height == 64u ) ? 0x12u : 0x02u;

    uint8_t seg_remap = dev->flip_180 ? SSD1306_CMD_SEG_REMAP_0 : SSD1306_CMD_SEG_REMAP_1;
    uint8_t com_scan  = dev->flip_180 ? SSD1306_CMD_COM_SCAN_INC : SSD1306_CMD_COM_SCAN_DEC;

    uint8_t charge_pump = dev->use_external_vcc ? 0x10u : 0x14u;
    uint8_t precharge   = dev->use_external_vcc ? 0x22u : 0xF1u;

    uint8_t contrast = ( height == 64u ) ? 0xCFu : 0x8Fu;

    const uint8_t init_cmds[] = {
        SSD1306_CMD_DISPLAY_OFF,

        SSD1306_CMD_SET_DISPLAY_CLOCK_DIV, 0x80u,
        SSD1306_CMD_SET_MULTIPLEX,         multiplex,

        SSD1306_CMD_SET_DISPLAY_OFFSET,    0x00u,
        SSD1306_CMD_SET_START_LINE | 0x00u,

        SSD1306_CMD_CHARGE_PUMP,           charge_pump,

        SSD1306_CMD_MEMORY_MODE,           0x00u,  // Horizontal addressing

        seg_remap,
        com_scan,

        SSD1306_CMD_SET_COM_PINS,          compins,
        SSD1306_CMD_SET_CONTRAST,          contrast,
        SSD1306_CMD_SET_PRECHARGE,         precharge,
        SSD1306_CMD_SET_VCOM_DETECT,       0x40u,

        SSD1306_CMD_ENTIRE_DISPLAY_RESUME,
        SSD1306_CMD_NORMAL_DISPLAY,
        SSD1306_CMD_DEACTIVATE_SCROLL,

        SSD1306_CMD_DISPLAY_ON
    };

    ssd1306_status_t st = ssd1306_write_cmd( dev, init_cmds, sizeof( init_cmds ) );
    if ( st != SSD1306_OK ) return st;

    // Clear framebuffer and push to display
    (void) ssd1306_clear( dev );
    st = ssd1306_flush( dev );
    if ( st != SSD1306_OK ) return st;

    dev->inited = 1u;
    return ssd1306_fail( dev, SSD1306_OK, I2C_OK );

}


ssd1306_status_t ssd1306_display_on( ssd1306_t *dev, uint8_t on ) {

	 if ( !dev || !dev->i2c ) return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

	    uint8_t cmd = on ? SSD1306_CMD_DISPLAY_ON : SSD1306_CMD_DISPLAY_OFF;
	    return ssd1306_write_cmd( dev, &cmd, 1u );
}

ssd1306_status_t ssd1306_clear( ssd1306_t *dev ) {

	if ( !dev || !dev->fb ) return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

	size_t n = ssd1306_required_fb_len( dev );
	memset( dev->fb, 0x00, n );

	return ( ssd1306_fail( dev, SSD1306_OK, I2C_OK ) );
}

ssd1306_status_t ssd1306_fill( ssd1306_t *dev, uint8_t pattern ) {

    if ( !dev || !dev->fb ) return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

    size_t n = ssd1306_required_fb_len( dev );
    memset( dev->fb, pattern, n );

    return ssd1306_fail( dev, SSD1306_OK, I2C_OK );
}

ssd1306_status_t ssd1306_draw_pixel( ssd1306_t *dev, uint8_t x, uint8_t y, uint8_t on ) {

    if ( !dev || !dev->fb ) return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );
    if ( x >= dev->width || y >= dev->height ) return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_OK );

    uint8_t page = (uint8_t) ( y >> 3 );
    uint8_t bit = (uint8_t) ( y & 0x7u );

    size_t idx = (size_t) page * (size_t) dev->width + (size_t) x;

    if ( on ) dev->fb[idx] |= (uint8_t)  ( 1u << bit );
    else      dev->fb[idx] &= (uint8_t) ~( 1u << bit );

    return ssd1306_fail( dev, SSD1306_OK, I2C_OK );
}

void ssd1306_set_font( ssd1306_t *dev, const ssd1306_font_t *font ) {
    if ( !dev ) return;
    dev->font = font ? font : ssd1306_default_font();
}

ssd1306_status_t ssd1306_draw_char( ssd1306_t *dev, uint8_t x, uint8_t y, char c, uint8_t on ) {

    if ( !dev || !dev->fb ) return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

    const ssd1306_font_t *f = dev->font ? dev->font : ssd1306_default_font();


    if ( !f || !f->data ) return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

    uint8_t uc = (uint8_t)c;
    if ( uc < f->first_char || uc > f->last_char ) {
        return ssd1306_fail( dev, SSD1306_OK, I2C_OK ); // ignore unknown
    }

    uint32_t glyph_index = (uint32_t) ( uc - f->first_char );
    const uint8_t *glyph = &f->data[glyph_index * f->width];

    // glyph is "width columns". each column has bits for rows
    for ( uint8_t col = 0; col < f->width; col++ ) {

    	uint8_t bits = glyph[col];

    	for ( uint8_t row = 0; row < f->height; row++ ) {

    		uint8_t px_on = ( ( bits >> row ) & 0x1u ) ? 1u : 0u;

    		if ( px_on ) {
    			(void) ssd1306_draw_pixel( dev, (uint8_t) ( x + col) , (uint8_t) ( y + row ), on );
    		} else {
    			// Leave for now
    		}
    	} // END FOR LOOP ROW
    } // END FOR LOOP COLUMNS

    return ssd1306_fail( dev, SSD1306_OK, I2C_OK );

}

ssd1306_status_t ssd1306_draw_str( ssd1306_t *dev, uint8_t x, uint8_t y, const char *s, uint8_t on )
{
    if ( !dev || !s ) return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

    const ssd1306_font_t *f = dev->font ? dev->font : ssd1306_default_font();
    uint8_t cursor_x = x;

    while ( *s ) {

        if ( cursor_x >= dev->width ) break;

        (void) ssd1306_draw_char( dev, cursor_x, y, *s, on );

        // Advance cursor: glyph width + 1 column spacing
        cursor_x = (uint8_t)(cursor_x + f->width + 1u);
        s++;
    }

    return ssd1306_fail( dev, SSD1306_OK, I2C_OK );
}

ssd1306_status_t ssd1306_flush( ssd1306_t *dev ) {

    if ( !dev || !dev->i2c || !dev->fb ) return ssd1306_fail( dev, SSD1306_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

    if ( dev->width == 0u || dev->height == 0u || dev->pages == 0u )
        return ssd1306_fail( dev, SSD1306_ERR_NOT_INIT, I2C_ERR_BAD_PARAM );

    ssd1306_status_t st = ssd1306_set_window_full( dev );
    if ( st != SSD1306_OK ) return st;

    size_t n = ssd1306_required_fb_len( dev );
    st = ssd1306_write_data_chunked( dev, dev->fb, n );
    if ( st != SSD1306_OK ) return st;

    return ssd1306_fail( dev, SSD1306_OK, I2C_OK );


}

ssd1306_status_t ssd1306_clear_and_flush( ssd1306_t *dev )
{
    ssd1306_status_t st = ssd1306_clear( dev );
    if ( st != SSD1306_OK ) return st;

    return ssd1306_flush( dev );
}

const char* ssd1306_status_str( ssd1306_status_t s )
{
    switch ( s ) {
        case SSD1306_OK:             return "SSD1306_OK";
        case SSD1306_ERR_BAD_PARAM:  return "SSD1306_ERR_BAD_PARAM";
        case SSD1306_ERR_UNSUPPORTED:return "SSD1306_ERR_UNSUPPORTED";
        case SSD1306_ERR_NOT_INIT:   return "SSD1306_ERR_NOT_INIT";
        case SSD1306_ERR_I2C:        return "SSD1306_ERR_I2C";
        case SSD1306_ERR_TIMEOUT:    return "SSD1306_ERR_TIMEOUT";
        default:                     return "SSD1306_ERR_UNKNOWN";
    }
}
