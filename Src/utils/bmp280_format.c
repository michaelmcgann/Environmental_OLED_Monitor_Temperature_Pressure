
#include "utils/bmp280_format.h"


static int append_char( char *out, size_t out_len, size_t *idx, char c ) {

	if ( !out || !idx ) return 0;
	if ( *idx + 1 >= out_len ) return 0;

	out[(*idx)++] = c;
	out[*idx] = '\0';

	return 1;
}

static int append_u32( char *out, size_t out_len, size_t *idx, uint32_t v ) {

	char tmp[10];
	size_t n = 0;

	do {
		tmp[n++] = (char)( '0' + ( v % 10u ) );
		v /= 10u;
	} while ( v != 0u ); // example note: 1234 in binary is now 4321 ASCII character array


	// Starting at last element working backwards to create ASCII coded array
	while ( n-- ) {
		if ( !append_char( out, out_len, idx, tmp[n] ) ) return 0;
	}

	return 1;
}

static int append_s32( char *out, size_t out_len, size_t *idx, int32_t v ) {

	if ( v < 0 ) {

		if ( !append_char( out, out_len, idx, '-' ) ) return 0;

		// Get magnitude, avoiding overflow
		uint32_t magnitude = (uint32_t) ( -( v + 1 ) ) + 1u;
		return append_u32( out, out_len, idx, magnitude );
	}
	return append_u32( out, out_len, idx, (uint32_t)v );

}

size_t bmp280_sample_to_ascii( const bmp280_sample_t *sample, char *out, size_t out_len ) {

	if ( !sample || !out || out_len < 8u ) return 0;

	out[0] = '\0';
	size_t idx = 0;

	// Format temperature
	int32_t t = sample->temp_c_x100;

	if ( !append_char( out, out_len, &idx, 'T' ) ) return 0;
	if ( !append_char( out, out_len, &idx, '=' ) ) return 0;

	// Split into integer part and 2 decimals
	int32_t t_abs = t < 0 ? -t : t;
	int32_t t_int = t_abs / 100;
	int32_t t_frac = t_abs % 100;

	if ( t < 0 ) { if ( !append_char( out, out_len, &idx, '-' ) ) return 0; }
	if ( !append_s32( out, out_len, &idx, t_int ) ) return 0;
	if ( !append_char( out, out_len, &idx, '.' ) ) return 0;
	if ( !append_char( out, out_len, &idx, (char)( '0' + ( t_frac / 10 ) ) ) ) return 0;
	if ( !append_char( out, out_len, &idx, (char)( '0' + ( t_frac % 10 ) ) ) ) return 0;
	if ( !append_char( out, out_len, &idx, 'C' ) ) return 0;

	if ( !append_char( out, out_len, &idx, '\n' ) ) return 0;
	if ( !append_char( out, out_len, &idx, '\r' ) ) return 0;

	// Format Pressure
	uint32_t p = sample->press_pa;
	uint32_t p_int = p / 100u;
	uint32_t p_frac = p % 100u;

    if (!append_char(out, out_len, &idx, 'P')) return 0;
    if (!append_char(out, out_len, &idx, '=')) return 0;
    if (!append_u32(out, out_len, &idx, p_int)) return 0;
    if (!append_char(out, out_len, &idx, '.')) return 0;
    if (!append_char(out, out_len, &idx, (char)('0' + (p_frac / 10u)))) return 0;
    if (!append_char(out, out_len, &idx, (char)('0' + (p_frac % 10u)))) return 0;
    if (!append_char(out, out_len, &idx, 'H')) return 0;
    if (!append_char(out, out_len, &idx, 'P')) return 0;
    if (!append_char(out, out_len, &idx, 'A')) return 0;

    return idx;

}

int bmp280_sample_to_ascii_split(
    const bmp280_sample_t *sample,
    char *out_temp,  size_t out_temp_len,
    char *out_press, size_t out_press_len
) {
    if ( !sample || !out_temp || !out_press ) return 0;
    if ( out_temp_len < 8u || out_press_len < 8u ) return 0;


    {
        out_temp[0] = '\0';
        size_t idx = 0;

        int32_t t = sample->temp_c_x100;

        if ( !append_char( out_temp, out_temp_len, &idx, 'T' ) ) return 0;
        if ( !append_char( out_temp, out_temp_len, &idx, '=' ) ) return 0;

        int32_t t_abs  = (t < 0) ? -t : t;
        int32_t t_int  = t_abs / 100;
        int32_t t_frac = t_abs % 100;

        if ( t < 0 ) {
            if ( !append_char( out_temp, out_temp_len, &idx, '-' ) ) return 0;
        }

        if ( !append_s32( out_temp, out_temp_len, &idx, t_int ) ) return 0;
        if ( !append_char( out_temp, out_temp_len, &idx, '.' ) ) return 0;

        // always 2 digits
        if ( !append_char( out_temp, out_temp_len, &idx, (char)('0' + (t_frac / 10)) ) ) return 0;
        if ( !append_char( out_temp, out_temp_len, &idx, (char)('0' + (t_frac % 10)) ) ) return 0;

        if ( !append_char( out_temp, out_temp_len, &idx, 'C' ) ) return 0;
    }

    {
        out_press[0] = '\0';
        size_t idx = 0;

        uint32_t p = sample->press_pa;

        uint32_t p_int  = p / 100u;
        uint32_t p_frac = p % 100u;

        if ( !append_char( out_press, out_press_len, &idx, 'P' ) ) return 0;
        if ( !append_char( out_press, out_press_len, &idx, '=' ) ) return 0;

        if ( !append_u32( out_press, out_press_len, &idx, p_int ) ) return 0;
        if ( !append_char( out_press, out_press_len, &idx, '.' ) ) return 0;

        // always 2 digits
        if ( !append_char( out_press, out_press_len, &idx, (char)('0' + (p_frac / 10u)) ) ) return 0;
        if ( !append_char( out_press, out_press_len, &idx, (char)('0' + (p_frac % 10u)) ) ) return 0;

        if ( !append_char( out_press, out_press_len, &idx, 'H' ) ) return 0;
        if ( !append_char( out_press, out_press_len, &idx, 'P' ) ) return 0;
        if ( !append_char( out_press, out_press_len, &idx, 'A' ) ) return 0;
    }

    return 1;
}

