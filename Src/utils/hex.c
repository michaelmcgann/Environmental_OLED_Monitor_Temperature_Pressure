#include "utils/hex.h"

// convert 8 bit value to two uppercase hex ASCII characters
void hex_u8_to_ascii( uint8_t v, char out[2] ) {

	static const char HEX[] = "0123456789ABCDEF";
	out[0] = HEX[ ( v >> 4 ) & 0xFu ]; // High nibble
	out[1] = HEX[ ( v & 0xFu ) ]; // low nibble

}

// convert 20 bit value into hex ASCII
void hex_u20_to_ascii( uint32_t v , char out[5] ) {

	for ( int i = 0; i < 5; i++ ) {

		uint8_t nib = (uint8_t) ( ( v >> ( ( 4 - i ) * 4 ) ) & 0xFu );
		out[i] = ( nib < 10u ) ? (char)( '0' + nib ) : (char)( 'A' + ( nib - 10u ) );

	}

}

