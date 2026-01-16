

#ifndef UTILS_HEX_H_
#define UTILS_HEX_H_

#include <stdint.h>

// convert 8 bit value to two uppercase hex ASCII characters
void hex_u8_to_ascii( uint8_t v, char out[2] );

void hex_u20_to_ascii( uint32_t v , char out[5] );


#endif /* UTILS_HEX_H_ */
