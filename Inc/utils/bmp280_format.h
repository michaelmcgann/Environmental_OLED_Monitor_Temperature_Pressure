

#ifndef UTILS_BMP280_FORMAT_H_
#define UTILS_BMP280_FORMAT_H_

#include <stddef.h>
#include <stdint.h>
#include "drivers/bmp280.h"

// returns number of chars written excluding '\0' or 0 if error
size_t bmp280_sample_to_ascii( const bmp280_sample_t *sample, char *out, size_t out_len );

int bmp280_sample_to_ascii_split(
    const bmp280_sample_t *sample,
    char *out_temp,  size_t out_temp_len,
    char *out_press, size_t out_press_len
);


#endif /* UTILS_BMP280_FORMAT_H_ */
