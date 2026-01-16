
#ifndef DRIVERS_BMP280_H_
#define DRIVERS_BMP280_H_

#include <stdint.h>
#include "drivers/i2c.h"

/////////////////////////////////////////////////////
////// CONSTANTS - Register addresses, IDs, commands
/////////////////////////////////////////////////////

#define BMP280_I2C_ADDR         ( 0x76u )
#define BMP280_REG_ID           ( 0xD0u )
#define BMP280_REG_RESET        ( 0xE0u )
#define BMP280_REG_STATUS       ( 0xF3u )
#define BMP280_REG_CTRL_MEAS    ( 0xF4u )
#define BMP280_REG_CONFIG       ( 0xF5u )
#define BMP280_REG_PRESS_MSB    ( 0xF7u )
#define BMP280_REG_TEMP_MSB     ( 0xFAu )
#define BMP280_CAL_START        ( 0x88u )

#define BMP280_CHIP_ID          ( 0x58u )
#define BMP280_SOFTRESET_CMD    ( 0xB6u )

// Status bits
#define BMP280_STATUS_MEASURING_MASK ( 1u << 3 )
#define BMP280_STATUS_IM_UPDATE_MASK    ( 1u << 0 )

// CTRL_MEAS fields
#define BMP280_CTRL_MEAS_OSRS_T_Pos  ( 5u )
#define BMP280_CTRL_MEAS_OSRS_P_Pos  ( 2u )
#define BMP280_CTRL_MEAS_MODE_Pos    ( 0u )
#define BMP280_CTRL_OSRS_BIT_MASK    ( 7u )
#define BMP280_MODE_MASK             ( 3u )

// CONFIG fields
#define BMP280_CONFIG_TSB_Pos        ( 5u )
#define BMP280_CONFIG_FILTER_Pos     ( 2u )
#define BMP280_CONFIG_STANDBY_MASK   ( 7u )
#define BMP280_CONFIG_FILTER_MASK    ( 7u )

// Timeout in ms
#define BMP280_TIMEOUT_MS ( 50u )

// Set up error enum
typedef enum {
	BMP280_OK = 0,

	BMP280_ERR_BAD_PARAM = -1,
	BMP280_ERR_I2C = -2,
    BMP280_ERR_BAD_CHIP_ID = -3,
    BMP280_ERR_TIMEOUT = -4,
    BMP280_ERR_NOT_READY = -5

} bmp280_status_t;

typedef enum {
    BMP280_MODE_SLEEP  = 0u, /* 00 */
    BMP280_MODE_FORCED = 1u, /* 01 (10 also forced) */
    BMP280_MODE_NORMAL = 3u  /* 11 */
} bmp280_mode_t;

// Over sampling encodings
typedef enum {
    BMP280_OSRS_SKIPPED = 0u,
    BMP280_OSRS_X1      = 1u,
    BMP280_OSRS_X2      = 2u,
    BMP280_OSRS_X4      = 3u,
    BMP280_OSRS_X8      = 4u,
    BMP280_OSRS_X16     = 5u
} bmp280_osrs_t;

// IIR filter coefficient options
typedef enum {
    BMP280_FILTER_OFF = 0u,
    BMP280_FILTER_2   = 1u,
    BMP280_FILTER_4   = 2u,
    BMP280_FILTER_8   = 3u,
    BMP280_FILTER_16  = 4u
} bmp280_filter_t;

// Standby time bitfield values (used in normal mode)
typedef enum {
    BMP280_TSB_0P5MS  = 0u,
    BMP280_TSB_62P5MS = 1u,
    BMP280_TSB_125MS  = 2u,
    BMP280_TSB_250MS  = 3u,
    BMP280_TSB_500MS  = 4u,
    BMP280_TSB_1000MS = 5u,
    BMP280_TSB_2000MS = 6u,
    BMP280_TSB_4000MS = 7u
} bmp280_tsb_t;

// Trimming coefficients from NVM (0x88 - 0xA1)
typedef struct {

    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

} bmp280_calib_t;

// Raw ADC values (20 bit stored in 32 bit containers
typedef struct {
	int32_t adc_T;
	int32_t adc_P;
} bmp280_raw_t;

// Compensated outputs
typedef struct {
	int32_t temp_c_x100;
	uint32_t press_pa;
} bmp280_sample_t;

// timebase hook for blocking waits
typedef uint32_t (*bmp280_time_ms_fn_t) (void);

// Device handle. Keeping cached copies of ctrl_meas/config so trigger forced can preserve OSRS bits
typedef struct {
	i2c_handle_t  *i2c;
	uint8_t        addr7;

	bmp280_calib_t calib;
	int32_t        t_fine;

	uint8_t        ctrl_meas_cached;
	uint8_t        config_cached;

	bmp280_time_ms_fn_t time_ms;

	bmp280_status_t last_error;
	i2c_status_t    last_i2c_error;

} bmp280_t;


// Configuration bundle
typedef struct {

	bmp280_osrs_t   osrs_t;
	bmp280_osrs_t   osrs_p;
	bmp280_filter_t filter;
	bmp280_tsb_t    standby;

} bmp280_settings_t;

/////////////////////////////////////////////////////
////// API
/////////////////////////////////////////////////////

// Initialise device handle and verify chip ID
bmp280_status_t bmp280_init( bmp280_t *dev, i2c_handle_t *i2c, uint8_t addr7, bmp280_time_ms_fn_t time_ms );

// Read chip ID
bmp280_status_t bmp280_read_chip_id( bmp280_t *dev, uint8_t *out_id );

// Soft reset - write 0xB6 to 0xE0
bmp280_status_t bmp280_soft_reset( bmp280_t *dev );

// Read calibration coefficients from 0x88..0xA1
bmp280_status_t bmp280_read_calibration( bmp280_t *dev );

// Configure over sampling, filter and standby. Config writes should be done in SLEEP mode.
bmp280_status_t bmp280_configure( bmp280_t *dev, const bmp280_settings_t *settings );

// Trigger a single forced measurement
bmp280_status_t bmp280_trigger_forced( bmp280_t *dev );

// Read STATUS register and report measuring/im_update bits
bmp280_status_t bmp280_read_status( bmp280_t *dev, uint8_t *out_status );

// blocking wait until measuring bit clears
bmp280_status_t bmp280_wait_measuring_clear( bmp280_t *dev, uint32_t timeout_ms );

// burst read raw pressure and temperature from 0xF7 to 0xFC
// Output adc_p and adc_t are 20 bit values stored in 32
bmp280_status_t bmp280_read_raw( bmp280_t *dev, bmp280_raw_t *out_raw );

//  Convert raw ADC -> compensated values using calibration coefficients.
//  Produces: temp_c_x100 (0.01°C) press_pa (Pa)
bmp280_status_t bmp280_compensate( bmp280_t *dev, const bmp280_raw_t *raw, bmp280_sample_t *out_sample );

// Convenience method -> one end to end forced sample
bmp280_status_t bmp280_read_forced_blocking( bmp280_t *dev, bmp280_sample_t *out_sample, uint32_t timeout_ms );

// Converts bmp280 status into a c string for debugging
const char *bmp280_status_str( bmp280_status_t st );



#endif /* DRIVERS_BMP280_H_ */
