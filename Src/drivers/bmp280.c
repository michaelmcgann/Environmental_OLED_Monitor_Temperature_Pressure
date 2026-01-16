#include "drivers/bmp280.h"
#include <string.h>

//////////////////////////////////
///// INTERNAL HELPERS
/////////////////////////////////

static bmp280_status_t bmp280_fail( bmp280_t *dev, bmp280_status_t err, i2c_status_t i2c_err ) {

	if ( dev ) {
		dev->last_error = err;
		dev->last_i2c_error = i2c_err;
	}
	return err;
}

// Read 1 byte from BMP280 register
static bmp280_status_t bmp280_read_u8( bmp280_t *dev, uint8_t reg, uint8_t *out ) {

	// Check for null pointers
	if ( !dev || !dev->i2c || !out ) return bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

	i2c_status_t st = i2c_mem_read( dev->i2c, dev->addr7, reg, out, 1u );
	if ( st != I2C_OK ) {
		return bmp280_fail( dev, BMP280_ERR_I2C, st );
	}

	return ( bmp280_fail( dev, BMP280_OK, I2C_OK ) );
}

// Write 1 byte to s BMP280 register
static bmp280_status_t bmp280_write_u8( bmp280_t *dev, uint8_t reg, uint8_t val ) {

	// Check for null pointers
	if ( !dev || !dev->i2c ) return bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

	i2c_status_t st = i2c_mem_write( dev->i2c, dev->addr7, reg, &val, 1u );
	if ( st != I2C_OK ) {
		return bmp280_fail( dev, BMP280_ERR_I2C, st );
	}

	return ( bmp280_fail( dev, BMP280_OK, I2C_OK ) );
}

// Little-endian combine helper LSB first in memory
static inline uint16_t le_u16( const uint8_t *p ) {

	return ( (uint16_t)( (uint16_t) p[0] | ( (uint16_t) p[1] << 8 ) ) );
}

static inline int16_t le_s16( const uint8_t *p ) {

	return ( (int16_t) le_u16( p ) );
}


/////////////////////////////////
///// PUBLIC HELPERS
/////////////////////////////////

// Reads bmp280 status register and copies value into out_status. bit 3 -> measuring. bit 0 -> im_update
bmp280_status_t bmp280_read_status( bmp280_t *dev, uint8_t *out_status ) {

	return ( bmp280_read_u8( dev, BMP280_REG_STATUS, out_status ) );
}

bmp280_status_t bmp280_read_chip_id( bmp280_t *dev, uint8_t *out_id ) {

	return ( bmp280_read_u8( dev, BMP280_REG_ID, out_id ) );
}

// Read calibration coefficients from 0x88..0xA1
bmp280_status_t bmp280_read_calibration( bmp280_t *dev ) {

	// 0x88 to 0x9F inclusive is 24 bytes
	uint8_t buf[24];
	i2c_status_t st = i2c_mem_read( dev->i2c, dev->addr7, BMP280_CAL_START, buf, sizeof(buf) );
	if ( st != I2C_OK ) return bmp280_fail( dev, BMP280_ERR_I2C, st );

	// Bytes come out little endian -> lower address of each 8 bit pair holds LSB

	// TEMP
	dev->calib.dig_T1 = le_u16( &buf[0] );
	dev->calib.dig_T2 = le_s16( &buf[2] );
	dev->calib.dig_T3 = le_s16( &buf[4] );

	// PRESSURE
	dev->calib.dig_P1 = le_u16( &buf[6] );
	dev->calib.dig_P2 = le_s16( &buf[8] );
	dev->calib.dig_P3 = le_s16( &buf[10] );
    dev->calib.dig_P4 = le_s16( &buf[12] );
    dev->calib.dig_P5 = le_s16( &buf[14] );
    dev->calib.dig_P6 = le_s16( &buf[16] );
    dev->calib.dig_P7 = le_s16( &buf[18] );
    dev->calib.dig_P8 = le_s16( &buf[20] );
    dev->calib.dig_P9 = le_s16( &buf[22] );

    return ( bmp280_fail( dev, BMP280_OK, I2C_OK ) );
}

// bmp280 init
bmp280_status_t bmp280_init( bmp280_t *dev, i2c_handle_t *i2c, uint8_t addr7, bmp280_time_ms_fn_t time_ms ) {

	// Check for null
	if ( !dev || !i2c ) return bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

	// Clear struct so start from a known state
	memset( dev, 0, sizeof(*dev) );

	dev->i2c     = i2c;
	dev->addr7   = addr7;
	dev->time_ms = time_ms;

	dev->last_error     = BMP280_OK;
	dev->last_i2c_error = I2C_OK;

	// Step one -> verify chip ID
	{
		uint8_t id = 0;
		bmp280_status_t st = bmp280_read_chip_id( dev, &id );
		if ( st != BMP280_OK ) return st; // Errors set by called function

		if ( id != BMP280_CHIP_ID ) return bmp280_fail( dev, BMP280_ERR_BAD_CHIP_ID, I2C_OK );
	}

	// Wait for im_update flag to clear
	{

		uint8_t status = 0;
		bmp280_status_t st = bmp280_read_status( dev, &status );
		if ( st != BMP280_OK ) return st;

		if ( ( status & BMP280_STATUS_IM_UPDATE_MASK ) && dev->time_ms ) {

			// Read
			uint32_t t0 = dev->time_ms();
			while (1) {

				st = bmp280_read_status( dev, &status );
				if ( st != BMP280_OK ) return st;

				// Check im_update cleared
				if ( ( status & BMP280_STATUS_IM_UPDATE_MASK ) == 0u ) break;

				// Check timeout
				if ( ( dev->time_ms() - t0 ) >= BMP280_TIMEOUT_MS ) return bmp280_fail( dev, BMP280_ERR_TIMEOUT, I2C_OK );
			} // END WHILE
		} // END IF IM_UPDATE CHECK
	} // END IM_UPDATE BLOCK


	// Read trimming/calibration coefficients (0x88 to 0x9F)
	{

		bmp280_status_t st = bmp280_read_calibration( dev );
		if ( st != BMP280_OK ) return st;

		// dig_P1 must be none zero since pressure compensation uses it as denominator
		if ( dev->calib.dig_P1 == 0u ) return bmp280_fail( dev, BMP280_ERR_NOT_READY, I2C_OK );

	} // END CALIBRATION BLOCK


	// Cache a known configuration for forced-mode. Chip is kept in sleep mode until later forced sampling
	{

		// CTRL_MEAS
		const bmp280_osrs_t osrs_t = BMP280_OSRS_X1;
		const bmp280_osrs_t osrs_p = BMP280_OSRS_X1;

		uint32_t ctrl = (
						( ( (uint32_t) osrs_t & BMP280_CTRL_OSRS_BIT_MASK ) << BMP280_CTRL_MEAS_OSRS_T_Pos ) |
						( ( (uint32_t) osrs_p & BMP280_CTRL_OSRS_BIT_MASK ) << BMP280_CTRL_MEAS_OSRS_P_Pos ) |
						( ( (uint32_t) BMP280_MODE_SLEEP & BMP280_MODE_MASK ) << BMP280_CTRL_MEAS_MODE_Pos  )
						); // 00100100 - 0x24

		dev->ctrl_meas_cached = (uint8_t) ctrl;

		// CONFIG - t_sb (standby time) irrelevant in forced mode but put in known state. filter is relevant
		const bmp280_filter_t filter    = BMP280_FILTER_OFF;
		const bmp280_tsb_t    standby   = BMP280_TSB_1000MS;

		uint32_t config = (
				 ( ( (uint32_t) standby & BMP280_CONFIG_STANDBY_MASK ) << BMP280_CONFIG_TSB_Pos ) |
				 ( ( (uint32_t) filter & BMP280_CONFIG_FILTER_MASK ) << BMP280_CONFIG_FILTER_Pos )
				 );

		dev->config_cached = (uint8_t) config;

	} // END OF CONFIG/CONTROL BLOCK

	// Write cached registers to device
	{

		// Write ctrl_meas first as config writes not guaranteed unless in normal mode
		bmp280_status_t st = bmp280_write_u8( dev, BMP280_REG_CTRL_MEAS, dev->ctrl_meas_cached );
		if ( st != BMP280_OK ) {
			return st;
		}

		st = bmp280_write_u8( dev, BMP280_REG_CONFIG, dev->config_cached );
		if ( st != BMP280_OK ) return st;

		dev->t_fine = 0;

		return ( bmp280_fail( dev, BMP280_OK, I2C_OK ) );
	}

}

bmp280_status_t bmp280_trigger_forced( bmp280_t *dev ) {

	if ( !dev || !dev->i2c ) return bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

	// ctrl meas layout
	// [7:5] = osrs_t, [4:2] = osrs_p, [1:0] = mode
	uint8_t ctrl = dev->ctrl_meas_cached;
	ctrl &= (uint8_t) ~( BMP280_MODE_MASK << BMP280_CTRL_MEAS_MODE_Pos );
	ctrl |= (uint8_t) ( ( BMP280_MODE_FORCED & BMP280_MODE_MASK ) << BMP280_CTRL_MEAS_MODE_Pos );

	// Writing forced mode starts a single measurement
	bmp280_status_t st = bmp280_write_u8( dev, BMP280_REG_CTRL_MEAS, ctrl );
	if ( st != BMP280_OK ) return st;

	return ( bmp280_fail( dev, BMP280_OK, I2C_OK ) );
}

bmp280_status_t bmp280_wait_measuring_clear( bmp280_t *dev, uint32_t timeout_ms ) {

	if ( !dev || !dev->i2c || !dev->time_ms || ( timeout_ms == 0u ) ) {
		return bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );
	}

	uint32_t t0 = dev->time_ms();
	uint8_t status = 0;

	while ( 1 ) {

		bmp280_status_t st = bmp280_read_status( dev, &status );
		if ( st != BMP280_OK ) return st;

		// Check if measuring flag is zero
		if ( ( status & BMP280_STATUS_MEASURING_MASK ) == 0u ) {
			return bmp280_fail( dev, BMP280_OK, I2C_OK ); // STATUS OK
		}

		if ( ( dev->time_ms() - t0 ) >= timeout_ms ) {
			return ( bmp280_fail( dev, BMP280_ERR_TIMEOUT, I2C_OK ) );
		}
	} // END WHILE LOOP FOR MEAS FLAG
}

bmp280_status_t bmp280_read_raw( bmp280_t *dev, bmp280_raw_t *out_raw ) {

	if ( !dev || !dev->i2c || !out_raw ) {
		return ( bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM ) );
	}

	uint8_t buf[6];

	// Burst read from pressure MSB
	i2c_status_t st = i2c_mem_read( dev->i2c, dev->addr7, BMP280_REG_PRESS_MSB, buf, sizeof(buf) );
	if ( st != I2C_OK ) return bmp280_fail( dev, BMP280_ERR_I2C, st );

	// unpack values as such:
	/*
	 * adc_P = [F7:MSB][F8:LSB][F9:XLSB(7:4)]
	 * adc_T = [FA:MSB][FB:LSB][FC:XLSB(7:4)]
	 */
	int32_t adc_P = ( ( (int32_t)buf[0] << 12u ) | ( (int32_t)buf[1] << 4u ) | ( (int32_t)buf[2] >> 4 ) );
	int32_t adc_T = ( ( (int32_t)buf[3] << 12u ) | ( (int32_t)buf[4] << 4u ) | ( (int32_t)buf[5] >> 4 ) );

	out_raw->adc_P = adc_P;
	out_raw->adc_T = adc_T;

	return ( bmp280_fail( dev, BMP280_OK, I2C_OK ) );
}

bmp280_status_t bmp280_soft_reset( bmp280_t *dev ) {

	if ( !dev || !dev->i2c ) {
		return bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );
	}

	// Writing RESET CMD to RESET_REG, then wait for trimming parameters to be copied from NVM to image registers
	// re-read calib and re-apply cached values
	bmp280_status_t st = bmp280_write_u8( dev, BMP280_REG_RESET, (uint8_t) BMP280_SOFTRESET_CMD );
	if ( st != BMP280_OK ) return st;

	// Wait for im_update flag to clear
	if ( dev->time_ms ) {

		uint32_t t0 = dev->time_ms();
		uint8_t status = 0;

		while ( 1 ) {

			st = bmp280_read_status( dev, &status );
			if ( st != BMP280_OK ) return st;

			if ( ( status & BMP280_STATUS_IM_UPDATE_MASK ) == 0u ) break;

			if ( ( dev->time_ms() - t0 ) >= BMP280_TIMEOUT_MS ) {
				return bmp280_fail( dev, BMP280_ERR_TIMEOUT, I2C_OK );
			}
		} // END WHILE

		// Re-read calibration after reset
		st = bmp280_read_calibration( dev );
		if ( st != BMP280_OK ) return st;

		// Re-apply cache registers
		{

			uint8_t ctrl = dev->ctrl_meas_cached;
			ctrl &= (uint8_t) ~( (uint8_t)( BMP280_MODE_MASK << BMP280_CTRL_MEAS_MODE_Pos ) );
			ctrl |= (uint8_t) ( ( BMP280_MODE_SLEEP & BMP280_MODE_MASK ) << BMP280_CTRL_MEAS_MODE_Pos );

			st = bmp280_write_u8( dev, BMP280_REG_CTRL_MEAS, ctrl );
			if ( st != BMP280_OK ) return st;

			st = bmp280_write_u8( dev, BMP280_REG_CONFIG, dev->config_cached );
			if ( st != BMP280_OK ) return st;
		}
	}

	return bmp280_fail( dev, BMP280_OK, I2C_OK );
}

// Configure over sampling, filter and standby. Should be done in sleep mode.
bmp280_status_t bmp280_configure( bmp280_t *dev, const bmp280_settings_t *settings ) {

	if ( !dev || !dev->i2c || !settings ) {
		return bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );
	}

	// Range validation, done this way for readability
	if ( settings->osrs_t > BMP280_OSRS_X16 )    return bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );
	if ( settings->osrs_p > BMP280_OSRS_X16 )    return bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );
	if ( settings->filter > BMP280_FILTER_16 )   return bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );
	if ( settings->standby > BMP280_TSB_4000MS ) return bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );

	// build ctrl, for this sleep mode needs to be forced
	uint8_t ctrl = 0u;
	ctrl |= (uint8_t) ( ( (uint32_t)settings->osrs_t & BMP280_CTRL_OSRS_BIT_MASK ) << BMP280_CTRL_MEAS_OSRS_T_Pos );
	ctrl |= (uint8_t) ( ( (uint32_t)settings->osrs_p & BMP280_CTRL_OSRS_BIT_MASK ) << BMP280_CTRL_MEAS_OSRS_P_Pos ) ;
	ctrl |= (uint8_t) ( ( (uint32_t)BMP280_MODE_SLEEP & BMP280_MODE_MASK ) << BMP280_CTRL_MEAS_MODE_Pos ) ;

	// Build config
	uint8_t config = 0u;
	config |= (uint8_t)( ( (uint32_t)settings->standby & BMP280_CONFIG_STANDBY_MASK ) << BMP280_CONFIG_TSB_Pos );
	config |= (uint8_t)( ( (uint32_t)settings->filter & BMP280_CONFIG_FILTER_MASK ) << BMP280_CONFIG_FILTER_Pos );

	// Write sleep first
	bmp280_status_t st = bmp280_write_u8( dev, BMP280_REG_CTRL_MEAS, ctrl );
	if ( st != BMP280_OK ) return st;

	st = bmp280_write_u8( dev, BMP280_REG_CONFIG, config );
	if ( st != BMP280_OK ) return st;

	// Cache for later use. trigger_forced uses cached values
	dev->ctrl_meas_cached = ctrl;
	dev->config_cached    = config;

	return bmp280_fail( dev, BMP280_OK, I2C_OK );
}

// Convert raw ADC -> compensated values using calibration coefficients
// Produces: temp_c_x100 (0.01 C) press_pa (Pa)
bmp280_status_t bmp280_compensate( bmp280_t *dev, const bmp280_raw_t *raw, bmp280_sample_t *out_sample ) {

	if ( !dev || !raw || !out_sample ) {
		return bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );
	}

	// dig_P1 is a divisor, check for zero value
	if ( dev->calib.dig_P1 == 0u ) {
		return bmp280_fail( dev, BMP280_ERR_NOT_READY, I2C_OK );
	}

	// Temperature compensation
	int32_t adc_T = raw->adc_T;

	int32_t var1 = (int32_t)(((((adc_T >> 3) - ((int32_t)dev->calib.dig_T1 << 1)) )
				* ((int32_t)dev->calib.dig_T2)) ) >> 11;

	int32_t var2 = (int32_t)(((((((adc_T >> 4) - ((int32_t)dev->calib.dig_T1)) * ((adc_T >> 4)
				- ((int32_t)dev->calib.dig_T1))) ) >> 12) * ((int32_t)dev->calib.dig_T3)) ) >> 14;

	dev->t_fine = var1 + var2;

	// Temperature in 0.01 C
	int32_t T = ( ( dev->t_fine * 5 ) + 128 ) >> 8;

	out_sample->temp_c_x100 = T;


	// Pressure compensation
	int32_t adc_P = raw->adc_P;

	int64_t p;
	int64_t v1;
	int64_t v2;

	v1 = ((int64_t)dev->t_fine) - 128000LL;
	v2 = v1 * v1 * (int64_t)dev->calib.dig_P6;
	v2 = v2 + ((v1 * (int64_t)dev->calib.dig_P5) << 17);
	v2 = v2 + (((int64_t)dev->calib.dig_P4) << 35);

	v1 = ((v1 * v1 * (int64_t)dev->calib.dig_P3) >> 8) + ((v1 * (int64_t)dev->calib.dig_P2) << 12);
	v1 = (((((int64_t)1) << 47) + v1) * (int64_t)dev->calib.dig_P1) >> 33;

	if (v1 == 0) { // avoid division by zero
		return bmp280_fail( dev, BMP280_ERR_NOT_READY, I2C_OK );
	}

	p = 1048576LL - (int64_t)adc_P;
	p = (((p << 31) - v2) * 3125LL) / v1;

	v1 = ((int64_t)dev->calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	v2 = ((int64_t)dev->calib.dig_P8 * p) >> 19;

	p = ((p + v1 + v2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);

	out_sample->press_pa = (uint32_t)((p + 128LL) >> 8);

	return bmp280_fail( dev, BMP280_OK, I2C_OK );

}

bmp280_status_t bmp280_read_forced_blocking( bmp280_t *dev, bmp280_sample_t *out_sample, uint32_t timeout_ms ) {

	if ( !dev || !out_sample ) {
			return bmp280_fail( dev, BMP280_ERR_BAD_PARAM, I2C_ERR_BAD_PARAM );
		}

	// 1. Start single conversion
	bmp280_status_t st = bmp280_trigger_forced( dev );
	if ( st != BMP280_OK ) return st;

	// 2. wait for measuring clear
	st = bmp280_wait_measuring_clear( dev, timeout_ms );
	if ( st != BMP280_OK ) return st;

	// 3. Burst read raw values
	bmp280_raw_t raw;
	st = bmp280_read_raw( dev, &raw );
	if ( st != BMP280_OK ) return st;

	// 4. Compensate
	return bmp280_compensate( dev, &raw, out_sample );

}




