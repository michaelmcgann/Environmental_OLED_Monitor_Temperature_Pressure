

#ifndef BSP_LOG_BACKEND_H_
#define BSP_LOG_BACKEND_H_

#include <stddef.h>
#include <stdint.h>

#include "utils/em_status.h"

em_status_t log_backend_init( void );

/**
 * @brief Write raw bytes to the log output backend.
 *
 * @param data Pointer to bytes to send (must not be NULL if len > 0)
 * @param len  Number of bytes to send
 *
 * @return EM_OK on success; otherwise an error describing the failure.
 */
em_status_t log_backend_write( const uint8_t *data, size_t len );


#endif /* BSP_LOG_BACKEND_H_ */
