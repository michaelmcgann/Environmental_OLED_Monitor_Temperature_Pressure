

#ifndef UTILS_LOG_H_
#define UTILS_LOG_H_

#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>

#include "utils/em_status.h"


#ifndef LOG_ENABLE
#define LOG_ENABLE 1
#endif

#ifndef LOG_MAX_LINE
#define LOG_MAX_LINE 128u
#endif

#ifndef LOG_DEFAULT_LEVEL
#define LOG_DEFAULT_LEVEL LOG_LEVEL_INFO
#endif

typedef enum {
	LOG_LEVEL_ERROR = 0,
	LOG_LEVEL_WARN  = 1,
	LOG_LEVEL_INFO  = 2,
	LOG_LEVEL_DEBUG = 3
} log_level_t;


em_status_t log_init( void );

void log_set_level( log_level_t level );

log_level_t log_get_level( void );

/**
 * @brief Printf-style logging with explicit severity level.
 *
 * @param level  Severity (ERROR/WARN/INFO/DEBUG)
 * @param fmt    printf-style format string
 * @return EM_OK on success, otherwise an em_status_t describing the failure.
 *
 * Notes (Sprint 0):
 * - Not intended to be called from ISRs.
 * - Output is best-effort; failures should not crash the system.
 */
em_status_t log_printf_level( log_level_t level, const char *fmt, ... );

em_status_t log_vprintf_level( log_level_t level, const char *fmt, va_list ap );

em_status_t log_write( const uint8_t *data, size_t len );

#if LOG_ENABLE

#define LOGE(...) (void)log_printf_level( LOG_LEVEL_ERROR, __VA_ARGS )
#define LOGW(...) (void)log_printf_level( LOG_LEVEL_WARN, __VA_ARGS )
#define LOGI(...) (void)log_printf_level( LOG_LEVEL_INFO, __VA_ARGS )
#define LOGD(...) (void)log_printf_level( LOG_LEVEL_DEGUB, __VA_ARGS )

#else

#define LOGE(...) do {} while (0)
#define LOGW(...) do {} while (0)
#define LOGI(...) do {} while (0)
#define LOGD(...) do {} while (0)

#endif


#endif /* UTILS_LOG_H_ */
