
#include "utils/log.h"
#include "bsp/log_backend.h"

#include <stdio.h>

static log_level_t s_log_level = LOG_LEVEL_INFO;

em_status_t log_init( void ) {

#if LOG_ENABLE
	return log_backend_init();
#else
	return EM_OK;
#endif
}

void log_set_level( log_level_t level ) {
	s_log_level = level;
}

log_level_t log_get_level( void ) {
	return s_log_level;
}

em_status_t log_write( const uint8_t *data, size_t len ) {

#if LOG_ENABLE
	return log_backend_write( data, len );
#else
	(void)data;
	(void)len;
	return EM_OK;
#endif
}

static const char *log_level_tag( log_level_t level ) {

	switch ( level ) {
	case LOG_LEVEL_ERROR:   return "E";
	case LOG_LEVEL_WARN:    return "W";
	case LOG_LEVEL_INFO:    return "I";
	case LOG_LEVEL_DEBUG: return "D";
	default:                return "?";
	}

}

em_status_t log_printf_level( log_level_t level, const char *fmt, ...) {
#if LOG_ENABLE
	va_list ap;
	va_start(ap, fmt);
	em_status_t status = log_vprintf_level(level, fmt, ap);
	va_end(ap);
	return status;
#else
	(void)level;
	(void)fmt;
	return EM_OK;
#endif
}

em_status_t log_vprintf_level( log_level_t level, const char *fmt, va_list ap ) {

#if LOG_ENABLE

	if ( fmt == NULL ) return EM_E_NULL;

	if ( level > s_log_level ) return EM_OK;

	char line[LOG_MAX_LINE];
	int n = vsnprintf( line, sizeof( line ), fmt, ap );

	if ( n < 0 ) return EM_E_STATE;

	size_t len;
	if ( (size_t) n >= sizeof(line) ) len = sizeof(line) - 1u;
	else len = (size_t) n;

	const char *tag = log_level_tag( level );
	char prefix[5];
    prefix[0] = '[';
    prefix[1] = tag[0];
    prefix[2] = ']';
    prefix[3] = ' ';
    prefix[4] = '\0';

    em_status_t status = log_backend_write( (const uint8_t *) prefix , 4u );

    if ( status != EM_OK ) return status;

    return log_backend_write( (const uint8_t *) line, len );

#else
    (void) level;
    (void) fmt;
    (void) ap;
    return EM_OK;

#endif

}




