

#ifndef UTILS_EM_PANIC_H_
#define UTILS_EM_PANIC_H_

#include <stdint.h>

typedef enum {
	EM_PANIC_ASSERT = 1, // internal invariant failed
	EM_PANIC_HARDFAULT,
	EM_PANIC_UNREACHABLE,
	EM_PANIC_DRIVER_FATAL
} em_panic_reason_t;


extern volatile em_panic_reason_t g_panic_reason;
extern volatile uint32_t          g_panic_line;

// Panic entry point, no return
__attribute__((noreturn)) void em_panic( em_panic_reason_t reason, const char *file, uint32_t line );


// Convenience macros
#define EM_PANIC(r) em_panic( (r), __FILE__, (uint32_t) __LINE__ )

#define EM_ASSERT(x) do { if ( !(x) ) { EM_PANIC( EM_PANIC_ASSERT ); } } while (0)


#endif /* UTILS_EM_PANIC_H_ */
