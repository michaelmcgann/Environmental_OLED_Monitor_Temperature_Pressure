
#include "utils/em_panic.h"
#include "stm32f4xx.h"

volatile em_panic_reason_t g_panic_reason;
volatile uint32_t g_panic_line;


__attribute__((noreturn)) void em_panic( em_panic_reason_t reason, const char *file, uint32_t line ) {

	(void)file; // unused for now

	g_panic_reason = reason;
	g_panic_line = line;

	__disable_irq();
	__DSB();
	__ISB();

#ifdef DEBUG
	__BKPT(0);
#endif

	while (1) {
		// sprint 0 -> do nothing
	}
}
