/* ---------------------------------------------------------------------------
 * iwdg.h -- independent watchdog.
 *
 * The IWDG runs from the LSI RC oscillator, which keeps ticking even if the
 * PLL or the HSE crystal dies.  That is exactly the failure we want covered:
 * if the main loop stops refreshing it -- deadlock in a driver, a stuck spin
 * on a peripheral flag, a wild jump into Default_Handler -- the board resets
 * itself and re-enumerates instead of going quiet.
 *
 * There are no LEDs to signal a fault with, so a reset is also the only
 * failure indication a user standing over the board can see: the USB serial
 * device disappears and comes back.
 * ------------------------------------------------------------------------- */

#ifndef IWDG_H
#define IWDG_H

#include <stdint.h>

/* Start the watchdog.  Once started it cannot be stopped except by reset. */
void iwdg_init(void);

/* Reload the counter.  Must be called more often than the timeout. */
void iwdg_refresh(void);

/* Nominal timeout, for documentation and for the host README. */
#define IWDG_TIMEOUT_MS  500U

#endif /* IWDG_H */
