/* ---------------------------------------------------------------------------
 * clock.h -- system clock bring-up for the Mini-LiDAR-Board.
 * ------------------------------------------------------------------------- */

#ifndef CLOCK_H
#define CLOCK_H

#include <stdbool.h>
#include <stdint.h>

/* CMSIS convention: current SYSCLK in Hz.  Written by clock_init(). */
extern uint32_t SystemCoreClock;

/* Bring the core up to 96 MHz from the 8 MHz HSE crystal, with an exact
 * 48 MHz PLLQ for USB.  Returns false if the HSE or the PLL never became
 * ready, in which case the caller should reset rather than run at an
 * unknown clock (USB enumeration would fail anyway). */
bool clock_init(void);

/* Called from Reset_Handler before main(); enables the FPU. */
void SystemInit(void);

#endif /* CLOCK_H */
