/* ---------------------------------------------------------------------------
 * iwdg.c -- independent watchdog configuration.
 *
 * Timing (RM0383 section 14.3):
 *   LSI is nominally 32 kHz on the STM32F411 (datasheet range 17-47 kHz --
 *   it is an uncalibrated RC oscillator, so the real timeout can be roughly
 *   0.34x to 0.94x of the nominal figure).
 *   Prescaler /32  ->  1 kHz counter tick
 *   Reload   500   ->  500 ms nominal timeout
 *
 * The main loop refreshes at well over 1 kHz, so even the worst-case fast LSI
 * (47 kHz, giving a ~340 ms timeout) leaves an enormous margin.
 * ------------------------------------------------------------------------- */

#include "iwdg.h"
#include "board.h"

/* Key register magic values (RM0383 14.4.1). */
#define IWDG_KEY_REFRESH  0xAAAAU
#define IWDG_KEY_WRITE    0x5555U
#define IWDG_KEY_START    0xCCCCU

void iwdg_init(void)
{
    /* The LSI must be running before the IWDG will count.  Starting the IWDG
     * would enable the LSI on its own, but waiting for LSIRDY here keeps the
     * timeout deterministic from the very first reload. */
    RCC->CSR |= RCC_CSR_LSION;
    while ((RCC->CSR & RCC_CSR_LSIRDY) == 0U) {
        /* typically a few hundred microseconds */
    }

    /* Freeze the watchdog whenever the core is halted by the debugger.
     * Without this, every SWD breakpoint trips the IWDG and resets the board
     * out from under the debug session. */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    /* ORDER MATTERS (RM0383 14.3, and it is the order ST's own driver uses).
     *
     * The IWDG must be STARTED first.  Its PR/RLR shadow registers are
     * clocked by the watchdog's own clock domain, and that domain does not
     * run until the 0xCCCC key has been written.  Writing PR/RLR before
     * starting the IWDG sets PVU/RVU in SR with nothing to ever clear them,
     * so the "wait for SR" loop below would spin forever -- which is exactly
     * what it did before this was fixed.
     */
    IWDG->KR  = IWDG_KEY_START;      /* start counting -- irreversible        */
    IWDG->KR  = IWDG_KEY_WRITE;      /* unlock PR and RLR                     */
    IWDG->PR  = 3U;                  /* prescaler /32 -> 1 kHz                */
    IWDG->RLR = IWDG_TIMEOUT_MS;     /* 500 ticks = 500 ms                    */

    /* PR and RLR are written across a clock domain crossing; the status bits
     * stay set until the new values have actually landed.  This is bounded:
     * the transfer takes a handful of LSI cycles, and if something is wrong
     * with the LSI we would rather run with the reset-default 256 ms timeout
     * than refuse to boot.  A watchdog must never be able to hang the thing
     * it is supposed to protect. */
    for (uint32_t spins = 0U; (IWDG->SR != 0U) && (spins < 1000000U); spins++) {
        /* wait for PVU and RVU to clear */
    }

    IWDG->KR = IWDG_KEY_REFRESH;     /* load the counter                      */
}

void iwdg_refresh(void)
{
    IWDG->KR = IWDG_KEY_REFRESH;
}
