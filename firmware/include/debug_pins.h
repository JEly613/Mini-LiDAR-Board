/* ---------------------------------------------------------------------------
 * debug_pins.h -- the board's only observable outputs.
 *
 * THIS BOARD HAS NO GPIO STATUS LEDS.  D3 is a plain green power indicator
 * wired across the 3.3 V rail through R4 with no MCU connection, so there is
 * nothing for firmware to blink.  Header J4 brings out three spare GPIOs;
 * putting a logic analyser or scope on them is the only way to see firmware
 * timing without the USB link.
 *
 *   DEBUG_A  PB3  toggles once per IMU data-ready (expect a 500 Hz square
 *                 wave when the IMU is streaming at 1 kHz)
 *   DEBUG_B  PB4  toggles once per Madgwick update (also 500 Hz, and its
 *                 phase relative to A shows the filter latency)
 *   DEBUG_C  PB5  toggles once per packet queued for USB (50 Hz at the
 *                 default 100 Hz packet rate)
 * ------------------------------------------------------------------------- */

#ifndef DEBUG_PINS_H
#define DEBUG_PINS_H

#include "board.h"

void debug_pins_init(void);

/* BSRR is write-only and atomic, so these are safe from any context and need
 * no read-modify-write.  ODR is read to work out which half of BSRR to hit. */
static inline void debug_pin_toggle(uint32_t pin)
{
    if ((DEBUG_PORT->ODR & (1UL << pin)) != 0U) {
        DEBUG_PORT->BSRR = (1UL << (pin + 16U));   /* reset */
    } else {
        DEBUG_PORT->BSRR = (1UL << pin);           /* set   */
    }
}

static inline void debug_pin_set(uint32_t pin, int high)
{
    DEBUG_PORT->BSRR = high ? (1UL << pin) : (1UL << (pin + 16U));
}

#define DEBUG_A_TOGGLE()  debug_pin_toggle(DEBUG_A_PIN)
#define DEBUG_B_TOGGLE()  debug_pin_toggle(DEBUG_B_PIN)
#define DEBUG_C_TOGGLE()  debug_pin_toggle(DEBUG_C_PIN)

#endif /* DEBUG_PINS_H */
