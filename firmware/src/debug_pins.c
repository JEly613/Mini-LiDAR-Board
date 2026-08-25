/* ---------------------------------------------------------------------------
 * debug_pins.c -- configure the J4 scope header GPIOs.
 *
 * One subtlety: PB3 and PB4 come out of reset as JTAG pins (JTDO-TRACESWO and
 * NJTRST respectively) with MODER = alternate-function.  Driving them as
 * plain outputs, as we do here, disables the JTAG-DP.  That is harmless on
 * this board because J5 exposes SWD only (SWDIO = PA13, SWCLK = PA14), and
 * SWD keeps working after PB3/PB4 are repurposed.  It does mean SWO trace is
 * unavailable -- an acceptable trade for three observable timing pins.
 * ------------------------------------------------------------------------- */

#include "debug_pins.h"

static void configure_output(GPIO_TypeDef *port, uint32_t pin)
{
    /* MODER  = 01  general purpose output          */
    port->MODER = (port->MODER & ~(3UL << (pin * 2U))) | (1UL << (pin * 2U));
    /* OTYPER = 0   push-pull                       */
    port->OTYPER &= ~(1UL << pin);
    /* OSPEEDR= 10  high speed: these are timing
     * markers, so fast edges matter                */
    port->OSPEEDR = (port->OSPEEDR & ~(3UL << (pin * 2U))) | (2UL << (pin * 2U));
    /* PUPDR  = 00  no pull                         */
    port->PUPDR &= ~(3UL << (pin * 2U));
}

void debug_pins_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;

    configure_output(DEBUG_PORT, DEBUG_A_PIN);
    configure_output(DEBUG_PORT, DEBUG_B_PIN);
    configure_output(DEBUG_PORT, DEBUG_C_PIN);

    DEBUG_PORT->BSRR = (1UL << (DEBUG_A_PIN + 16U))
                     | (1UL << (DEBUG_B_PIN + 16U))
                     | (1UL << (DEBUG_C_PIN + 16U));
}
