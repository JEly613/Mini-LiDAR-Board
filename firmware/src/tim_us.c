/* ---------------------------------------------------------------------------
 * tim_us.c -- 1 MHz free-running time base on TIM5.
 *
 * Why TIM5:
 *   - TIM2 and TIM5 are the only 32-bit timers on the STM32F411, and a 32-bit
 *     counter gives 71.6 minutes of unambiguous microsecond timestamps
 *     instead of the 65 ms a 16-bit timer would manage.
 *   - TIM2 is reserved: PB10 is TIM2_CH3 and will drive the LiDAR motor PWM
 *     in a later milestone (see board.h).  So TIM5 it is.
 *
 * Clocking: TIM5 sits on APB1.  RM0383 6.2 -- when the APB1 prescaler is not
 * /1 the timer clock is 2 x PCLK1.  We run APB1 at /2, so TIM5 is fed
 * 2 x 48 MHz = 96 MHz.  A prescaler of (96 - 1) divides that to exactly
 * 1 MHz, i.e. one tick per microsecond with no rounding error.
 *
 * The timer generates no interrupts at all -- reading CNT is a single
 * load from a peripheral register, which is safe from any context including
 * ISRs, with no critical section required.
 * ------------------------------------------------------------------------- */

#include "tim_us.h"
#include "board.h"

void tim_us_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    (void)RCC->APB1ENR;                 /* read back to guarantee the clock  */

    TIM5->CR1  = 0U;                    /* stop and reset control            */
    TIM5->PSC  = (BOARD_APB1_TIM_HZ / 1000000UL) - 1UL;  /* 96 -> 1 MHz      */
    TIM5->ARR  = 0xFFFFFFFFUL;          /* full 32-bit wrap                  */
    TIM5->CNT  = 0U;
    TIM5->DIER = 0U;                    /* no interrupts, ever               */

    /* UG forces the prescaler shadow register to load immediately; without it
     * the first update event would still be running at the reset prescaler. */
    TIM5->EGR  = TIM_EGR_UG;
    TIM5->SR   = 0U;                    /* discard the update flag UG set    */

    TIM5->CR1  = TIM_CR1_CEN;
}

uint32_t micros(void)
{
    return TIM5->CNT;
}

uint32_t millis(void)
{
    return TIM5->CNT / 1000UL;
}

void delay_us(uint32_t us)
{
    const uint32_t start = TIM5->CNT;

    /* Unsigned wrap-around subtraction: correct even when CNT rolls over. */
    while ((TIM5->CNT - start) < us) {
        /* spin */
    }
}

void delay_ms(uint32_t ms)
{
    while (ms-- > 0U) {
        delay_us(1000U);
    }
}
