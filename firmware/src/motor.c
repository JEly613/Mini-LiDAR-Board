/* ---------------------------------------------------------------------------
 * motor.c -- see motor.h.
 *
 * Timer clock arithmetic:
 *   TIM2 sits on APB1.  APB1's prescaler is /2, and the STM32 doubles the
 *   timer clock whenever the APB prescaler is not /1, so TIM2 counts at
 *   2 x 48 MHz = 96 MHz (RM0383 6.2, "Clock tree").
 *
 *   PSC = 0, ARR = 4799  ->  96 MHz / 4800 = 20 kHz PWM
 *
 *   4800 counts of resolution across a 0-1000 permille control range is far
 *   finer than the spindle can actually respond to, so the duty arithmetic
 *   below never needs to worry about rounding.
 * ------------------------------------------------------------------------- */

#include "motor.h"
#include "board.h"

#define MOTOR_PWM_ARR   4799UL          /* 20 kHz from a 96 MHz timer clock  */

static uint16_t s_permille;

void motor_init(void)
{
    /* ---- GPIO: PB10 -> AF1 (TIM2_CH3), push-pull -------------------------
     * Deliberately left at LOW speed: this is a 20 kHz signal into a motor
     * driver, and a slow edge radiates far less into the SPI and UART traces
     * running nearby.  The project's layout notes call this out explicitly. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    const uint32_t pin = LIDAR_PWM_PIN;

    LIDAR_PWM_PORT->MODER   = (LIDAR_PWM_PORT->MODER & ~(3UL << (pin * 2U)))
                            | (2UL << (pin * 2U));          /* alternate fn  */
    LIDAR_PWM_PORT->OTYPER &= ~(1UL << pin);                /* push-pull     */
    LIDAR_PWM_PORT->OSPEEDR &= ~(3UL << (pin * 2U));        /* low speed     */
    LIDAR_PWM_PORT->PUPDR  &= ~(3UL << (pin * 2U));         /* no pull       */
    LIDAR_PWM_PORT->AFR[1]  = (LIDAR_PWM_PORT->AFR[1] & ~(0xFUL << ((pin - 8U) * 4U)))
                            | (1UL << ((pin - 8U) * 4U));   /* AF1 = TIM2    */

    /* ---- Timer ---------------------------------------------------------- */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1  = 0U;
    TIM2->PSC  = 0U;
    TIM2->ARR  = MOTOR_PWM_ARR;
    TIM2->CCR3 = 0U;                    /* start stopped                     */

    /* Channel 3: PWM mode 1, preloaded compare register so a duty change
     * takes effect at the next update event instead of mid-pulse. */
    TIM2->CCMR2 = (6UL << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
    TIM2->CCER  = TIM_CCER_CC3E;

    TIM2->EGR   = TIM_EGR_UG;           /* latch PSC/ARR immediately         */
    TIM2->CR1   = TIM_CR1_ARPE | TIM_CR1_CEN;

    s_permille = 0U;
}

void motor_set_permille(uint16_t permille)
{
    if (permille > MOTOR_PERMILLE_MAX) {
        permille = MOTOR_PERMILLE_MAX;
    }
    s_permille = permille;

    /* (ARR + 1) * permille / 1000, in 32-bit arithmetic so the multiply
     * cannot overflow: 4800 * 1000 fits comfortably. */
    TIM2->CCR3 = ((MOTOR_PWM_ARR + 1UL) * (uint32_t)permille) / 1000UL;
}

uint16_t motor_get_permille(void)
{
    return s_permille;
}
