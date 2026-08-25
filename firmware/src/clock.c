/* ---------------------------------------------------------------------------
 * clock.c -- STM32F411CEU6 clock tree bring-up.
 *
 * Target configuration (all constraints checked against RM0383 rev 3,
 * section 6 "Reset and clock control", and the STM32F411xC/E datasheet):
 *
 *   HSE          8 MHz    (Y1 crystal, 22 pF load caps on PH0/PH1)
 *   PLL M = 4    -> VCO input   8 MHz / 4   =  2 MHz
 *                   RM0383 6.3.2: PLLM must divide the source down into the
 *                   1-2 MHz window, and 2 MHz is the recommended value to
 *                   minimise PLL jitter.
 *   PLL N = 192  -> VCO output  2 MHz * 192 = 384 MHz
 *                   RM0383 6.3.2: VCO output must be 100-432 MHz.  OK.
 *   PLL P = 4    -> SYSCLK      384 MHz / 4 =  96 MHz   (<= 100 MHz limit)
 *   PLL Q = 8    -> OTG_FS clk  384 MHz / 8 =  48 MHz   (exact; USB needs
 *                   48 MHz +/- 0.25 % and a fractional value will not
 *                   enumerate reliably)
 *
 *   AHB  prescaler /1  -> HCLK  96 MHz
 *   APB1 prescaler /2  -> PCLK1 48 MHz   (APB1 limit on F411 is 50 MHz)
 *   APB2 prescaler /1  -> PCLK2 96 MHz   (APB2 limit on F411 is 100 MHz)
 *
 *   APB1 timers (TIM2..TIM5) are clocked at 2 x PCLK1 = 96 MHz whenever the
 *   APB1 prescaler is not /1 (RM0383 6.2).  tim_us.c relies on that.
 *
 *   Voltage scaling: the F411 needs Scale 1 (PWR_CR VOS = 0b11) for SYSCLK
 *   above 84 MHz.  Reset value is Scale 2, so we must raise it -- and the
 *   PWR peripheral clock has to be enabled before PWR_CR is writable.
 *
 *   Flash latency: datasheet Table "Number of wait states according to CPU
 *   clock frequency", 2.7 V-3.6 V column (this board runs at 3.3 V):
 *       0 WS  <= 30 MHz,  1 WS <= 64 MHz,  2 WS <= 90 MHz,  3 WS <= 100 MHz
 *   96 MHz therefore needs 3 wait states.  The prefetch buffer and the
 *   instruction/data caches are enabled at the same time.
 *
 * Ordering matters: raise the flash latency BEFORE switching to the faster
 * clock, and verify the latency actually took effect before proceeding (the
 * FLASH_ACR write is posted and silently ignored if the value is illegal).
 * ------------------------------------------------------------------------- */

#include "clock.h"
#include "board.h"

uint32_t SystemCoreClock = 16000000UL;  /* HSI at reset, updated by clock_init */

/* Generous spin-timeouts, expressed in loop iterations rather than time
 * because at this point no timer is running yet.  At the 16 MHz HSI reset
 * clock each iteration is a handful of cycles, so ~2 million iterations is
 * comfortably longer than the worst-case HSE start-up time (a few ms) without
 * being long enough to look like a hang. */
#define CLOCK_SPIN_TIMEOUT  2000000UL

/* PLL field values.  Encoded per RM0383 6.3.2 (RCC_PLLCFGR). */
#define PLL_M   4UL
#define PLL_N   192UL
#define PLL_P   4UL     /* encoded as (P/2 - 1) = 0b01 in PLLP[1:0]          */
#define PLL_Q   8UL

/* ---------------------------------------------------------------------------
 * SystemInit -- called from Reset_Handler before main().
 *
 * Only does what must happen before any C code that might touch a float:
 * grant full access to coprocessors 10 and 11 (the FPU).  Without this the
 * first VFP instruction takes a UsageFault.  The rest of the clock setup runs
 * from main() via clock_init(), where it can report failure.
 * ------------------------------------------------------------------------- */
void SystemInit(void)
{
    /* CPACR: CP10 and CP11 -> full access (bits 20-23 = 0b1111). */
    SCB->CPACR |= (3UL << 20U) | (3UL << 22U);

    __DSB();
    __ISB();
}

/* Spin until (reg & mask) matches `want`, or the timeout expires.
 * Returns true on success. */
static bool wait_for_bits(volatile const uint32_t *reg, uint32_t mask, uint32_t want)
{
    uint32_t spins = CLOCK_SPIN_TIMEOUT;

    while (((*reg) & mask) != want) {
        if (spins-- == 0U) {
            return false;
        }
    }
    return true;
}

bool clock_init(void)
{
    /* ---- 1. Regulator voltage scale 1 (required above 84 MHz) ---------- */
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC->APB1ENR;                 /* read back: guarantees the clock is
                                         * really enabled before PWR is used */
    PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0 | PWR_CR_VOS_1;

    /* ---- 2. Start the 8 MHz HSE crystal oscillator --------------------- */
    RCC->CR |= RCC_CR_HSEON;
    if (!wait_for_bits(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY)) {
        return false;                   /* crystal did not start            */
    }

    /* ---- 3. Flash latency + caches, BEFORE going fast ------------------ */
    FLASH->ACR = FLASH_ACR_PRFTEN       /* prefetch buffer                  */
               | FLASH_ACR_ICEN         /* instruction cache                */
               | FLASH_ACR_DCEN         /* data cache                       */
               | FLASH_ACR_LATENCY_3WS; /* 3 wait states for 96 MHz @ 3.3 V */
    if (!wait_for_bits(&FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_3WS)) {
        return false;                   /* latency write did not stick      */
    }

    /* ---- 4. Bus prescalers ---------------------------------------------
     * Set these before the PLL becomes SYSCLK so that no bus is ever
     * momentarily overclocked during the switch. */
    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2))
              | RCC_CFGR_HPRE_DIV1      /* AHB  = SYSCLK      = 96 MHz      */
              | RCC_CFGR_PPRE1_DIV2     /* APB1 = AHB / 2     = 48 MHz      */
              | RCC_CFGR_PPRE2_DIV1;    /* APB2 = AHB / 1     = 96 MHz      */

    /* ---- 5. Configure and start the main PLL ---------------------------- */
    RCC->PLLCFGR = (PLL_M << RCC_PLLCFGR_PLLM_Pos)
                 | (PLL_N << RCC_PLLCFGR_PLLN_Pos)
                 | (((PLL_P / 2UL) - 1UL) << RCC_PLLCFGR_PLLP_Pos)
                 | (PLL_Q << RCC_PLLCFGR_PLLQ_Pos)
                 | RCC_PLLCFGR_PLLSRC_HSE;

    RCC->CR |= RCC_CR_PLLON;
    if (!wait_for_bits(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY)) {
        return false;
    }

    /* ---- 6. Switch SYSCLK to the PLL ------------------------------------ */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    if (!wait_for_bits(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL)) {
        return false;
    }

    /* ---- 7. The HSI is no longer needed --------------------------------- */
    RCC->CR &= ~RCC_CR_HSION;

    SystemCoreClock = BOARD_SYSCLK_HZ;
    return true;
}

/* CMSIS declares this in system_stm32f4xx.h; we always know our own clock, so
 * it just republishes the constant rather than reverse-engineering RCC. */
void SystemCoreClockUpdate(void);
void SystemCoreClockUpdate(void)
{
    SystemCoreClock = ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
                    ? BOARD_SYSCLK_HZ
                    : 16000000UL;
}
