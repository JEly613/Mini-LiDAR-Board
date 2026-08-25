/* ---------------------------------------------------------------------------
 * startup_stm32f411xe.c -- vector table and reset entry point.
 *
 * Written in C rather than assembly so the whole boot path is readable in one
 * language.  The only assumptions are the ones the ARMv7-M architecture
 * guarantees: on reset the core loads the initial MSP from address 0x00000000
 * and the reset vector from 0x00000004, both of which live at the start of
 * the .isr_vector section that the linker script places first in flash.
 *
 * Every interrupt in the STM32F411 vector table is listed explicitly, aliased
 * to Default_Handler unless a strong definition exists elsewhere.  Reserved
 * slots are zero, exactly as in ST's table, so the numbering stays honest.
 * ------------------------------------------------------------------------- */

#include <stdint.h>

/* Symbols provided by the linker script. */
extern uint32_t _sidata;   /* start of the .data initialiser image in flash   */
extern uint32_t _sdata;    /* start of .data in RAM                           */
extern uint32_t _edata;    /* end   of .data in RAM                           */
extern uint32_t _sbss;     /* start of .bss  in RAM                           */
extern uint32_t _ebss;     /* end   of .bss  in RAM                           */
extern uint32_t _estack;   /* top of stack (one past the end of SRAM)         */

extern int  main(void);
extern void SystemInit(void);

void Reset_Handler(void);
void Default_Handler(void);

/* Every handler is weakly aliased to Default_Handler.  Any translation unit
 * that defines one of these names with external linkage overrides the alias
 * at link time -- that is how, e.g., spi1_dma.c claims DMA2_Stream0_IRQHandler. */
#define WEAK_ALIAS  __attribute__((weak, alias("Default_Handler")))

/* Cortex-M4 system exceptions. */
void NMI_Handler(void)              WEAK_ALIAS;
void HardFault_Handler(void)        WEAK_ALIAS;
void MemManage_Handler(void)        WEAK_ALIAS;
void BusFault_Handler(void)         WEAK_ALIAS;
void UsageFault_Handler(void)       WEAK_ALIAS;
void SVC_Handler(void)              WEAK_ALIAS;
void DebugMon_Handler(void)         WEAK_ALIAS;
void PendSV_Handler(void)           WEAK_ALIAS;
void SysTick_Handler(void)          WEAK_ALIAS;

/* STM32F411 peripheral interrupts. */
void WWDG_IRQHandler(void)               WEAK_ALIAS;
void PVD_IRQHandler(void)                WEAK_ALIAS;
void TAMP_STAMP_IRQHandler(void)         WEAK_ALIAS;
void RTC_WKUP_IRQHandler(void)           WEAK_ALIAS;
void FLASH_IRQHandler(void)              WEAK_ALIAS;
void RCC_IRQHandler(void)                WEAK_ALIAS;
void EXTI0_IRQHandler(void)              WEAK_ALIAS;
void EXTI1_IRQHandler(void)              WEAK_ALIAS;
void EXTI2_IRQHandler(void)              WEAK_ALIAS;
void EXTI3_IRQHandler(void)              WEAK_ALIAS;
void EXTI4_IRQHandler(void)              WEAK_ALIAS;
void DMA1_Stream0_IRQHandler(void)       WEAK_ALIAS;
void DMA1_Stream1_IRQHandler(void)       WEAK_ALIAS;
void DMA1_Stream2_IRQHandler(void)       WEAK_ALIAS;
void DMA1_Stream3_IRQHandler(void)       WEAK_ALIAS;
void DMA1_Stream4_IRQHandler(void)       WEAK_ALIAS;
void DMA1_Stream5_IRQHandler(void)       WEAK_ALIAS;
void DMA1_Stream6_IRQHandler(void)       WEAK_ALIAS;
void ADC_IRQHandler(void)                WEAK_ALIAS;
void EXTI9_5_IRQHandler(void)            WEAK_ALIAS;
void TIM1_BRK_TIM9_IRQHandler(void)      WEAK_ALIAS;
void TIM1_UP_TIM10_IRQHandler(void)      WEAK_ALIAS;
void TIM1_TRG_COM_TIM11_IRQHandler(void) WEAK_ALIAS;
void TIM1_CC_IRQHandler(void)            WEAK_ALIAS;
void TIM2_IRQHandler(void)               WEAK_ALIAS;
void TIM3_IRQHandler(void)               WEAK_ALIAS;
void TIM4_IRQHandler(void)               WEAK_ALIAS;
void I2C1_EV_IRQHandler(void)            WEAK_ALIAS;
void I2C1_ER_IRQHandler(void)            WEAK_ALIAS;
void I2C2_EV_IRQHandler(void)            WEAK_ALIAS;
void I2C2_ER_IRQHandler(void)            WEAK_ALIAS;
void SPI1_IRQHandler(void)               WEAK_ALIAS;
void SPI2_IRQHandler(void)               WEAK_ALIAS;
void USART1_IRQHandler(void)             WEAK_ALIAS;
void USART2_IRQHandler(void)             WEAK_ALIAS;
void EXTI15_10_IRQHandler(void)          WEAK_ALIAS;
void RTC_Alarm_IRQHandler(void)          WEAK_ALIAS;
void OTG_FS_WKUP_IRQHandler(void)        WEAK_ALIAS;
void DMA1_Stream7_IRQHandler(void)       WEAK_ALIAS;
void SDIO_IRQHandler(void)               WEAK_ALIAS;
void TIM5_IRQHandler(void)               WEAK_ALIAS;
void SPI3_IRQHandler(void)               WEAK_ALIAS;
void DMA2_Stream0_IRQHandler(void)       WEAK_ALIAS;
void DMA2_Stream1_IRQHandler(void)       WEAK_ALIAS;
void DMA2_Stream2_IRQHandler(void)       WEAK_ALIAS;
void DMA2_Stream3_IRQHandler(void)       WEAK_ALIAS;
void DMA2_Stream4_IRQHandler(void)       WEAK_ALIAS;
void OTG_FS_IRQHandler(void)             WEAK_ALIAS;
void DMA2_Stream5_IRQHandler(void)       WEAK_ALIAS;
void DMA2_Stream6_IRQHandler(void)       WEAK_ALIAS;
void DMA2_Stream7_IRQHandler(void)       WEAK_ALIAS;
void USART6_IRQHandler(void)             WEAK_ALIAS;
void I2C3_EV_IRQHandler(void)            WEAK_ALIAS;
void I2C3_ER_IRQHandler(void)            WEAK_ALIAS;
void FPU_IRQHandler(void)                WEAK_ALIAS;
void SPI4_IRQHandler(void)               WEAK_ALIAS;
void SPI5_IRQHandler(void)               WEAK_ALIAS;

typedef void (*vector_entry_t)(void);

/* The table is 16 system entries followed by 86 device entries (0..85). */
__attribute__((section(".isr_vector"), used))
const vector_entry_t g_vector_table[16 + 86] = {
    /* ---- Cortex-M4 system vectors ------------------------------------- */
    (vector_entry_t)(&_estack),      /*   0  initial MSP                   */
    Reset_Handler,                   /*   1  reset                         */
    NMI_Handler,                     /*   2                                */
    HardFault_Handler,               /*   3                                */
    MemManage_Handler,               /*   4                                */
    BusFault_Handler,                /*   5                                */
    UsageFault_Handler,              /*   6                                */
    0, 0, 0, 0,                      /* 7-10 reserved                      */
    SVC_Handler,                     /*  11                                */
    DebugMon_Handler,                /*  12                                */
    0,                               /*  13  reserved                      */
    PendSV_Handler,                  /*  14                                */
    SysTick_Handler,                 /*  15                                */

    /* ---- STM32F411 external interrupts, IRQ 0 upwards ------------------ */
    WWDG_IRQHandler,                 /*   0                                */
    PVD_IRQHandler,                  /*   1                                */
    TAMP_STAMP_IRQHandler,           /*   2                                */
    RTC_WKUP_IRQHandler,             /*   3                                */
    FLASH_IRQHandler,                /*   4                                */
    RCC_IRQHandler,                  /*   5                                */
    EXTI0_IRQHandler,                /*   6  <- ICM-42688-P INT1 on PB0    */
    EXTI1_IRQHandler,                /*   7                                */
    EXTI2_IRQHandler,                /*   8                                */
    EXTI3_IRQHandler,                /*   9                                */
    EXTI4_IRQHandler,                /*  10                                */
    DMA1_Stream0_IRQHandler,         /*  11                                */
    DMA1_Stream1_IRQHandler,         /*  12                                */
    DMA1_Stream2_IRQHandler,         /*  13                                */
    DMA1_Stream3_IRQHandler,         /*  14                                */
    DMA1_Stream4_IRQHandler,         /*  15                                */
    DMA1_Stream5_IRQHandler,         /*  16                                */
    DMA1_Stream6_IRQHandler,         /*  17                                */
    ADC_IRQHandler,                  /*  18                                */
    0, 0, 0, 0,                      /* 19-22 reserved (CAN, absent on F411)*/
    EXTI9_5_IRQHandler,              /*  23                                */
    TIM1_BRK_TIM9_IRQHandler,        /*  24                                */
    TIM1_UP_TIM10_IRQHandler,        /*  25                                */
    TIM1_TRG_COM_TIM11_IRQHandler,   /*  26                                */
    TIM1_CC_IRQHandler,              /*  27                                */
    TIM2_IRQHandler,                 /*  28                                */
    TIM3_IRQHandler,                 /*  29                                */
    TIM4_IRQHandler,                 /*  30                                */
    I2C1_EV_IRQHandler,              /*  31                                */
    I2C1_ER_IRQHandler,              /*  32                                */
    I2C2_EV_IRQHandler,              /*  33                                */
    I2C2_ER_IRQHandler,              /*  34                                */
    SPI1_IRQHandler,                 /*  35                                */
    SPI2_IRQHandler,                 /*  36                                */
    USART1_IRQHandler,               /*  37                                */
    USART2_IRQHandler,               /*  38                                */
    0,                               /*  39  reserved (USART3)             */
    EXTI15_10_IRQHandler,            /*  40                                */
    RTC_Alarm_IRQHandler,            /*  41                                */
    OTG_FS_WKUP_IRQHandler,          /*  42                                */
    0, 0, 0, 0,                      /* 43-46 reserved                     */
    DMA1_Stream7_IRQHandler,         /*  47                                */
    0,                               /*  48  reserved (FSMC)               */
    SDIO_IRQHandler,                 /*  49                                */
    TIM5_IRQHandler,                 /*  50  <- free-running microsecond   */
    SPI3_IRQHandler,                 /*  51                                */
    0, 0, 0, 0,                      /* 52-55 reserved                     */
    DMA2_Stream0_IRQHandler,         /*  56  <- SPI1_RX DMA complete       */
    DMA2_Stream1_IRQHandler,         /*  57                                */
    DMA2_Stream2_IRQHandler,         /*  58  (kept free for USART1_RX)     */
    DMA2_Stream3_IRQHandler,         /*  59  <- SPI1_TX DMA                */
    DMA2_Stream4_IRQHandler,         /*  60                                */
    0, 0, 0, 0, 0, 0,                /* 61-66 reserved                     */
    OTG_FS_IRQHandler,               /*  67  <- USB device stack           */
    DMA2_Stream5_IRQHandler,         /*  68  (kept free for USART1_RX)     */
    DMA2_Stream6_IRQHandler,         /*  69                                */
    DMA2_Stream7_IRQHandler,         /*  70                                */
    USART6_IRQHandler,               /*  71                                */
    I2C3_EV_IRQHandler,              /*  72                                */
    I2C3_ER_IRQHandler,              /*  73                                */
    0, 0, 0, 0, 0, 0, 0,             /* 74-80 reserved                     */
    FPU_IRQHandler,                  /*  81                                */
    0, 0,                            /* 82-83 reserved                     */
    SPI4_IRQHandler,                 /*  84                                */
    SPI5_IRQHandler                  /*  85                                */
};

/* ---------------------------------------------------------------------------
 * Reset_Handler
 *
 * Runs with the MSP already loaded from vector slot 0.  We must not touch any
 * initialised global before .data has been copied, so this function keeps
 * everything in locals.
 * ------------------------------------------------------------------------- */
void Reset_Handler(void)
{
    uint32_t       *dst;
    const uint32_t *src;

    /* Copy .data from its flash image into SRAM. */
    src = &_sidata;
    for (dst = &_sdata; dst < &_edata; dst++) {
        *dst = *src++;
    }

    /* Zero .bss. */
    for (dst = &_sbss; dst < &_ebss; dst++) {
        *dst = 0U;
    }

    /* Enable the FPU and any other pre-main core setup. */
    SystemInit();

    (void)main();

    /* main() is not expected to return.  If it somehow does, stop here rather
     * than falling off the end of flash -- the IWDG will reset us shortly. */
    for (;;) {
        __asm__ volatile("wfi");
    }
}

/* ---------------------------------------------------------------------------
 * Default_Handler -- catch-all for any interrupt we did not implement.
 *
 * There are no status LEDs on this board, so the only thing a spin loop buys
 * us is a stable state for a debugger to inspect.  The independent watchdog
 * (see iwdg_init()) will reset the board out of here within ~500 ms in the
 * field, which is the desired behaviour for an unexpected interrupt.
 * ------------------------------------------------------------------------- */
void Default_Handler(void)
{
    for (;;) {
        /* Breakpoint target: halt here and read IPSR to identify the vector. */
    }
}
