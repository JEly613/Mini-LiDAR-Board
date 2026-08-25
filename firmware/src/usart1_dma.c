/* ---------------------------------------------------------------------------
 * usart1_dma.c -- see usart1_dma.h.
 *
 * DMA choice: USART1_RX can only be mapped to DMA2 Stream 2 channel 4 or
 * DMA2 Stream 5 channel 4 (RM0383 Table 28).  spi1_dma.c deliberately avoided
 * both by taking Streams 0 and 3, so either is free; we take Stream 5.
 *
 * There is no transfer-complete interrupt and no half-transfer interrupt: the
 * stream runs forever in circular mode and the main loop reads whatever has
 * landed.  That is the entire point -- the CPU has no real-time obligation to
 * the LiDAR at all.
 * ------------------------------------------------------------------------- */

#include "usart1_dma.h"
#include "board.h"
#include "tim_us.h"

#define U1_RX_STREAM   DMA2_Stream5
#define U1_RX_CHANNEL  4UL

static uint8_t  s_rx[USART1_RX_RING_BYTES];
static uint16_t s_tail;                  /* our read position in s_rx        */
static uint32_t s_errors;

/* The DMA controller counts NDTR *down* from the buffer size, so the number
 * of bytes it has written is (size - NDTR). */
static inline uint16_t dma_head(void)
{
    return (uint16_t)(USART1_RX_RING_BYTES - (uint16_t)U1_RX_STREAM->NDTR);
}

static void gpio_af7(uint32_t pin)
{
    GPIOA->MODER   = (GPIOA->MODER & ~(3UL << (pin * 2U))) | (2UL << (pin * 2U));
    GPIOA->OTYPER &= ~(1UL << pin);
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3UL << (pin * 2U))) | (2UL << (pin * 2U));
    /* Pull-up on both lines: an unplugged LiDAR then idles high (the UART
     * marking state) instead of floating and generating framing errors. */
    GPIOA->PUPDR   = (GPIOA->PUPDR & ~(3UL << (pin * 2U))) | (1UL << (pin * 2U));

    if (pin < 8U) {
        GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFUL << (pin * 4U))) | (7UL << (pin * 4U));
    } else {
        GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFUL << ((pin - 8U) * 4U)))
                      | (7UL << ((pin - 8U) * 4U));
    }
}

void usart1_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    gpio_af7(LIDAR_UART_TX_PIN);
    gpio_af7(LIDAR_UART_RX_PIN);

    /* ---- Stop everything before reconfiguring (this is also the recovery
     * path, so it must tolerate a running stream). ------------------------ */
    USART1->CR1 = 0U;
    U1_RX_STREAM->CR &= ~DMA_SxCR_EN;
    while ((U1_RX_STREAM->CR & DMA_SxCR_EN) != 0U) {
        /* the controller finishes the current beat before clearing EN */
    }
    /* Clear all six event flags for stream 5 in HIFCR (bits 6..11). */
    DMA2->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5
                | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;

    /* ---- USART1: 115200 8N1 ---------------------------------------------
     * USART1 is on APB2 at 96 MHz.  With OVER8 = 0 the BRR register holds
     * fCK/baud directly, in units of 1/16 -- so a plain rounded division is
     * the correct value, no mantissa/fraction assembly needed.
     *   96e6 / 115200 = 833.33 -> 833 = 0x341, which yields 115246 baud,
     *   an error of 0.04 %.  (Tolerance for 8N1 is about 2 %.)              */
    USART1->BRR = (BOARD_APB2_HZ + (LIDAR_UART_BAUD / 2UL)) / LIDAR_UART_BAUD;
    USART1->CR2 = 0U;                            /* 1 stop bit               */
    USART1->CR3 = USART_CR3_DMAR;                /* RX via DMA               */

    /* ---- DMA2 Stream 5 channel 4: peripheral -> memory, circular -------- */
    U1_RX_STREAM->PAR  = (uint32_t)&USART1->DR;
    U1_RX_STREAM->M0AR = (uint32_t)s_rx;
    U1_RX_STREAM->NDTR = USART1_RX_RING_BYTES;
    U1_RX_STREAM->FCR  = 0U;                     /* direct mode              */
    U1_RX_STREAM->CR   = (U1_RX_CHANNEL << DMA_SxCR_CHSEL_Pos)
                       | DMA_SxCR_MINC              /* advance through s_rx  */
                       | DMA_SxCR_CIRC              /* wrap forever          */
                       | (1UL << DMA_SxCR_PL_Pos);  /* medium priority       */
                       /* DIR = 00 (peripheral -> memory), sizes = 8 bit     */

    s_tail   = 0U;
    s_errors = 0U;

    U1_RX_STREAM->CR |= DMA_SxCR_EN;

    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

uint16_t usart1_rx_available(void)
{
    const uint16_t head = dma_head();

    if (head >= s_tail) {
        return (uint16_t)(head - s_tail);
    }
    return (uint16_t)((uint32_t)USART1_RX_RING_BYTES - (uint32_t)s_tail + (uint32_t)head);
}

uint16_t usart1_read(uint8_t *dst, uint16_t max)
{
    uint16_t n = usart1_rx_available();

    if (n > max) {
        n = max;
    }

    for (uint16_t i = 0U; i < n; i++) {
        dst[i] = s_rx[s_tail];
        s_tail = (uint16_t)((s_tail + 1U) % USART1_RX_RING_BYTES);
    }

    /* An overrun sets ORE and stalls reception until the flag is cleared.
     * With DMA servicing the data register this should never fire, but if it
     * does we clear it (read SR then DR) and count it rather than going deaf. */
    if ((USART1->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0U) {
        (void)USART1->SR;
        (void)USART1->DR;
        s_errors++;
    }
    return n;
}

void usart1_rx_flush(void)
{
    s_tail = dma_head();
}

bool usart1_write(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0U; i < len; i++) {
        const uint32_t started = micros();

        while ((USART1->SR & USART_SR_TXE) == 0U) {
            if ((micros() - started) > 10000UL) {   /* 10 ms per byte is an age */
                return false;
            }
        }
        USART1->DR = data[i];
    }

    const uint32_t started = micros();

    while ((USART1->SR & USART_SR_TC) == 0U) {
        if ((micros() - started) > 10000UL) {
            return false;
        }
    }
    return true;
}

uint32_t usart1_error_count(void)
{
    return s_errors;
}
