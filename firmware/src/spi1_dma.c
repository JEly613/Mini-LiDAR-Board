/* ---------------------------------------------------------------------------
 * spi1_dma.c -- SPI1 <-> ICM-42688-P transport.
 *
 * Pins (from the KiCad netlist):
 *   PA4  IMU_CS    software-driven GPIO output (NOT hardware NSS)
 *   PA5  SPI1_SCK  AF5
 *   PA6  SPI1_MISO AF5
 *   PA7  SPI1_MOSI AF5
 *
 * Mode: CPOL = 0, CPHA = 0 (SPI mode 0), MSB first, 8-bit frames.  The
 * ICM-42688-P accepts mode 0 and mode 3; mode 0 is the datasheet default with
 * DEVICE_CONFIG.SPI_MODE left at 0, so we leave that register alone.
 *
 * ---------------------------------------------------------------------------
 * DMA STREAM CHOICE -- THIS IS DELIBERATE, DO NOT "SIMPLIFY" IT
 * ---------------------------------------------------------------------------
 * On the STM32F411 (RM0383 Table 28, DMA2 request mapping) the options are:
 *
 *   SPI1_RX   : DMA2 Stream 0 ch 3   or   DMA2 Stream 2 ch 3
 *   SPI1_TX   : DMA2 Stream 3 ch 3   or   DMA2 Stream 5 ch 3
 *   USART1_RX : DMA2 Stream 2 ch 4   or   DMA2 Stream 5 ch 4
 *
 * The LiDAR milestone will need USART1_RX on DMA (an RPLIDAR pushes a
 * continuous scan stream that cannot be serviced byte-by-byte alongside a
 * 1 kHz IMU).  Its only two homes are Stream 2 and Stream 5.  So this driver
 * takes Stream 0 for RX and Stream 3 for TX, leaving BOTH USART1_RX options
 * free.  Picking Stream 2 or Stream 5 here would work fine today and then
 * force a painful rework later.
 *
 * No LiDAR code exists in this milestone -- this is purely a resource
 * reservation.
 * ---------------------------------------------------------------------------
 *
 * The steady-state path is: EXTI0 (IMU data-ready) -> spi1_dma_xfer() ->
 * DMA2_Stream0 transfer-complete ISR -> callback.  The CPU never polls a
 * status flag while data is moving.
 * ------------------------------------------------------------------------- */

#include "spi1_dma.h"
#include "board.h"

#define SPI1_RX_STREAM  DMA2_Stream0
#define SPI1_TX_STREAM  DMA2_Stream3
#define SPI1_DMA_CHANNEL  3UL           /* channel 3 for both, see table above */

/* Bit patterns for SPI1_CR1.BR[5:3]: fPCLK2 / 2^(BR+1). */
#define SPI_BR_DIV8   (2UL << SPI_CR1_BR_Pos)   /* 96 MHz /  8 = 12   MHz */
#define SPI_BR_DIV64  (5UL << SPI_CR1_BR_Pos)   /* 96 MHz / 64 =  1.5 MHz */

static volatile bool     s_dma_busy;
static spi1_done_cb_t    s_done_cb;

/* --------------------------------------------------------------------------
 * Chip select.  PA4 is a plain GPIO: the ICM-42688-P needs CS held low across
 * a whole multi-byte transaction, which hardware NSS in master mode does not
 * do on this part family.
 * ------------------------------------------------------------------------ */
static inline void cs_assert(void)   { IMU_CS_PORT->BSRR = (1UL << (IMU_CS_PIN + 16U)); }
static inline void cs_release(void)  { IMU_CS_PORT->BSRR = (1UL << IMU_CS_PIN); }

static void gpio_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af)
{
    port->MODER   = (port->MODER   & ~(3UL << (pin * 2U))) | (2UL << (pin * 2U));
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR = (port->OSPEEDR & ~(3UL << (pin * 2U))) | (3UL << (pin * 2U));
    port->PUPDR  &= ~(3UL << (pin * 2U));
    if (pin < 8U) {
        port->AFR[0] = (port->AFR[0] & ~(0xFUL << (pin * 4U))) | (af << (pin * 4U));
    } else {
        port->AFR[1] = (port->AFR[1] & ~(0xFUL << ((pin - 8U) * 4U)))
                     | (af << ((pin - 8U) * 4U));
    }
}

void spi1_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR;

    /* CS: push-pull output, idle high, configured BEFORE the SPI pins so the
     * IMU never sees a clock edge with CS accidentally low. */
    cs_release();
    IMU_CS_PORT->MODER = (IMU_CS_PORT->MODER & ~(3UL << (IMU_CS_PIN * 2U)))
                       | (1UL << (IMU_CS_PIN * 2U));
    IMU_CS_PORT->OTYPER  &= ~(1UL << IMU_CS_PIN);
    IMU_CS_PORT->OSPEEDR |= (3UL << (IMU_CS_PIN * 2U));

    gpio_af(GPIOA, IMU_SCK_PIN,  5UL);
    gpio_af(GPIOA, IMU_MISO_PIN, 5UL);
    gpio_af(GPIOA, IMU_MOSI_PIN, 5UL);

    /* SPI1: master, software NSS (SSM + SSI, otherwise a low NSS input would
     * drop us out of master mode), 8-bit, MSB first, mode 0, start slow. */
    SPI1->CR1 = 0U;
    SPI1->CR2 = 0U;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_BR_DIV64;
    SPI1->CR1 |= SPI_CR1_SPE;

    /* The RX DMA stream owns the transfer-complete interrupt.  Priority 1
     * matches EXTI0 so the IMU path cannot be preempted by USB (priority 3). */
    NVIC_SetPriority(DMA2_Stream0_IRQn, 1U);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    s_dma_busy = false;
    s_done_cb  = 0;
}

static void set_baud(uint32_t br_bits)
{
    /* BR[5:3] may only be changed with the peripheral disabled. */
    SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1  = (SPI1->CR1 & ~SPI_CR1_BR) | br_bits;
    SPI1->CR1 |= SPI_CR1_SPE;
}

void spi1_speed_slow(void) { set_baud(SPI_BR_DIV64); }
void spi1_speed_fast(void) { set_baud(SPI_BR_DIV8);  }

/* Discard anything left in the receive register, so a new transfer starts
 * from a known state even after an aborted one. */
static void drain_rx(void)
{
    while ((SPI1->SR & SPI_SR_RXNE) != 0U) {
        (void)SPI1->DR;
    }
    (void)SPI1->SR;
}

void spi1_xfer_blocking(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    drain_rx();
    cs_assert();

    for (uint16_t i = 0U; i < len; i++) {
        while ((SPI1->SR & SPI_SR_TXE) == 0U) {
            /* wait for the transmit register to accept a byte */
        }
        *(volatile uint8_t *)&SPI1->DR = tx[i];

        while ((SPI1->SR & SPI_SR_RXNE) == 0U) {
            /* full duplex: every byte out produces exactly one byte in */
        }
        rx[i] = *(volatile uint8_t *)&SPI1->DR;
    }

    /* RXNE for the final byte implies the frame finished, but BSY can lag by
     * the last clock edge; releasing CS early would truncate it. */
    while ((SPI1->SR & SPI_SR_BSY) != 0U) {
        /* wait */
    }
    cs_release();
}

bool spi1_dma_busy(void)
{
    return s_dma_busy;
}

bool spi1_dma_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len, spi1_done_cb_t done)
{
    if (s_dma_busy || len == 0U) {
        return false;
    }
    s_dma_busy = true;
    s_done_cb  = done;

    SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    drain_rx();

    /* Clear every stale interrupt flag for both streams.  Stream 0 lives in
     * the low register (bits 0-5), stream 3 in the low register's upper half
     * (bits 22-27); LIFCR is write-1-to-clear. */
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0
                | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0
                | DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3
                | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;

    /* ---- Receive stream: peripheral -> memory ------------------------- */
    SPI1_RX_STREAM->CR   = 0U;
    while ((SPI1_RX_STREAM->CR & DMA_SxCR_EN) != 0U) {
        /* a stream must read back disabled before it can be reprogrammed */
    }
    SPI1_RX_STREAM->PAR  = (uint32_t)&SPI1->DR;
    SPI1_RX_STREAM->M0AR = (uint32_t)rx;
    SPI1_RX_STREAM->NDTR = len;
    SPI1_RX_STREAM->FCR  = 0U;                       /* direct mode, no FIFO */
    SPI1_RX_STREAM->CR   = (SPI1_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos)
                         | (3UL << DMA_SxCR_PL_Pos)  /* very high priority   */
                         | DMA_SxCR_MINC             /* walk the rx buffer   */
                         | DMA_SxCR_TCIE;            /* dir = 00, P -> M     */

    /* ---- Transmit stream: memory -> peripheral ------------------------ */
    SPI1_TX_STREAM->CR   = 0U;
    while ((SPI1_TX_STREAM->CR & DMA_SxCR_EN) != 0U) {
        /* wait for disable to take effect */
    }
    SPI1_TX_STREAM->PAR  = (uint32_t)&SPI1->DR;
    SPI1_TX_STREAM->M0AR = (uint32_t)tx;
    SPI1_TX_STREAM->NDTR = len;
    SPI1_TX_STREAM->FCR  = 0U;
    SPI1_TX_STREAM->CR   = (SPI1_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos)
                         | (2UL << DMA_SxCR_PL_Pos)  /* high priority        */
                         | DMA_SxCR_MINC
                         | DMA_SxCR_DIR_0;           /* dir = 01, M -> P     */

    cs_assert();

    /* Arm RX before TX: the first MOSI byte immediately produces a MISO byte,
     * and if the receive stream were not already listening we would take an
     * overrun and lose it. */
    SPI1_RX_STREAM->CR |= DMA_SxCR_EN;
    SPI1_TX_STREAM->CR |= DMA_SxCR_EN;

    SPI1->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
    return true;
}

/* ---------------------------------------------------------------------------
 * DMA2 Stream 0 (SPI1_RX) transfer complete.
 *
 * RX completion is the authoritative "transfer finished" event: in full
 * duplex the last byte received is by definition after the last byte sent.
 * ------------------------------------------------------------------------- */
void DMA2_Stream0_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void)
{
    if ((DMA2->LISR & DMA_LISR_TCIF0) == 0U) {
        /* Not our event (transfer error / FIFO error): clear everything and
         * fail the transfer rather than leaving the driver wedged. */
        DMA2->LIFCR = DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
    }
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;

    SPI1_RX_STREAM->CR = 0U;
    SPI1_TX_STREAM->CR = 0U;
    SPI1->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

    while ((SPI1->SR & SPI_SR_BSY) != 0U) {
        /* a few clocks at most; guarantees CS does not cut the last frame */
    }
    cs_release();

    s_dma_busy = false;
    if (s_done_cb != 0) {
        s_done_cb();
    }
}
