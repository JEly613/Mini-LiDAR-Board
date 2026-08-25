/* ---------------------------------------------------------------------------
 * spi1_dma.h -- SPI1 master driver for the ICM-42688-P, with a DMA path for
 * the steady-state sample burst and a polled path for configuration.
 * ------------------------------------------------------------------------- */

#ifndef SPI1_DMA_H
#define SPI1_DMA_H

#include <stdbool.h>
#include <stdint.h>

/* Called from the DMA2 Stream 0 (SPI1_RX) transfer-complete ISR, after chip
 * select has been released.  Runs in interrupt context: keep it short. */
typedef void (*spi1_done_cb_t)(void);

void spi1_init(void);

/* Bus speed.  The ICM-42688-P tolerates 24 MHz max on SPI; we use a
 * conservative ~1.5 MHz while writing configuration registers (where a
 * marginal edge would silently corrupt a setting) and 12 MHz for the
 * data burst (where throughput matters and the transaction is verifiable). */
void spi1_speed_slow(void);   /* PCLK2 / 64 = 1.5 MHz */
void spi1_speed_fast(void);   /* PCLK2 /  8 =  12 MHz */

/* Polled full-duplex transfer.  Asserts and releases CS itself.  Only used
 * during initialisation and error recovery -- never in the sample path. */
void spi1_xfer_blocking(const uint8_t *tx, uint8_t *rx, uint16_t len);

/* True while a DMA transfer is in flight. */
bool spi1_dma_busy(void);

/* Start a non-blocking full-duplex DMA transfer.  `tx` and `rx` must both be
 * valid for `len` bytes and must stay alive until `done` fires.  Returns
 * false (and does nothing) if a transfer is already in flight. */
bool spi1_dma_xfer(const uint8_t *tx, uint8_t *rx, uint16_t len, spi1_done_cb_t done);

#endif /* SPI1_DMA_H */
