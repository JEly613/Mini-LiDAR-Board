/* ---------------------------------------------------------------------------
 * usart1_dma.h -- USART1 transport for the RPLIDAR A1.
 *
 * RX is a circular DMA into a ring buffer that the CPU never has to service
 * on time: the LiDAR pushes a continuous unsolicited stream once scanning
 * starts, and there is no flow control, so any byte-at-a-time RX interrupt
 * competing with the 1 kHz IMU path would eventually drop bytes.  The main
 * loop reads out of the ring at its own pace by comparing its own tail index
 * against the DMA controller's write position (derived from NDTR).
 *
 * TX is polled.  The host only ever sends the LiDAR 2-byte commands, a
 * handful of times per session, so a DMA channel would be pure ceremony.
 * ------------------------------------------------------------------------- */

#ifndef USART1_DMA_H
#define USART1_DMA_H

#include <stdbool.h>
#include <stdint.h>

/* Sized for ~180 ms of traffic at 115200 baud, which is far more slack than
 * the main loop (which runs at hundreds of hertz) can ever need. */
#define USART1_RX_RING_BYTES  2048U

/* Configure PA9/PA10, bring up USART1 at LIDAR_UART_BAUD, and start the
 * circular RX DMA.  Safe to call more than once (used by stall recovery). */
void usart1_init(void);

/* Bytes available to read right now. */
uint16_t usart1_rx_available(void);

/* Pop up to `max` bytes into `dst`; returns how many were copied. */
uint16_t usart1_read(uint8_t *dst, uint16_t max);

/* Throw away everything currently buffered -- used after sending a command
 * that makes the device's previous output meaningless. */
void usart1_rx_flush(void);

/* Blocking send.  Bounded by a timeout so a wedged peripheral cannot hang
 * the main loop; returns false if the timeout expired. */
bool usart1_write(const uint8_t *data, uint16_t len);

/* Count of DMA overrun/framing/noise errors seen since init. */
uint32_t usart1_error_count(void);

#endif /* USART1_DMA_H */
