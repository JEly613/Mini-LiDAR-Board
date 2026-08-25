/* ---------------------------------------------------------------------------
 * icm42688.h -- driver for the on-board TDK InvenSense ICM-42688-P (U2).
 *
 * Configuration produced by icm_init():
 *   gyroscope      +/- 500 dps, 1 kHz ODR, low-noise mode
 *   accelerometer  +/-   8 g,   1 kHz ODR, low-noise mode
 *   temperature    enabled
 *   data-ready interrupt on INT1 -> PB0 -> EXTI0
 *
 * Data flow, once running, touches the CPU for only a few microseconds per
 * sample:
 *
 *   INT1 rising edge
 *     -> EXTI0_IRQHandler: latch micros(), kick a 15-byte SPI DMA burst
 *        -> DMA2_Stream0 TC IRQ: decode, scale, push into a lock-free queue
 *           -> main loop pops it and runs the Madgwick update
 * ------------------------------------------------------------------------- */

#ifndef ICM42688_H
#define ICM42688_H

#include <stdbool.h>
#include <stdint.h>

/* Value the WHO_AM_I register (0x75) must return for an ICM-42688-P. */
#define ICM_WHO_AM_I_EXPECTED  0x47U

/* Configured full-scale ranges and rate, mirrored into the status packet. */
#define ICM_ODR_HZ             1000U
#define ICM_GYRO_FS_DPS        500
#define ICM_ACCEL_FS_G         8

typedef enum {
    ICM_OK = 0,
    ICM_ERR_WHO_AM_I,     /* device answered, but not with 0x47             */
    ICM_ERR_NO_RESPONSE   /* bus reads returned only 0x00 or 0xFF           */
} icm_status_t;

/* One scaled sample. */
typedef struct {
    uint32_t t_us;     /* micros() latched in the data-ready ISR            */
    float    ax, ay, az;  /* g,   sensor frame                              */
    float    gx, gy, gz;  /* dps, sensor frame, no bias correction applied  */
    float    temp_c;      /* die temperature, degrees Celsius               */
} icm_sample_t;

/* Reset and configure the device.  Leaves SPI1 at its fast clock and the
 * data-ready interrupt enabled on success.  `who_am_i_out` (optional)
 * receives the raw byte actually read, so the failure can be reported to the
 * host instead of just being a return code. */
icm_status_t icm_init(uint8_t *who_am_i_out);

/* Pop the oldest queued sample.  Returns false if the queue is empty.
 * Safe to call from the main loop while the ISRs keep producing. */
bool icm_pop_sample(icm_sample_t *out);

/* Total samples successfully read since boot. */
uint32_t icm_sample_count(void);

/* Times a data-ready arrived while the previous DMA burst was still running
 * (SPI overrun), plus times the decoded-sample queue was full. */
uint32_t icm_overrun_count(void);

/* micros() of the most recent data-ready edge; used to detect a stalled IMU. */
uint32_t icm_last_drdy_us(void);

/* Disable/enable the INT1 EXTI line -- used around re-initialisation. */
void icm_irq_disable(void);
void icm_irq_enable(void);

#endif /* ICM42688_H */
