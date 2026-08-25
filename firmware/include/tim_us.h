/* ---------------------------------------------------------------------------
 * tim_us.h -- free-running microsecond time base.
 * ------------------------------------------------------------------------- */

#ifndef TIM_US_H
#define TIM_US_H

#include <stdint.h>

/* Start TIM5 as a 32-bit free-running 1 MHz up-counter. */
void tim_us_init(void);

/* Microseconds since tim_us_init().  Wraps every 2^32 us = 4295 s (71.6 min).
 * All timestamp arithmetic in this firmware uses unsigned subtraction, which
 * stays correct across exactly one wrap; the host is told about the wrap in
 * PROTOCOL.md so it can unwrap the timeline itself. */
uint32_t micros(void);

/* Milliseconds since tim_us_init(); convenience wrapper, same wrap caveat
 * scaled by 1000 (wraps every ~49.7 days). */
uint32_t millis(void);

/* Busy-wait delays.  Only for start-up sequencing (crystal settling, IMU
 * reset timing); never called from the steady-state sample path. */
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#endif /* TIM_US_H */
