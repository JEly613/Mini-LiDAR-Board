/* ---------------------------------------------------------------------------
 * motor.h -- RPLIDAR A1 spindle speed control via TIM2_CH3 PWM on PB10.
 *
 * The A1's motor supply (VMOTO) is hard-wired to 5 V on this board through
 * ferrite bead FB2, so the PWM duty cycle is the ONLY control we have over
 * the spindle.  0 permille stops it; 1000 permille runs it flat out.
 *
 * The A1's own documentation asks for a PWM input in the low tens of kHz; we
 * use 20 kHz, which is above audio and well within the driver's range.
 * ------------------------------------------------------------------------- */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* Duty cycle limits, in permille (parts per thousand). */
#define MOTOR_PERMILLE_MAX      1000U

/* Spinning much below this stalls the spindle on most A1 units rather than
 * turning slowly, so the host UI clamps to it as a floor for "running". */
#define MOTOR_PERMILLE_MIN_RUN  250U

/* Sensible starting point.  Measured on this board: duty 600 -> 3.5 rev/s and
 * duty 900 -> 6.2 rev/s, so ~820 lands on the A1's nominal 5.5 rev/s.  600 was
 * the original default and it turned out to be marginal -- the spindle ran
 * below its rated speed and eventually stalled during a long session. */
#define MOTOR_PERMILLE_DEFAULT  820U

/* Configure PB10 as TIM2_CH3 and start the timer with the duty at zero. */
void motor_init(void);

/* Set the duty cycle, clamped to [0, MOTOR_PERMILLE_MAX]. */
void motor_set_permille(uint16_t permille);

/* Last value actually programmed (post-clamp). */
uint16_t motor_get_permille(void);

#endif /* MOTOR_H */
