/* ---------------------------------------------------------------------------
 * rplidar.h -- Slamtec RPLIDAR A1 protocol driver (standard SCAN mode).
 *
 * Responsibilities: bring the device up, keep it scanning, decode the 5-byte
 * measurement nodes coming off USART1, and assemble them into complete 360
 * degree revolutions.  It does no geometry and no fusion -- it hands whole
 * rings to its caller and nothing more.
 *
 * ---------------------------------------------------------------------------
 * WHY STANDARD SCAN AND NOT EXPRESS SCAN
 * ---------------------------------------------------------------------------
 * The A1 link is 115200 baud.  Standard SCAN spends 5 bytes per measurement,
 * so the wire caps out at 11520/5 = ~2300 samples/s no matter what the sensor
 * is capable of internally.  At the A1's nominal 5.5 rev/s that is ~400
 * points per revolution, which matches the density this project was designed
 * around.  EXPRESS scan's capsuled format would raise the ceiling, but it is
 * a substantially more involved decoder (delta-angle cabins, capsule-to-
 * capsule interpolation) and is not needed to hit the target density.
 * ------------------------------------------------------------------------- */

#ifndef RPLIDAR_H
#define RPLIDAR_H

#include <stdbool.h>
#include <stdint.h>

/* Upper bound on points kept per revolution.  At the wire's ~2300 samples/s
 * ceiling this is reached only below ~2.3 rev/s, i.e. a nearly stalled
 * spindle.  Rings longer than this are truncated and flagged rather than
 * silently corrupting the packet length. */
#define RPLIDAR_MAX_POINTS   1024U

/* Decimation: keep 1 point in N.  This is the "sampling rate" control the
 * host exposes -- see the note in rplidar.c about what is and is not
 * adjustable on an A1. */
#define RPLIDAR_DECIM_MIN    1U
#define RPLIDAR_DECIM_MAX    16U

typedef struct {
    uint16_t angle_q6;    /* degrees * 64, 0 .. 23039                        */
    uint16_t dist_mm;     /* millimetres; 0 means "no return" (invalid)      */
} rplidar_point_t;

typedef struct {
    uint32_t        t_us;         /* micros() when the revolution closed     */
    uint32_t        seq;          /* revolutions since scanning started      */
    uint16_t        count;        /* points in pts[]                         */
    bool            truncated;    /* ring hit RPLIDAR_MAX_POINTS             */
    rplidar_point_t pts[RPLIDAR_MAX_POINTS];
} rplidar_ring_t;

typedef enum {
    RPLIDAR_OK = 0,
    RPLIDAR_ERR_NO_RESPONSE,   /* device never answered a command            */
    RPLIDAR_ERR_BAD_HEALTH,    /* device reported an error state             */
    RPLIDAR_ERR_BAD_DESCRIPTOR /* answered, but not with the expected header */
} rplidar_status_t;

/* Called from rplidar_poll() each time a revolution completes.  The ring is
 * owned by the driver and is only valid for the duration of the call. */
typedef void (*rplidar_ring_cb_t)(const rplidar_ring_t *ring);

/* `service` is called inside the driver's bounded blocking waits so the
 * caller can keep the watchdog fed and the USB link pumped.  May be NULL. */
void rplidar_init(rplidar_ring_cb_t on_ring, void (*service)(void));

/* Query health, then put the device into standard SCAN mode.  Assumes the
 * spindle is already turning (see motor.h) -- the caller spins it up first
 * and gives it a moment to reach speed. */
rplidar_status_t rplidar_start_scan(void);

/* Send STOP and go idle.  The motor is NOT touched; that is motor.c's job. */
void rplidar_stop_scan(void);

/* Drain USART1 and decode.  Call often from the main loop; it never blocks. */
void rplidar_poll(void);

/* ---- Controls ---------------------------------------------------------- */
void     rplidar_set_decimation(uint8_t every_nth);
uint8_t  rplidar_get_decimation(void);

/* ---- Observability ----------------------------------------------------- */
bool     rplidar_is_scanning(void);
float    rplidar_rotation_hz(void);     /* measured, from ring-close times   */
float    rplidar_sample_hz(void);       /* measured, points/s BEFORE decim   */
uint32_t rplidar_ring_count(void);
uint32_t rplidar_resync_count(void);    /* node-framing resynchronisations   */
uint32_t rplidar_last_node_us(void);    /* micros() of the last valid node   */
/* micros() of the last completed revolution.  Distinct from the node time on
 * purpose: a spindle that has stalled keeps streaming nodes at a frozen
 * angle, so nodes continue while revolutions stop.  A watchdog that only
 * watches nodes cannot see that failure. */
uint32_t rplidar_last_ring_us(void);
uint8_t  rplidar_health(void);          /* 0 good, 1 warning, 2 error        */

#endif /* RPLIDAR_H */
