/* ---------------------------------------------------------------------------
 * rplidar.c -- see rplidar.h.
 *
 * ---------------------------------------------------------------------------
 * PROTOCOL SUMMARY (Slamtec "RPLIDAR Interface Protocol and Application Notes")
 * ---------------------------------------------------------------------------
 * Requests are 2 bytes for the commands used here:  0xA5, <command>.
 *
 * Commands that produce output answer with a 7-byte response descriptor:
 *
 *     A5 5A <len:30 | mode:2, 4 bytes LE> <data type, 1 byte>
 *
 * then either one payload of `len` bytes (mode 0, single response) or an
 * unbounded stream of `len`-byte payloads (mode 1, multiple response).
 *
 *   STOP           0x25  no answer at all; device needs ~10 ms to settle
 *   GET_HEALTH     0x52  descriptor + 3 bytes: status, error_code (LE16)
 *   SCAN           0x20  descriptor + endless 5-byte measurement nodes
 *
 * Measurement node, 5 bytes:
 *
 *   byte 0   bit 0     S       start of a new revolution
 *            bit 1     !S      inverse of S -- must differ, else desync
 *            bits 2-7  quality
 *   byte 1   bit 0     C       check bit, always 1
 *            bits 1-7  angle_q6[6:0]
 *   byte 2             angle_q6[14:7]
 *   bytes 3-4          distance_q2, little-endian uint16
 *
 *   angle_deg  = angle_q6 / 64      distance_mm = distance_q2 / 4
 *
 * The two redundancy bits (S xor !S, and C) are what make byte-level
 * resynchronisation possible: on a mismatch we discard one byte and try
 * again, rather than staying permanently one byte out of phase.
 *
 * ---------------------------------------------------------------------------
 * WHAT "SAMPLING RATE" MEANS ON AN A1
 * ---------------------------------------------------------------------------
 * The A1 has no command to change its internal measurement rate in standard
 * SCAN mode -- the rate is fixed by the firmware and then bounded further by
 * the 115200 baud link.  What the operator can genuinely control is:
 *
 *   - the SPINDLE SPEED, via the PWM pin (motor.c).  Slower spin = more
 *     points per revolution, because the sample rate is constant.
 *   - the DECIMATION applied here, which thins the points actually forwarded
 *     to the host.
 *
 * Both are exposed to the host UI; the driver also measures and reports the
 * true rates so the display never has to guess.
 * ------------------------------------------------------------------------- */

#include "rplidar.h"
#include "board.h"
#include "debug_pins.h"
#include "tim_us.h"
#include "usart1_dma.h"
#include "util.h"

/* ---- Commands ---------------------------------------------------------- */
#define RP_SYNC_BYTE       0xA5U
#define RP_CMD_STOP        0x25U
#define RP_CMD_SCAN        0x20U
#define RP_CMD_GET_HEALTH  0x52U

#define RP_DESC_SYNC1      0xA5U
#define RP_DESC_SYNC2      0x5AU
#define RP_DTYPE_SCAN      0x81U
#define RP_DTYPE_HEALTH    0x06U

/* ---- Node field extraction --------------------------------------------- */
#define NODE_LEN           5U
#define NODE_START(b0)     ((b0) & 0x01U)
#define NODE_START_BAR(b0) (((b0) >> 1) & 0x01U)
#define NODE_CHECK(b1)     ((b1) & 0x01U)

typedef enum {
    ST_IDLE = 0,
    ST_WAIT_SCAN_DESCRIPTOR,
    ST_STREAMING
} rp_state_t;

static rplidar_ring_cb_t s_on_ring;
static void            (*s_service)(void);

static rp_state_t   s_state;
static uint8_t      s_node[NODE_LEN];
static uint8_t      s_node_len;

static rplidar_ring_t s_ring;
static uint32_t     s_ring_seq;
static uint32_t     s_ring_count;
static uint32_t     s_resyncs;
static uint32_t     s_last_node_us;
static uint32_t     s_last_ring_us;
static bool         s_have_last_ring;
static uint8_t      s_health;

static uint8_t      s_decim = RPLIDAR_DECIM_MIN;
static uint32_t     s_decim_phase;

/* Raw node count in the ring currently being assembled, i.e. before
 * decimation.  Needed to report a true sample rate. */
static uint32_t     s_raw_in_ring;

static float        s_rot_hz;
static float        s_sample_hz;

/* Discard the first revolution after SCAN starts: we almost certainly began
 * listening partway through one, so it would be a partial arc. */
static bool         s_drop_first_ring;

static void service(void)
{
    if (s_service != NULL) {
        s_service();
    }
}

/* Bounded wait for `n` bytes.  Returns false on timeout.  Keeps the caller's
 * housekeeping running so the watchdog does not fire and USB stays alive. */
static bool read_exact(uint8_t *dst, uint16_t n, uint32_t timeout_us)
{
    const uint32_t started = micros();
    uint16_t       got     = 0U;

    while (got < n) {
        got += usart1_read(&dst[got], (uint16_t)(n - got));

        if (got < n) {
            service();
            if ((micros() - started) > timeout_us) {
                return false;
            }
        }
    }
    return true;
}

static bool send_cmd(uint8_t cmd)
{
    const uint8_t req[2] = { RP_SYNC_BYTE, cmd };

    return usart1_write(req, sizeof(req));
}

/* Hunt for the 0xA5 0x5A descriptor preamble, then take the remaining 5
 * bytes.  Hunting rather than assuming alignment matters because a device
 * that was already scanning when we booted has been mid-stream all along. */
static bool read_descriptor(uint8_t *desc, uint32_t timeout_us)
{
    const uint32_t started = micros();
    uint8_t        b;
    uint8_t        matched = 0U;

    while ((micros() - started) <= timeout_us) {
        if (usart1_read(&b, 1U) == 0U) {
            service();
            continue;
        }

        if (matched == 0U) {
            matched = (b == RP_DESC_SYNC1) ? 1U : 0U;
        } else if (matched == 1U) {
            if (b == RP_DESC_SYNC2) {
                matched = 2U;
            } else {
                matched = (b == RP_DESC_SYNC1) ? 1U : 0U;
            }
        }

        if (matched == 2U) {
            desc[0] = RP_DESC_SYNC1;
            desc[1] = RP_DESC_SYNC2;
            return read_exact(&desc[2], 5U, timeout_us);
        }
    }
    return false;
}

void rplidar_init(rplidar_ring_cb_t on_ring, void (*service_cb)(void))
{
    s_on_ring   = on_ring;
    s_service   = service_cb;
    s_state     = ST_IDLE;
    s_node_len  = 0U;
    s_ring.count = 0U;
    s_ring_seq  = 0U;
    s_ring_count = 0U;
    s_resyncs   = 0U;
    s_health    = 0xFFU;          /* unknown until asked                     */
    s_decim     = RPLIDAR_DECIM_MIN;
    s_decim_phase = 0U;
    s_raw_in_ring = 0U;
    s_rot_hz    = 0.0f;
    s_sample_hz = 0.0f;
    s_last_node_us = micros();
    s_last_ring_us = micros();
    s_have_last_ring = false;
    s_drop_first_ring = true;
}

rplidar_status_t rplidar_start_scan(void)
{
    /* ---- 1. Always STOP first ------------------------------------------
     * The device may still be streaming from a previous run (a warm reset of
     * the MCU does not reset the LiDAR), in which case its output would be
     * mistaken for our command responses. */
    (void)send_cmd(RP_CMD_STOP);
    for (uint32_t i = 0U; i < 30U; i++) {   /* ~30 ms, servicing as we go    */
        delay_ms(1U);
        service();
    }
    usart1_rx_flush();

    /* ---- 2. Health check ------------------------------------------------ */
    if (!send_cmd(RP_CMD_GET_HEALTH)) {
        return RPLIDAR_ERR_NO_RESPONSE;
    }

    uint8_t desc[7];

    if (!read_descriptor(desc, 500000UL)) {
        return RPLIDAR_ERR_NO_RESPONSE;
    }
    if (desc[6] != RP_DTYPE_HEALTH) {
        return RPLIDAR_ERR_BAD_DESCRIPTOR;
    }

    uint8_t health[3];

    if (!read_exact(health, sizeof(health), 500000UL)) {
        return RPLIDAR_ERR_NO_RESPONSE;
    }
    s_health = health[0];

    /* status 2 = error: the device wants a RESET before it will scan
     * usefully.  1 = warning, which is still worth scanning through. */
    if (s_health == 2U) {
        return RPLIDAR_ERR_BAD_HEALTH;
    }

    /* ---- 3. Start scanning ---------------------------------------------- */
    usart1_rx_flush();

    if (!send_cmd(RP_CMD_SCAN)) {
        return RPLIDAR_ERR_NO_RESPONSE;
    }

    if (!read_descriptor(desc, 500000UL)) {
        return RPLIDAR_ERR_NO_RESPONSE;
    }
    if (desc[6] != RP_DTYPE_SCAN) {
        return RPLIDAR_ERR_BAD_DESCRIPTOR;
    }

    s_state           = ST_STREAMING;
    s_node_len        = 0U;
    s_ring.count      = 0U;
    s_ring.truncated  = false;
    s_raw_in_ring     = 0U;
    s_decim_phase     = 0U;
    s_drop_first_ring = true;
    s_last_node_us    = micros();
    /* Seed the ring clock to "now" so the caller's revolution watchdog has a
     * sane reference before the first revolution ever closes. */
    s_last_ring_us    = micros();
    s_have_last_ring  = false;
    s_rot_hz          = 0.0f;
    s_sample_hz       = 0.0f;
    return RPLIDAR_OK;
}

void rplidar_stop_scan(void)
{
    (void)send_cmd(RP_CMD_STOP);
    s_state    = ST_IDLE;
    s_node_len = 0U;
    delay_ms(10U);
    usart1_rx_flush();
}

/* Close the ring currently being assembled and hand it to the caller. */
static void close_ring(uint32_t now_us)
{
    s_ring.t_us = now_us;
    s_ring.seq  = s_ring_seq;

    if (s_have_last_ring) {
        const uint32_t period_us = now_us - s_last_ring_us;

        /* Ignore absurd periods (first ring, or a stall) rather than letting
         * them poison the smoothed rate. */
        if ((period_us > 20000UL) && (period_us < 2000000UL)) {
            const float hz  = 1000000.0f / (float)period_us;
            const float sps = (float)s_raw_in_ring * hz;

            /* Light exponential smoothing: the display should show a steady
             * number, not one that twitches with every revolution. */
            s_rot_hz    = (s_rot_hz    == 0.0f) ? hz  : (0.8f * s_rot_hz    + 0.2f * hz);
            s_sample_hz = (s_sample_hz == 0.0f) ? sps : (0.8f * s_sample_hz + 0.2f * sps);
        }
    }
    s_last_ring_us   = now_us;
    s_have_last_ring = true;

    if (s_drop_first_ring) {
        s_drop_first_ring = false;      /* partial arc, not worth showing    */
    } else if ((s_ring.count > 0U) && (s_on_ring != NULL)) {
        s_ring_seq++;
        s_ring_count++;
        DEBUG_A_TOGGLE();               /* scope: one edge per revolution    */
        s_on_ring(&s_ring);
    }

    s_ring.count     = 0U;
    s_ring.truncated = false;
    s_raw_in_ring    = 0U;
    s_decim_phase    = 0U;
}

/* Decode one validated 5-byte node into the ring under construction. */
static void accept_node(const uint8_t *n, uint32_t now_us)
{
    s_last_node_us = now_us;

    if (NODE_START(n[0]) != 0U) {
        close_ring(now_us);             /* this node opens a new revolution  */
    }

    s_raw_in_ring++;

    /* Decimation: keep 1 in N.  Done here rather than on the host so the
     * saving is real -- it shrinks the USB packet, not just the plot. */
    if (s_decim > 1U) {
        const bool keep = (s_decim_phase == 0U);

        s_decim_phase = (s_decim_phase + 1U) % s_decim;
        if (!keep) {
            return;
        }
    }

    if (s_ring.count >= RPLIDAR_MAX_POINTS) {
        s_ring.truncated = true;
        return;
    }

    const uint16_t angle_q6    = (uint16_t)(((uint16_t)n[2] << 7) | (n[1] >> 1));
    const uint16_t distance_q2 = (uint16_t)((uint16_t)n[3] | ((uint16_t)n[4] << 8));

    s_ring.pts[s_ring.count].angle_q6 = angle_q6;
    s_ring.pts[s_ring.count].dist_mm  = (uint16_t)(distance_q2 >> 2);
    s_ring.count++;
}

void rplidar_poll(void)
{
    uint8_t  chunk[128];
    uint16_t n;

    while ((n = usart1_read(chunk, sizeof(chunk))) != 0U) {
        const uint32_t now_us = micros();

        for (uint16_t i = 0U; i < n; i++) {
            if (s_state != ST_STREAMING) {
                continue;               /* drain and discard while idle      */
            }

            s_node[s_node_len] = chunk[i];
            s_node_len++;

            if (s_node_len < NODE_LEN) {
                continue;
            }

            /* ---- Validate the two redundancy bits ----------------------- */
            const bool ok = (NODE_START(s_node[0]) != NODE_START_BAR(s_node[0]))
                         && (NODE_CHECK(s_node[1]) == 1U);

            if (ok) {
                accept_node(s_node, now_us);
                s_node_len = 0U;
            } else {
                /* Out of phase.  Drop the oldest byte and re-test with the
                 * next one shifted in -- this recovers alignment in at most
                 * four more bytes instead of resetting the whole stream. */
                s_node[0] = s_node[1];
                s_node[1] = s_node[2];
                s_node[2] = s_node[3];
                s_node[3] = s_node[4];
                s_node_len = NODE_LEN - 1U;
                s_resyncs++;
            }
        }
    }
}

void rplidar_set_decimation(uint8_t every_nth)
{
    if (every_nth < RPLIDAR_DECIM_MIN) { every_nth = RPLIDAR_DECIM_MIN; }
    if (every_nth > RPLIDAR_DECIM_MAX) { every_nth = RPLIDAR_DECIM_MAX; }

    s_decim       = every_nth;
    s_decim_phase = 0U;
}

uint8_t  rplidar_get_decimation(void) { return s_decim; }
bool     rplidar_is_scanning(void)    { return s_state == ST_STREAMING; }
float    rplidar_rotation_hz(void)    { return s_rot_hz; }
float    rplidar_sample_hz(void)      { return s_sample_hz; }
uint32_t rplidar_ring_count(void)     { return s_ring_count; }
uint32_t rplidar_resync_count(void)   { return s_resyncs; }
uint32_t rplidar_last_node_us(void)   { return s_last_node_us; }
uint32_t rplidar_last_ring_us(void)   { return s_last_ring_us; }
uint8_t  rplidar_health(void)         { return s_health; }
