/* ---------------------------------------------------------------------------
 * main.c -- Mini-LiDAR-Board IMU orientation firmware, milestone 1.
 *
 * What this firmware does:
 *   - brings the STM32F411CEU6 up to 96 MHz from the 8 MHz crystal, with the
 *     exact 48 MHz PLLQ that USB needs;
 *   - configures the on-board ICM-42688-P for 1 kHz accel + gyro in low-noise
 *     mode and reads it over DMA on every data-ready interrupt;
 *   - measures and removes the gyroscope's zero-rate bias at start-up;
 *   - runs a 6-DOF Madgwick filter at the full 1 kHz sample rate;
 *   - streams the resulting quaternion to the host at ~100 Hz over a
 *     bare-metal USB CDC-ACM link, in the framing described in PROTOCOL.md.
 *
 *   - drives the RPLIDAR A1 spindle with TIM2_CH3 PWM, decodes its standard
 *     SCAN stream off USART1 (circular DMA), and forwards each complete
 *     revolution to the host paired with the quaternion at ring-close time.
 *
 * What this firmware deliberately does NOT do: any 3D reconstruction.  Rings
 * are shipped in the sensor's own 2D polar frame; the quaternion travels
 * alongside them but nothing rotates anything yet.
 *
 * ---------------------------------------------------------------------------
 * A NOTE ON STATUS REPORTING
 * ---------------------------------------------------------------------------
 * This board has no GPIO-driven LEDs.  D3 is a power indicator hard-wired
 * across the 3.3 V rail; the MCU cannot touch it.  Every piece of firmware
 * state is therefore reported one of two ways:
 *   1. as a STATUS packet on the USB link (the primary channel), or
 *   2. as an edge on DEBUG_A/B/C (PB3/PB4/PB5, header J4) for a scope.
 * There is no blink code anywhere in this project, and there cannot be.
 * ------------------------------------------------------------------------- */

#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "clock.h"
#include "debug_pins.h"
#include "icm42688.h"
#include "iwdg.h"
#include "madgwick.h"
#include "motor.h"
#include "packet.h"
#include "pkt_ring.h"
#include "rplidar.h"
#include "spi1_dma.h"
#include "tim_us.h"
#include "usart1_dma.h"
#include "usb_cdc.h"
#include "util.h"

/* ---------------------------------------------------------------------------
 * Tunables
 * ------------------------------------------------------------------------- */

/* Madgwick gain.  Madgwick's own guidance is beta = sqrt(3/4) * gyro
 * measurement error in rad/s; 0.1 corresponds to roughly 6.6 deg/s of assumed
 * gyro error, which is a good general-purpose starting point for a
 * hand-held board.  Runtime-tunable from the host (PKT_TYPE_CMD_SET_BETA). */
#define MADGWICK_BETA_DEFAULT   0.1f

/* Stream every Nth filter update.  1 kHz / 10 = 100 Hz, which is far more
 * than a screen can show and keeps the USB link at ~7 kB/s. */
#define PACKET_DECIMATION       10U

/* Gyro bias calibration: discard the first samples (the filters inside the
 * IMU are still settling), then average this many. */
#define CALIB_SETTLE_SAMPLES    200U
#define CALIB_AVERAGE_SAMPLES   2000U
/* If the board moves during calibration the average is meaningless.  Reject
 * and retry if any axis exceeds this rate, in deg/s. */
#define CALIB_MAX_RATE_DPS      5.0f
#define CALIB_MAX_ATTEMPTS      3U

/* Declare the IMU stalled if no data-ready edge arrives for this long.  At
 * 1 kHz we expect one every millisecond. */
#define IMU_STALL_TIMEOUT_US    200000U

/* Declare the LiDAR stalled if no valid measurement node arrives for this
 * long.  A healthy A1 delivers one every ~430 us. */
#define LIDAR_STALL_TIMEOUT_US  1500000U

/* Declare the SPINDLE stalled if no revolution completes for this long.
 *
 * This is a separate failure from the one above, and the one that actually
 * bit us: when the spindle stalls, the A1 keeps streaming measurement nodes
 * at a frozen angle, so the node watchdog stays happy forever while no ring
 * ever closes.  At the slowest useful spin (~2 rev/s) a revolution takes
 * 500 ms, so 3 s is comfortably clear of normal operation. */
#define SPINDLE_STALL_TIMEOUT_US  3000000U

/* How long to hold full duty when trying to break a stalled spindle free.
 * Static friction takes more torque to overcome than rotation does to
 * sustain, so a brief kick restarts a motor that will then happily run at
 * the commanded duty. */
#define SPINDLE_KICK_MS         400U

/* How long to let the spindle come up to speed before asking it to scan.
 * The A1 has no "at speed" report in standard SCAN mode, so this is an open
 * loop wait; the measured rotation rate afterwards is the real check. */
#define LIDAR_SPINUP_MS         2000U

#define DEG_TO_RAD              0.01745329252f

/* ---------------------------------------------------------------------------
 * State
 * ------------------------------------------------------------------------- */
static float    s_gyro_bias_dps[3];
static bool     s_bias_valid;
static bool     s_imu_ok;
static uint8_t  s_who_am_i;
static uint32_t s_stall_recoveries;
static uint32_t s_last_sample_t_us;
static bool     s_have_last_sample;

/* Set from the USB interrupt by the command parser, acted on in the main
 * loop so that no host command can run inside an ISR. */
static volatile bool  s_cmd_recalibrate;
static volatile bool  s_cmd_ping;
static volatile bool  s_cmd_beta_changed;

static bool     s_lidar_ok;
static bool     s_lidar_want_scan = true;
static uint32_t s_lidar_recoveries;
static uint32_t s_scan_frames_sent;

/* Set from the USB interrupt, acted on in the main loop. */
static volatile bool     s_cmd_motor_changed;
static volatile uint16_t s_cmd_motor_permille;
static volatile bool     s_cmd_decim_changed;
static volatile uint8_t  s_cmd_decim;
static volatile bool     s_cmd_lidar_enable_changed;
static volatile uint8_t  s_cmd_lidar_enable;

static uint8_t  s_frame[PKT_MAX_FRAME];

/* The scan serialiser walks the point array as interleaved uint16 pairs, so
 * the driver's point struct must be exactly that and nothing more. */
STATIC_ASSERT(sizeof(rplidar_point_t) == 4U, "rplidar_point_t must be 2 x u16");

/* ---------------------------------------------------------------------------
 * Status reporting
 * ------------------------------------------------------------------------- */
static uint8_t current_flags(void)
{
    uint8_t f = 0U;

    if (s_imu_ok)                { f |= PKT_FLAG_IMU_OK; }
    if (s_bias_valid)            { f |= PKT_FLAG_BIAS_VALID; }
    if (usb_cdc_configured())    { f |= PKT_FLAG_USB_CONFIGURED; }
    if (!s_bias_valid)           { f |= PKT_FLAG_CALIBRATING; }
    if (s_stall_recoveries > 0U) { f |= PKT_FLAG_STALL_RECOVERED; }
    if (s_lidar_ok)              { f |= PKT_FLAG_LIDAR_OK; }
    if (rplidar_is_scanning())   { f |= PKT_FLAG_LIDAR_SCANNING; }
    return f;
}

static void send_status(pkt_status_code_t code, const char *msg)
{
    pkt_status_t st;

    st.t_us      = micros();
    st.uptime_ms = millis();
    st.code      = (uint8_t)code;
    st.who_am_i  = s_who_am_i;
    st.flags     = current_flags();
    pkt_set_msg(&st, msg);

    const uint16_t n = pkt_build_status(s_frame, &st);
    (void)usb_cdc_send_frame(s_frame, n);
}

/* ---------------------------------------------------------------------------
 * Host command channel (bulk OUT).  Runs in the USB interrupt: it validates
 * the frame and records the request, and does nothing expensive.
 * ------------------------------------------------------------------------- */
static void on_usb_rx(const uint8_t *data, uint16_t len)
{
    /* Smallest possible command frame is header + CRC with no payload. */
    if (len < PKT_OVERHEAD) {
        return;
    }
    if ((data[0] != PKT_SYNC0) || (data[1] != PKT_SYNC1) || (data[2] != PKT_VERSION)) {
        return;
    }

    const uint16_t payload_len = (uint16_t)(data[4] | ((uint16_t)data[5] << 8));

    if ((uint32_t)payload_len + PKT_OVERHEAD > len) {
        return;                 /* truncated or bogus length */
    }

    const uint16_t want = pkt_crc16(&data[2], (uint16_t)(4U + payload_len));
    const uint16_t got  = (uint16_t)(data[PKT_HEADER_LEN + payload_len]
                        | ((uint16_t)data[PKT_HEADER_LEN + payload_len + 1U] << 8));

    if (want != got) {
        return;
    }

    switch (data[3]) {
    case PKT_TYPE_CMD_SET_BETA:
        if (payload_len >= 4U) {
            union { uint32_t u; float f; } conv;

            conv.u = (uint32_t)data[6]
                   | ((uint32_t)data[7] << 8)
                   | ((uint32_t)data[8] << 16)
                   | ((uint32_t)data[9] << 24);

            /* Reject NaN and absurd gains rather than poisoning the filter.
             * (x != x) is the standard NaN test without <math.h>. */
            if ((conv.f == conv.f) && (conv.f >= 0.0f) && (conv.f <= 10.0f)) {
                madgwick_set_beta(conv.f);
                s_cmd_beta_changed = true;
            }
        }
        break;

    case PKT_TYPE_CMD_RECALIBRATE:
        s_cmd_recalibrate = true;
        break;

    case PKT_TYPE_CMD_PING:
        s_cmd_ping = true;
        break;

    case PKT_TYPE_CMD_SET_MOTOR:
        if (payload_len >= 2U) {
            s_cmd_motor_permille = (uint16_t)(data[6] | ((uint16_t)data[7] << 8));
            s_cmd_motor_changed  = true;
        }
        break;

    case PKT_TYPE_CMD_SET_DECIM:
        if (payload_len >= 1U) {
            s_cmd_decim         = data[6];
            s_cmd_decim_changed = true;
        }
        break;

    case PKT_TYPE_CMD_LIDAR_ENABLE:
        if (payload_len >= 1U) {
            s_cmd_lidar_enable         = data[6];
            s_cmd_lidar_enable_changed = true;
        }
        break;

    default:
        break;
    }
}

/* ---------------------------------------------------------------------------
 * Gyro bias calibration
 *
 * Zero-rate offset, not noise, is what dominates yaw drift on a MEMS gyro: a
 * steady 1 deg/s bias is 60 degrees of error after a minute, whereas the
 * random walk is a fraction of that.  A few seconds of averaging while the
 * board sits still removes almost all of it.
 *
 * This runs before the filter starts and blocks the main loop, so it refreshes
 * the watchdog and services USB itself.
 * ------------------------------------------------------------------------- */
static bool calibrate_gyro_bias(void)
{
    for (uint32_t attempt = 0U; attempt < CALIB_MAX_ATTEMPTS; attempt++) {
        icm_sample_t s;
        float        sum[3] = { 0.0f, 0.0f, 0.0f };
        uint32_t     settled = 0U;
        uint32_t     taken   = 0U;
        bool         moved   = false;
        uint32_t     started = micros();

        while (taken < CALIB_AVERAGE_SAMPLES) {
            iwdg_refresh();
            usb_cdc_poll();

            /* Give up on this attempt if the IMU stops delivering; the caller
             * will report the failure. */
            if ((micros() - started) > 20000000UL) {
                return false;
            }

            if (!icm_pop_sample(&s)) {
                continue;
            }

            if (settled < CALIB_SETTLE_SAMPLES) {
                settled++;
                continue;
            }

            if ((s.gx > CALIB_MAX_RATE_DPS) || (s.gx < -CALIB_MAX_RATE_DPS)
             || (s.gy > CALIB_MAX_RATE_DPS) || (s.gy < -CALIB_MAX_RATE_DPS)
             || (s.gz > CALIB_MAX_RATE_DPS) || (s.gz < -CALIB_MAX_RATE_DPS)) {
                moved = true;
                break;
            }

            sum[0] += s.gx;
            sum[1] += s.gy;
            sum[2] += s.gz;
            taken++;
        }

        if (!moved && (taken == CALIB_AVERAGE_SAMPLES)) {
            s_gyro_bias_dps[0] = sum[0] / (float)CALIB_AVERAGE_SAMPLES;
            s_gyro_bias_dps[1] = sum[1] / (float)CALIB_AVERAGE_SAMPLES;
            s_gyro_bias_dps[2] = sum[2] / (float)CALIB_AVERAGE_SAMPLES;
            return true;
        }

        send_status(PKT_STATUS_CALIB_START, "board moved, retrying");
    }

    /* Out of attempts: run with no bias correction rather than refusing to
     * start.  The host is told, and the operator can ask for another go. */
    s_gyro_bias_dps[0] = 0.0f;
    s_gyro_bias_dps[1] = 0.0f;
    s_gyro_bias_dps[2] = 0.0f;
    return false;
}

static void run_calibration(void)
{
    s_bias_valid = false;
    send_status(PKT_STATUS_CALIB_START, "hold still: gyro bias cal");

    const bool ok = calibrate_gyro_bias();

    s_bias_valid = ok;
    madgwick_init(madgwick_get_beta());   /* start from identity afterwards */
    s_have_last_sample = false;

    send_status(ok ? PKT_STATUS_CALIB_DONE : PKT_STATUS_CALIB_START,
                ok ? "gyro bias captured" : "cal failed, bias = 0");
}

/* ---------------------------------------------------------------------------
 * Housekeeping callback handed to the LiDAR driver.
 *
 * rplidar_start_scan() has to wait on a device that may take half a second to
 * answer.  Rather than let that stall the watchdog and the USB link, the
 * driver calls this from inside its wait loops.  It deliberately does NOT
 * touch the IMU: samples keep queueing in the driver's own ring and are
 * drained by the main loop afterwards.
 * ------------------------------------------------------------------------- */
static void housekeeping(void)
{
    iwdg_refresh();
    usb_cdc_poll();
}

/* ---------------------------------------------------------------------------
 * One completed LiDAR revolution.  Called from rplidar_poll(), i.e. from the
 * main loop -- not from an interrupt -- so building and queueing a ~1.5 KB
 * frame here is safe.
 * ------------------------------------------------------------------------- */
static void on_lidar_ring(const rplidar_ring_t *ring)
{
    const quat_t      q = madgwick_quat();
    pkt_scan_header_t h;

    h.t_us           = ring->t_us;
    h.seq            = ring->seq;
    h.q[0]           = q.w;
    h.q[1]           = q.x;
    h.q[2]           = q.y;
    h.q[3]           = q.z;
    h.rot_hz         = rplidar_rotation_hz();
    h.sample_hz      = rplidar_sample_hz();
    h.point_count    = ring->count;
    h.motor_permille = motor_get_permille();
    h.decimation     = rplidar_get_decimation();
    h.flags          = current_flags();
    h.reserved       = 0U;

    if (ring->truncated) {
        h.flags |= PKT_FLAG_SCAN_TRUNCATED;
    }

    const uint16_t n = pkt_build_scan(s_frame, &h, (const uint16_t *)ring->pts);

    if (n != 0U) {
        DEBUG_C_TOGGLE();
        (void)usb_cdc_send_frame(s_frame, n);
        s_scan_frames_sent++;
    }
}

/* ---------------------------------------------------------------------------
 * LiDAR bring-up, shared by the initial start and by stall recovery.
 * The spindle must already be turning before this is called.
 * ------------------------------------------------------------------------- */
static bool bring_up_lidar(void)
{
    const rplidar_status_t st = rplidar_start_scan();

    switch (st) {
    case RPLIDAR_OK:
        s_lidar_ok = true;
        send_status(PKT_STATUS_LIDAR_OK, "RPLIDAR A1 scanning");
        return true;

    case RPLIDAR_ERR_BAD_HEALTH:
        s_lidar_ok = false;
        send_status(PKT_STATUS_LIDAR_BAD_HEALTH, "LiDAR reports error state");
        return false;

    case RPLIDAR_ERR_BAD_DESCRIPTOR:
        s_lidar_ok = false;
        send_status(PKT_STATUS_LIDAR_BAD_DESC, "unexpected LiDAR descriptor");
        return false;

    case RPLIDAR_ERR_NO_RESPONSE:
    default:
        s_lidar_ok = false;
        send_status(PKT_STATUS_LIDAR_NO_RESPONSE, "no UART reply from LiDAR");
        return false;
    }
}

/* ---------------------------------------------------------------------------
 * IMU bring-up, shared by the initial start and by stall recovery
 * ------------------------------------------------------------------------- */
static bool bring_up_imu(void)
{
    const icm_status_t st = icm_init(&s_who_am_i);

    switch (st) {
    case ICM_OK:
        s_imu_ok = true;
        send_status(PKT_STATUS_IMU_OK, "ICM-42688-P ready");
        return true;

    case ICM_ERR_WHO_AM_I:
        s_imu_ok = false;
        send_status(PKT_STATUS_IMU_WHO_AM_I_FAIL, "WHO_AM_I != 0x47");
        return false;

    case ICM_ERR_NO_RESPONSE:
    default:
        s_imu_ok = false;
        send_status(PKT_STATUS_IMU_NO_RESPONSE, "no SPI response from IMU");
        return false;
    }
}

/* ---------------------------------------------------------------------------
 * One filter step for one sample
 * ------------------------------------------------------------------------- */
static void process_sample(const icm_sample_t *s, uint32_t *decim_counter)
{
    /* dt from the hardware timestamps, so a dropped sample lengthens the step
     * instead of silently slowing the integration down. */
    float dt;

    if (s_have_last_sample) {
        const uint32_t delta_us = s->t_us - s_last_sample_t_us;  /* wrap-safe */

        dt = (float)delta_us * 1e-6f;

        /* Clamp: a wrap of the 32-bit microsecond counter, or a long stall,
         * would otherwise hand the filter a huge dt and throw the quaternion
         * across the sphere. */
        if ((dt <= 0.0f) || (dt > 0.02f)) {
            dt = 1.0f / (float)ICM_ODR_HZ;
        }
    } else {
        dt = 1.0f / (float)ICM_ODR_HZ;
        s_have_last_sample = true;
    }
    s_last_sample_t_us = s->t_us;

    const float gx_dps = s->gx - s_gyro_bias_dps[0];
    const float gy_dps = s->gy - s_gyro_bias_dps[1];
    const float gz_dps = s->gz - s_gyro_bias_dps[2];

    DEBUG_B_TOGGLE();
    madgwick_update_imu(gx_dps * DEG_TO_RAD,
                        gy_dps * DEG_TO_RAD,
                        gz_dps * DEG_TO_RAD,
                        s->ax, s->ay, s->az,
                        dt);

    (*decim_counter)++;
    if (*decim_counter < PACKET_DECIMATION) {
        return;
    }
    *decim_counter = 0U;

    const quat_t q = madgwick_quat();
    pkt_orientation_t o;

    o.t_us         = s->t_us;
    o.q[0]         = q.w;
    o.q[1]         = q.x;
    o.q[2]         = q.y;
    o.q[3]         = q.z;
    o.accel_g[0]   = s->ax;
    o.accel_g[1]   = s->ay;
    o.accel_g[2]   = s->az;
    o.gyro_dps[0]  = gx_dps;
    o.gyro_dps[1]  = gy_dps;
    o.gyro_dps[2]  = gz_dps;
    o.temp_c       = s->temp_c;
    o.sample_count = icm_sample_count();
    o.drop_count   = usb_cdc_tx_drops();
    o.imu_odr_hz   = ICM_ODR_HZ;
    o.flags        = current_flags();

    const uint16_t n = pkt_build_orientation(s_frame, &o);

    DEBUG_C_TOGGLE();
    (void)usb_cdc_send_frame(s_frame, n);
}

/* ---------------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------------- */
int main(void)
{
    if (!clock_init()) {
        /* Without the crystal there is no 48 MHz for USB and no way to report
         * anything.  Reset and try again; a genuinely dead crystal turns into
         * a boot loop, which is at least diagnosable with a scope on PH0. */
        NVIC_SystemReset();
    }

    tim_us_init();
    debug_pins_init();
    pkt_ring_init();

    madgwick_init(MADGWICK_BETA_DEFAULT);

    /* USB comes up before the IMU so that an IMU failure can be reported over
     * the link.  Frames queued before enumeration completes simply sit in the
     * ring buffer (oldest discarded on overflow) until the host attaches. */
    usb_cdc_set_rx_callback(on_usb_rx);
    usb_cdc_init();

    spi1_init();

    send_status(PKT_STATUS_BOOT, "boot: 96 MHz, USB up");

    s_imu_ok = bring_up_imu();

    /* ---- Gyro bias calibration happens BEFORE the spindle starts ---------
     * The calibration averages the gyro while the board is supposed to be
     * still, and rejects the attempt if any axis exceeds a few deg/s.  A
     * spinning LiDAR motor couples real vibration into the IMU, which would
     * either bias the average or trip the motion rejection.  So: calibrate
     * first, spin second. */
    if (s_imu_ok) {
        run_calibration();
    }

    /* ---- LiDAR -----------------------------------------------------------
     * Spin the spindle up, give it a couple of seconds to reach speed, then
     * ask the device to scan.  The motor supply is hard-wired on, so the PWM
     * duty is the only control we have. */
    motor_init();
    usart1_init();
    rplidar_init(on_lidar_ring, housekeeping);

    motor_set_permille(MOTOR_PERMILLE_DEFAULT);

    for (uint32_t i = 0U; i < LIDAR_SPINUP_MS; i++) {
        delay_ms(1U);
        usb_cdc_poll();
    }

    s_lidar_ok = bring_up_lidar();

    /* Start the watchdog only once all the slow, blocking init is finished. */
    iwdg_init();

    uint32_t decim = 0U;
    uint32_t last_stall_check_us = micros();

    uint32_t last_lidar_check_us = micros();

    for (;;) {
        iwdg_refresh();
        usb_cdc_poll();

        /* ---- Drain every sample the ISRs have queued -------------------- */
        icm_sample_t s;

        while (icm_pop_sample(&s)) {
            process_sample(&s, &decim);
        }

        /* ---- Drain the LiDAR UART and emit any completed revolution -----
         * Deliberately after the IMU: the quaternion paired with a ring
         * should be as fresh as possible at the moment the ring closes. */
        rplidar_poll();

        /* ---- Host commands ---------------------------------------------- */
        if (s_cmd_ping) {
            s_cmd_ping = false;
            send_status(PKT_STATUS_PONG, "pong");
        }
        if (s_cmd_beta_changed) {
            s_cmd_beta_changed = false;
            send_status(PKT_STATUS_BETA_CHANGED, "beta updated");
        }
        if (s_cmd_recalibrate) {
            s_cmd_recalibrate = false;
            if (s_imu_ok) {
                run_calibration();
            }
        }
        if (s_cmd_motor_changed) {
            s_cmd_motor_changed = false;
            motor_set_permille(s_cmd_motor_permille);
            send_status(PKT_STATUS_MOTOR_CHANGED, "motor duty updated");
        }
        if (s_cmd_decim_changed) {
            s_cmd_decim_changed = false;
            rplidar_set_decimation(s_cmd_decim);
            send_status(PKT_STATUS_DECIM_CHANGED, "decimation updated");
        }
        if (s_cmd_lidar_enable_changed) {
            s_cmd_lidar_enable_changed = false;
            s_lidar_want_scan = (s_cmd_lidar_enable != 0U);

            if (s_lidar_want_scan) {
                s_lidar_ok = bring_up_lidar();
            } else {
                rplidar_stop_scan();
                s_lidar_ok = false;
                send_status(PKT_STATUS_LIDAR_STOPPED, "scan stopped by host");
            }
        }

        /* ---- IMU stall detection and recovery ---------------------------
         * If INT1 stops pulsing -- a glitched configuration write, a brown-out
         * on the IMU rail, a stuck DMA -- re-initialise the device rather than
         * streaming a frozen quaternion forever.  Checked at 10 Hz so the
         * happy path costs nothing. */
        if ((micros() - last_stall_check_us) > 100000UL) {
            last_stall_check_us = micros();

            if ((micros() - icm_last_drdy_us()) > IMU_STALL_TIMEOUT_US) {
                send_status(PKT_STATUS_IMU_STALL, "no IMU data-ready");

                icm_irq_disable();
                if (bring_up_imu()) {
                    s_stall_recoveries++;
                    s_have_last_sample = false;
                    send_status(PKT_STATUS_IMU_RECOVERED, "IMU re-initialised");
                }
                /* Whether or not recovery worked, the loop keeps running and
                 * keeps refreshing the watchdog, so the link stays alive and
                 * the host keeps seeing status packets. */
            }
        }

        /* ---- LiDAR stall detection and recovery -------------------------
         * Only meaningful while we are supposed to be scanning AND the motor
         * is actually commanded to turn: a deliberately stopped spindle
         * produces no nodes, and that is not a fault. */
        if ((micros() - last_lidar_check_us) > 500000UL) {
            last_lidar_check_us = micros();

            const bool should_be_running =
                s_lidar_want_scan && (motor_get_permille() >= MOTOR_PERMILLE_MIN_RUN);

            /* (a) The device has gone quiet altogether. */
            const bool nodes_dead = should_be_running
                && ((micros() - rplidar_last_node_us()) > LIDAR_STALL_TIMEOUT_US);

            /* (b) The device is still talking but the spindle is not turning.
             * Nodes keep arriving at a frozen angle, so (a) never fires. */
            const bool spindle_dead = should_be_running && rplidar_is_scanning()
                && ((micros() - rplidar_last_ring_us()) > SPINDLE_STALL_TIMEOUT_US);

            if (nodes_dead || spindle_dead) {
                send_status(PKT_STATUS_LIDAR_STALL,
                            spindle_dead ? "spindle stalled: no revolutions"
                                         : "no LiDAR measurements");

                if (spindle_dead) {
                    /* Kick the motor to full duty briefly to break static
                     * friction, then hand it back the commanded duty. */
                    const uint16_t want = motor_get_permille();

                    motor_set_permille(MOTOR_PERMILLE_MAX);
                    for (uint32_t i = 0U; i < SPINDLE_KICK_MS; i++) {
                        delay_ms(1U);
                        iwdg_refresh();
                        usb_cdc_poll();
                    }
                    motor_set_permille(want);
                }

                usart1_init();               /* re-arm the DMA from scratch  */
                if (bring_up_lidar()) {
                    s_lidar_recoveries++;
                    send_status(PKT_STATUS_LIDAR_RECOVERED, "LiDAR restarted");
                }
            }
        }
    }
}
