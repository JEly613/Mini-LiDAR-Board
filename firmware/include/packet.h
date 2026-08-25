/* ---------------------------------------------------------------------------
 * packet.h -- binary wire format shared by the firmware and the Python host.
 *
 * THE NORMATIVE DEFINITION OF THIS FORMAT IS PROTOCOL.md, sitting next to
 * this file.  Both this header and host/lidarscan/protocol.py are written
 * from that document; if you change one, change all three.
 *
 * Frame layout (all multi-byte fields LITTLE-ENDIAN, floats IEEE-754 binary32):
 *
 *     offset  size  field
 *     0       1     sync0     = 0xA5
 *     1       1     sync1     = 0x5A
 *     2       1     version   = 0x01
 *     3       1     type      (see pkt_type_t)
 *     4       2     length    payload length in bytes, uint16
 *     6       N     payload
 *     6+N     2     crc16     CRC-16/CCITT-FALSE over bytes [2 .. 6+N-1]
 *
 * The CRC deliberately excludes the sync bytes: they are a resynchronisation
 * marker, not data, and covering them would add nothing.  It does cover the
 * version, type and length, so a corrupted length cannot make the parser
 * accept a mis-sized frame.
 * ------------------------------------------------------------------------- */

#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

#define PKT_SYNC0            0xA5U
#define PKT_SYNC1            0x5AU
#define PKT_VERSION          0x01U

#define PKT_HEADER_LEN       6U
#define PKT_CRC_LEN          2U
#define PKT_OVERHEAD         (PKT_HEADER_LEN + PKT_CRC_LEN)
/* A full LiDAR revolution dominates this: a 40-byte scan header plus four
 * bytes per point.  RPLIDAR_MAX_POINTS (1024) x 4 + 40 = 4136. */
#define PKT_MAX_PAYLOAD      4160U
#define PKT_MAX_FRAME        (PKT_OVERHEAD + PKT_MAX_PAYLOAD)

/* --------------------------------------------------------------------------
 * Packet types.
 *
 * 0x01-0x0F : device -> host telemetry for this (IMU) milestone
 * 0x10-0x1F : LiDAR telemetry
 * 0x80-0x8F : host -> device commands
 * ------------------------------------------------------------------------ */
typedef enum {
    PKT_TYPE_ORIENTATION      = 0x01U,  /* quaternion + scaled IMU, ~100 Hz  */
    PKT_TYPE_STATUS           = 0x02U,  /* asynchronous state / error report */

    PKT_TYPE_LIDAR_SCAN       = 0x10U,  /* one complete revolution          */

    PKT_TYPE_CMD_SET_BETA     = 0x80U,  /* payload: float32 beta             */
    PKT_TYPE_CMD_RECALIBRATE  = 0x81U,  /* payload: none                     */
    PKT_TYPE_CMD_PING         = 0x82U,  /* payload: none, answered by STATUS */
    PKT_TYPE_CMD_SET_MOTOR    = 0x83U,  /* payload: uint16 permille 0..1000  */
    PKT_TYPE_CMD_SET_DECIM    = 0x84U,  /* payload: uint8 keep-1-in-N, 1..16 */
    PKT_TYPE_CMD_LIDAR_ENABLE = 0x85U   /* payload: uint8 0 = stop, 1 = scan */
} pkt_type_t;

/* Bit flags carried in both the orientation and status payloads. */
#define PKT_FLAG_IMU_OK          (1U << 0)
#define PKT_FLAG_BIAS_VALID      (1U << 1)
#define PKT_FLAG_USB_CONFIGURED  (1U << 2)
#define PKT_FLAG_CALIBRATING     (1U << 3)
#define PKT_FLAG_STALL_RECOVERED (1U << 4)
#define PKT_FLAG_LIDAR_OK        (1U << 5)
#define PKT_FLAG_LIDAR_SCANNING  (1U << 6)
#define PKT_FLAG_SCAN_TRUNCATED  (1U << 7)

/* Status codes carried in PKT_TYPE_STATUS. */
typedef enum {
    PKT_STATUS_BOOT              = 0U,
    PKT_STATUS_IMU_OK            = 1U,
    PKT_STATUS_IMU_WHO_AM_I_FAIL = 2U,
    PKT_STATUS_IMU_NO_RESPONSE   = 3U,
    PKT_STATUS_CALIB_START       = 4U,
    PKT_STATUS_CALIB_DONE        = 5U,
    PKT_STATUS_IMU_STALL         = 6U,
    PKT_STATUS_IMU_RECOVERED     = 7U,
    PKT_STATUS_PONG              = 8U,
    PKT_STATUS_BETA_CHANGED      = 9U,
    PKT_STATUS_LIDAR_OK          = 10U,
    PKT_STATUS_LIDAR_NO_RESPONSE = 11U,
    PKT_STATUS_LIDAR_BAD_HEALTH  = 12U,
    PKT_STATUS_LIDAR_BAD_DESC    = 13U,
    PKT_STATUS_LIDAR_STALL       = 14U,
    PKT_STATUS_LIDAR_RECOVERED   = 15U,
    PKT_STATUS_MOTOR_CHANGED     = 16U,
    PKT_STATUS_DECIM_CHANGED     = 17U,
    PKT_STATUS_LIDAR_STOPPED     = 18U
} pkt_status_code_t;

/* ---- Payload sizes, asserted against the builders in packet.c ----------- */
#define PKT_ORIENTATION_PAYLOAD_LEN  60U
#define PKT_STATUS_PAYLOAD_LEN       44U
#define PKT_STATUS_MSG_LEN           32U

/* The scan payload is a fixed 40-byte header followed by 4 bytes per point,
 * so its length is variable and the host reads point_count to size it. */
#define PKT_SCAN_HEADER_LEN          40U
#define PKT_SCAN_POINT_LEN           4U

/* Host-visible frame sizes, quoted in PROTOCOL.md. */
#define PKT_ORIENTATION_FRAME_LEN    (PKT_OVERHEAD + PKT_ORIENTATION_PAYLOAD_LEN) /* 68 */
#define PKT_STATUS_FRAME_LEN         (PKT_OVERHEAD + PKT_STATUS_PAYLOAD_LEN)      /* 52 */

/* --------------------------------------------------------------------------
 * In-memory representations.  These are ordinary structs -- they are NOT the
 * wire layout and are never memcpy'd onto the link.  Serialisation goes
 * through the builders below, byte by byte, so the wire format is independent
 * of the compiler's padding and of the CPU's endianness.
 * ------------------------------------------------------------------------ */
typedef struct {
    uint32_t t_us;            /* device microsecond timestamp of the sample  */
    float    q[4];            /* q0=w, q1=x, q2=y, q3=z (unit quaternion)    */
    float    accel_g[3];      /* scaled accelerometer, g                     */
    float    gyro_dps[3];     /* scaled gyroscope, deg/s, bias removed       */
    float    temp_c;          /* IMU die temperature                         */
    uint32_t sample_count;    /* IMU samples since boot                      */
    uint32_t drop_count;      /* packets discarded by the USB ring buffer    */
    uint16_t imu_odr_hz;      /* configured IMU output data rate             */
    uint8_t  flags;           /* PKT_FLAG_*                                  */
} pkt_orientation_t;

typedef struct {
    uint32_t t_us;
    uint32_t uptime_ms;
    uint8_t  code;            /* pkt_status_code_t                           */
    uint8_t  who_am_i;        /* last WHO_AM_I byte read from the IMU        */
    uint8_t  flags;           /* PKT_FLAG_*                                  */
    char     msg[PKT_STATUS_MSG_LEN];  /* NUL-padded ASCII, not NUL-required */
} pkt_status_t;

/* --------------------------------------------------------------------------
 * LiDAR scan packet: one complete revolution.
 *
 * The quaternion is the board's orientation at the instant the revolution
 * closed.  Nothing in this milestone uses it -- the host draws the ring in
 * the sensor's own 2D frame -- but pairing it here, at the only place where
 * both timestamps are known exactly, is what makes the later 3D stage a
 * matter of applying a rotation rather than re-deriving the association.
 * ------------------------------------------------------------------------ */
typedef struct {
    uint32_t t_us;             /* device time the revolution closed          */
    uint32_t seq;              /* revolutions since scanning started         */
    float    q[4];             /* orientation at t_us (w, x, y, z)           */
    float    rot_hz;           /* measured spindle rate                      */
    float    sample_hz;        /* measured points/s BEFORE decimation        */
    uint16_t point_count;      /* points that follow                         */
    uint16_t motor_permille;   /* PWM duty currently commanded               */
    uint8_t  decimation;       /* keep-1-in-N currently applied              */
    uint8_t  flags;            /* PKT_FLAG_*                                 */
    uint16_t reserved;         /* zero; keeps the header 4-byte aligned      */
} pkt_scan_header_t;

/* CRC-16/CCITT-FALSE: polynomial 0x1021, init 0xFFFF, no reflection,
 * no final XOR.  (Same parameters as the "CRC-16/IBM-3740" alias.) */
uint16_t pkt_crc16(const uint8_t *data, uint16_t len);

/* Build a complete frame into `buf`, which must be at least PKT_MAX_FRAME
 * bytes.  Returns the number of bytes written. */
uint16_t pkt_build_orientation(uint8_t *buf, const pkt_orientation_t *o);
uint16_t pkt_build_status(uint8_t *buf, const pkt_status_t *s);

/* Build a scan frame.  `pts` is 2 * h->point_count uint16 values, laid out
 * as (angle_q6, dist_mm) pairs -- which is exactly the memory layout of the
 * driver's point array.  `buf` must be at least PKT_MAX_FRAME bytes.
 * Returns 0 if point_count would overflow PKT_MAX_PAYLOAD. */
uint16_t pkt_build_scan(uint8_t *buf, const pkt_scan_header_t *h,
                        const uint16_t *pts);

/* Fill a pkt_status_t message field from a C string, NUL-padding the rest. */
void pkt_set_msg(pkt_status_t *s, const char *text);

#endif /* PACKET_H */
