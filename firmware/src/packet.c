/* ---------------------------------------------------------------------------
 * packet.c -- wire-format serialisers.
 *
 * Everything is written one byte at a time in explicit little-endian order.
 * That is slightly more code than a packed-struct memcpy, but it means:
 *   - no reliance on __attribute__((packed)) and its alignment traps,
 *   - no hidden dependency on the CPU's byte order,
 *   - the code reads in the same order as the table in PROTOCOL.md.
 * At 100 packets per second the cost is irrelevant.
 * ------------------------------------------------------------------------- */

#include "packet.h"
#include "util.h"

/* Compile-time cross-checks: if a payload layout below ever stops matching
 * the length constants the host parser uses, the build breaks here rather
 * than producing frames the host silently rejects. */
STATIC_ASSERT(PKT_ORIENTATION_PAYLOAD_LEN == 60U, "orientation payload size");
STATIC_ASSERT(PKT_STATUS_PAYLOAD_LEN == 44U,      "status payload size");
STATIC_ASSERT(PKT_ORIENTATION_FRAME_LEN <= PKT_MAX_FRAME, "frame fits buffer");
STATIC_ASSERT(PKT_SCAN_HEADER_LEN == 40U,          "scan header size");

uint16_t pkt_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((uint16_t)(crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ---- Little-endian primitive writers ------------------------------------ */

static uint16_t put_u8(uint8_t *buf, uint16_t off, uint8_t v)
{
    buf[off] = v;
    return off + 1U;
}

static uint16_t put_u16(uint8_t *buf, uint16_t off, uint16_t v)
{
    buf[off + 0U] = (uint8_t)(v & 0xFFU);
    buf[off + 1U] = (uint8_t)((v >> 8) & 0xFFU);
    return off + 2U;
}

static uint16_t put_u32(uint8_t *buf, uint16_t off, uint32_t v)
{
    buf[off + 0U] = (uint8_t)(v & 0xFFU);
    buf[off + 1U] = (uint8_t)((v >> 8)  & 0xFFU);
    buf[off + 2U] = (uint8_t)((v >> 16) & 0xFFU);
    buf[off + 3U] = (uint8_t)((v >> 24) & 0xFFU);
    return off + 4U;
}

static uint16_t put_f32(uint8_t *buf, uint16_t off, float v)
{
    /* Reinterpret the float's bits without type-punning through a pointer
     * cast (which would violate strict aliasing).  A union is the portable,
     * warning-free way to do this in C. */
    union { float f; uint32_t u; } conv;

    conv.f = v;
    return put_u32(buf, off, conv.u);
}

/* Write the 6-byte header and return the offset of the payload. */
static uint16_t put_header(uint8_t *buf, uint8_t type, uint16_t payload_len)
{
    uint16_t off = 0U;

    off = put_u8(buf, off, PKT_SYNC0);
    off = put_u8(buf, off, PKT_SYNC1);
    off = put_u8(buf, off, PKT_VERSION);
    off = put_u8(buf, off, type);
    off = put_u16(buf, off, payload_len);
    return off;
}

/* Append the CRC over bytes [2 .. off-1] and return the total frame length. */
static uint16_t put_trailer(uint8_t *buf, uint16_t off)
{
    const uint16_t crc = pkt_crc16(&buf[2], (uint16_t)(off - 2U));

    return put_u16(buf, off, crc);
}

uint16_t pkt_build_orientation(uint8_t *buf, const pkt_orientation_t *o)
{
    uint16_t off = put_header(buf, (uint8_t)PKT_TYPE_ORIENTATION,
                              PKT_ORIENTATION_PAYLOAD_LEN);

    off = put_u32(buf, off, o->t_us);            /* +0  */
    off = put_f32(buf, off, o->q[0]);            /* +4  */
    off = put_f32(buf, off, o->q[1]);            /* +8  */
    off = put_f32(buf, off, o->q[2]);            /* +12 */
    off = put_f32(buf, off, o->q[3]);            /* +16 */
    off = put_f32(buf, off, o->accel_g[0]);      /* +20 */
    off = put_f32(buf, off, o->accel_g[1]);      /* +24 */
    off = put_f32(buf, off, o->accel_g[2]);      /* +28 */
    off = put_f32(buf, off, o->gyro_dps[0]);     /* +32 */
    off = put_f32(buf, off, o->gyro_dps[1]);     /* +36 */
    off = put_f32(buf, off, o->gyro_dps[2]);     /* +40 */
    off = put_f32(buf, off, o->temp_c);          /* +44 */
    off = put_u32(buf, off, o->sample_count);    /* +48 */
    off = put_u32(buf, off, o->drop_count);      /* +52 */
    off = put_u16(buf, off, o->imu_odr_hz);      /* +56 */
    off = put_u8(buf,  off, o->flags);           /* +58 */
    off = put_u8(buf,  off, 0U);                 /* +59 reserved, must be 0 */

    return put_trailer(buf, off);
}

uint16_t pkt_build_status(uint8_t *buf, const pkt_status_t *s)
{
    uint16_t off = put_header(buf, (uint8_t)PKT_TYPE_STATUS,
                              PKT_STATUS_PAYLOAD_LEN);

    off = put_u32(buf, off, s->t_us);            /* +0  */
    off = put_u32(buf, off, s->uptime_ms);       /* +4  */
    off = put_u8(buf,  off, s->code);            /* +8  */
    off = put_u8(buf,  off, s->who_am_i);        /* +9  */
    off = put_u8(buf,  off, s->flags);           /* +10 */
    off = put_u8(buf,  off, 0U);                 /* +11 reserved, must be 0 */

    for (uint16_t i = 0U; i < PKT_STATUS_MSG_LEN; i++) {
        off = put_u8(buf, off, (uint8_t)s->msg[i]);   /* +12 .. +43 */
    }

    return put_trailer(buf, off);
}

void pkt_set_msg(pkt_status_t *s, const char *text)
{
    uint16_t i = 0U;

    memset(s->msg, 0, sizeof(s->msg));
    while ((i < PKT_STATUS_MSG_LEN) && (text[i] != '\0')) {
        s->msg[i] = text[i];
        i++;
    }
}

/* ---------------------------------------------------------------------------
 * Scan frame: a 40-byte header followed by point_count x (angle_q6, dist_mm),
 * every field little-endian.  See PROTOCOL.md.
 * ------------------------------------------------------------------------- */
uint16_t pkt_build_scan(uint8_t *buf, const pkt_scan_header_t *h,
                        const uint16_t *pts)
{
    const uint32_t payload_len =
        (uint32_t)PKT_SCAN_HEADER_LEN
      + ((uint32_t)h->point_count * (uint32_t)PKT_SCAN_POINT_LEN);

    /* Refuse rather than overrun: the caller decimates or truncates upstream,
     * so reaching this is a programming error, not a runtime condition. */
    if (payload_len > (uint32_t)PKT_MAX_PAYLOAD) {
        return 0U;
    }

    uint16_t off = put_header(buf, (uint8_t)PKT_TYPE_LIDAR_SCAN,
                              (uint16_t)payload_len);

    off = put_u32(buf, off, h->t_us);
    off = put_u32(buf, off, h->seq);
    off = put_f32(buf, off, h->q[0]);
    off = put_f32(buf, off, h->q[1]);
    off = put_f32(buf, off, h->q[2]);
    off = put_f32(buf, off, h->q[3]);
    off = put_f32(buf, off, h->rot_hz);
    off = put_f32(buf, off, h->sample_hz);
    off = put_u16(buf, off, h->point_count);
    off = put_u16(buf, off, h->motor_permille);
    off = put_u8 (buf, off, h->decimation);
    off = put_u8 (buf, off, h->flags);
    off = put_u16(buf, off, 0U);              /* reserved                    */

    for (uint16_t i = 0U; i < h->point_count; i++) {
        off = put_u16(buf, off, pts[(uint32_t)i * 2U]);        /* angle_q6   */
        off = put_u16(buf, off, pts[((uint32_t)i * 2U) + 1U]); /* dist_mm    */
    }

    return put_trailer(buf, off);
}
