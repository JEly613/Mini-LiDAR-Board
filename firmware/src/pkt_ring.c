/* ---------------------------------------------------------------------------
 * pkt_ring.c -- see pkt_ring.h for the design rationale.
 * ------------------------------------------------------------------------- */

#include "pkt_ring.h"
#include "board.h"
#include "util.h"

static uint8_t  s_bytes[PKT_RING_BYTES];
static uint16_t s_len[PKT_RING_FRAMES];

static uint32_t s_byte_head;    /* next write position in s_bytes           */
static uint32_t s_byte_tail;    /* next read  position in s_bytes           */
static uint32_t s_byte_used;    /* bytes currently occupied                 */

static uint32_t s_frame_head;   /* next write slot in s_len                 */
static uint32_t s_frame_tail;   /* next read  slot in s_len                 */
static uint32_t s_frame_used;   /* frames currently queued                  */

static uint32_t s_drops;

/* Bytes of the oldest queued frame that pkt_ring_pop() has already handed
 * out.  A frame larger than one USB transfer is drained across several calls,
 * so the tail can sit part-way through a frame.  While that is true the frame
 * MUST NOT be discarded -- see pkt_ring_push(). */
static uint16_t s_head_consumed;

void pkt_ring_init(void)
{
    s_byte_head  = 0U;
    s_byte_tail  = 0U;
    s_byte_used  = 0U;
    s_frame_head = 0U;
    s_frame_tail = 0U;
    s_frame_used = 0U;
    s_drops      = 0U;
    s_head_consumed = 0U;
}

/* Remove the oldest frame.  Caller must hold the critical section and must
 * have checked that at least one frame is queued. */
static void discard_oldest(void)
{
    /* Only the not-yet-popped remainder is still occupying the byte array. */
    const uint16_t remaining =
        (uint16_t)(s_len[s_frame_tail] - s_head_consumed);

    s_frame_tail = (s_frame_tail + 1U) % PKT_RING_FRAMES;
    s_frame_used--;
    s_byte_tail  = (s_byte_tail + remaining) % PKT_RING_BYTES;
    s_byte_used -= remaining;
    s_head_consumed = 0U;

    if (s_drops != 0xFFFFFFFFUL) {
        s_drops++;
    }
}

bool pkt_ring_push(const uint8_t *frame, uint16_t len)
{
    if ((len == 0U) || ((uint32_t)len > PKT_RING_BYTES)) {
        return false;
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* Make room: drop whole frames from the front until this one fits, both
     * in the byte array and in the length queue. */
    while (((PKT_RING_BYTES - s_byte_used) < (uint32_t)len)
           || (s_frame_used >= PKT_RING_FRAMES)) {
        if (s_frame_used == 0U) {
            /* Cannot happen given the size check above, but bail rather than
             * spin forever if the invariants are ever broken. */
            __set_PRIMASK(primask);
            return false;
        }

        /* The normal overflow policy is "drop the oldest", because fresh
         * telemetry is worth more than stale.  There is exactly one
         * exception: if the oldest frame is already part-way out of the USB
         * endpoint, discarding it would leave the host holding a fragment.
         * In that narrow case we drop the INCOMING frame instead.  It costs
         * one packet and it guarantees the host never has to resynchronise
         * out of a truncation we caused ourselves. */
        if (s_head_consumed != 0U) {
            if (s_drops != 0xFFFFFFFFUL) {
                s_drops++;
            }
            __set_PRIMASK(primask);
            return true;
        }
        discard_oldest();
    }

    /* The byte array is circular, so a frame may straddle the wrap point. */
    const uint32_t first = (PKT_RING_BYTES - s_byte_head < len)
                         ? (PKT_RING_BYTES - s_byte_head)
                         : len;
    memcpy(&s_bytes[s_byte_head], frame, first);
    if (first < len) {
        memcpy(&s_bytes[0], &frame[first], (uint32_t)len - first);
    }
    s_byte_head  = (s_byte_head + len) % PKT_RING_BYTES;
    s_byte_used += len;

    s_len[s_frame_head] = len;
    s_frame_head = (s_frame_head + 1U) % PKT_RING_FRAMES;
    s_frame_used++;

    __set_PRIMASK(primask);
    return true;
}

uint16_t pkt_ring_pop(uint8_t *dst, uint16_t max_bytes)
{
    uint16_t copied = 0U;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* Fill `dst` as a byte stream, spanning as many frames as fit and leaving
     * the last one part-consumed if it does not.
     *
     * This used to stop on frame boundaries, which quietly capped the maximum
     * frame size at max_bytes: a LiDAR ring (~1.5 kB) is far larger than one
     * USB transfer (MAX_IN_XFER), so it could never be popped at all and it
     * wedged the pipe until it aged out of the ring.  USB CDC is a byte
     * stream and the host parser frames on sync bytes plus CRC, so splitting
     * a frame across transfers is invisible to it. */
    while ((s_frame_used > 0U) && (copied < max_bytes)) {
        const uint16_t len       = s_len[s_frame_tail];
        const uint16_t remaining = (uint16_t)(len - s_head_consumed);
        uint16_t       take      = (uint16_t)(max_bytes - copied);

        if (take > remaining) {
            take = remaining;
        }

        const uint32_t first = (PKT_RING_BYTES - s_byte_tail < take)
                             ? (PKT_RING_BYTES - s_byte_tail)
                             : take;
        memcpy(&dst[copied], &s_bytes[s_byte_tail], first);
        if (first < take) {
            memcpy(&dst[copied + first], &s_bytes[0], (uint32_t)take - first);
        }

        s_byte_tail  = (s_byte_tail + take) % PKT_RING_BYTES;
        s_byte_used -= take;
        copied       = (uint16_t)(copied + take);
        s_head_consumed = (uint16_t)(s_head_consumed + take);

        if (s_head_consumed == len) {        /* frame fully handed over */
            s_frame_tail    = (s_frame_tail + 1U) % PKT_RING_FRAMES;
            s_frame_used--;
            s_head_consumed = 0U;
        }
    }

    __set_PRIMASK(primask);
    return copied;
}

uint32_t pkt_ring_drops(void)   { return s_drops; }
uint32_t pkt_ring_pending(void) { return s_frame_used; }
