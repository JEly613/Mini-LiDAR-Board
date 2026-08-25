/* ---------------------------------------------------------------------------
 * pkt_ring.h -- frame-granular ring buffer between the producer (main loop)
 * and the USB IN endpoint interrupt that drains it.
 *
 * Why not a plain byte FIFO: the overflow policy is "drop the oldest".  On a
 * byte FIFO that would chop a frame in half, and although the host parser
 * resynchronises on the sync bytes and validates the CRC, feeding it
 * deliberate garbage is a bad habit.  This ring therefore stores whole frames:
 * a byte array for the payload plus a small circular array of frame lengths,
 * so the discard step always removes an entire frame.
 *
 * Push and drop stay frame-granular so an overflow never truncates a frame in
 * the middle; only pop is a byte stream, because the USB endpoint below it is
 * one too.
 *
 * Concurrency: pkt_ring_push() is called from the main loop, pkt_ring_pop()
 * from the OTG_FS interrupt.  Both take a very short critical section rather
 * than relying on lock-free index tricks, because the two indices and the
 * length queue must move together.
 * ------------------------------------------------------------------------- */

#ifndef PKT_RING_H
#define PKT_RING_H

#include <stdbool.h>
#include <stdint.h>

#define PKT_RING_BYTES   16384U  /* a few 4 KB scan frames plus orientation  */
#define PKT_RING_FRAMES  64U     /* power of two, >= BYTES / smallest frame  */

void pkt_ring_init(void);

/* Enqueue one complete frame.  If there is not enough room, the oldest
 * frames are discarded until it fits (and the drop counter increases by the
 * number discarded).  Returns false only if the frame is larger than the
 * whole ring, which would be a programming error. */
bool pkt_ring_push(const uint8_t *frame, uint16_t len);

/* Copy up to `max_bytes` into `dst` as a BYTE STREAM, removing what it takes
 * from the ring.  Frames larger than `max_bytes` are handed out across
 * successive calls -- necessary because a LiDAR ring is several times larger
 * than one USB transfer.  Returns the number of bytes copied. */
uint16_t pkt_ring_pop(uint8_t *dst, uint16_t max_bytes);

/* Number of frames dropped due to overflow since boot (saturating at
 * UINT32_MAX, which will never be reached in practice). */
uint32_t pkt_ring_drops(void);

/* Frames currently queued. */
uint32_t pkt_ring_pending(void);

#endif /* PKT_RING_H */
