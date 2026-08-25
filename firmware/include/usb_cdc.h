/* ---------------------------------------------------------------------------
 * usb_cdc.h -- bare-metal USB CDC-ACM device on the STM32F411 OTG_FS core.
 *
 * The board exposes a single USB-C connector.  Plugging it into a Mac gives a
 * /dev/cu.usbmodem* serial device with no driver install; the host software
 * opens that port and reads the binary frames defined in PROTOCOL.md.
 * ------------------------------------------------------------------------- */

#ifndef USB_CDC_H
#define USB_CDC_H

#include <stdbool.h>
#include <stdint.h>

/* Called from the USB interrupt when the host sends bytes on the bulk OUT
 * endpoint.  `data` is only valid for the duration of the call. */
typedef void (*usb_cdc_rx_cb_t)(const uint8_t *data, uint16_t len);

/* Configure PA11/PA12, bring up OTG_FS in device mode and attach to the bus. */
void usb_cdc_init(void);

/* Register the host->device command handler (optional). */
void usb_cdc_set_rx_callback(usb_cdc_rx_cb_t cb);

/* True once the host has completed enumeration and issued
 * SET_CONFIGURATION(1).  Until then, queued frames simply pile up in the
 * ring buffer and the oldest are discarded. */
bool usb_cdc_configured(void);

/* Queue one complete protocol frame for transmission.  Never blocks; on a
 * full ring the oldest frames are dropped and the drop counter advances.
 * Returns false only for a malformed (over-long) frame. */
bool usb_cdc_send_frame(const uint8_t *frame, uint16_t len);

/* Frames discarded by ring-buffer overflow since boot. */
uint32_t usb_cdc_tx_drops(void);

/* Give the driver a chance to start a transfer from thread context.  The IN
 * endpoint interrupt normally keeps the pipe busy on its own; this covers the
 * case where the ring went empty and the pipe went idle. */
void usb_cdc_poll(void);

#endif /* USB_CDC_H */
