/* ---------------------------------------------------------------------------
 * usb_cdc.c -- bare-metal USB CDC-ACM device stack for the STM32F411 OTG_FS.
 *
 * This is the highest-risk file in the project: it is the only part that has
 * to satisfy an external state machine (the host's USB stack) rather than
 * just our own code.  It is therefore deliberately minimal -- one
 * configuration, one interface pair, four endpoints, no composite devices, no
 * remote wakeup, no DMA -- and every non-obvious register write cites the
 * relevant section of RM0383 (STM32F411 reference manual) or of the USB 2.0 /
 * CDC 1.2 specifications.
 *
 * ===========================================================================
 * ENDPOINT AND FIFO LAYOUT
 * ===========================================================================
 * Endpoints:
 *   EP0  IN/OUT  control     64 bytes
 *   EP1  IN      bulk        64 bytes   telemetry frames, device -> host
 *   EP1  OUT     bulk        64 bytes   commands, host -> device
 *   EP2  IN      interrupt    8 bytes   CDC notifications (required by the
 *                                       class descriptor, never actually used)
 *
 * The OTG_FS core on the F411 has 1.25 KiB of packet RAM = 320 32-bit words,
 * partitioned once at init (RM0383 22.16.x, "FIFO RAM allocation"):
 *
 *   words   0..127   receive FIFO (shared by all OUT endpoints)
 *   words 128..191   EP0 IN transmit FIFO   (64 words = 256 bytes)
 *   words 192..255   EP1 IN transmit FIFO   (64 words = 256 bytes)
 *   words 256..271   EP2 IN transmit FIFO   (16 words =  64 bytes)
 *                    (48 words left spare)
 *
 * The 128-word RX FIFO is far more than the RM0383 minimum for this endpoint
 * set (about 32 words); the spare RAM has no other use here.
 *
 * ===========================================================================
 * VBUS SENSING IS DISABLED -- THIS IS A HARDWARE FACT, NOT A SHORTCUT
 * ===========================================================================
 * The OTG_FS VBUS sense input is PA9.  On this board PA9 is wired to the
 * LiDAR UART (USART1_TX) instead, so the core would never see a valid VBUS
 * and would refuse to attach.  GCCFG.NOVBUSSENS makes the core treat VBUS as
 * permanently valid, which is correct for a bus-powered device that can only
 * be powered when VBUS is present anyway.
 * ------------------------------------------------------------------------- */

#include "usb_cdc.h"
#include "board.h"
#include "packet.h"
#include "pkt_ring.h"
#include "tim_us.h"
#include "util.h"

/* ===========================================================================
 * Register access helpers
 * ======================================================================== */
#define USBx           USB_OTG_FS
#define USBx_DEVICE    ((USB_OTG_DeviceTypeDef *)((uint32_t)USBx + USB_OTG_DEVICE_BASE))
#define USBx_INEP(i)   ((USB_OTG_INEndpointTypeDef *) \
                        ((uint32_t)USBx + USB_OTG_IN_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define USBx_OUTEP(i)  ((USB_OTG_OUTEndpointTypeDef *) \
                        ((uint32_t)USBx + USB_OTG_OUT_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define USBx_DFIFO(i)  (*(volatile uint32_t *) \
                        ((uint32_t)USBx + USB_OTG_FIFO_BASE + ((i) * USB_OTG_FIFO_SIZE)))
#define USBx_PCGCCTL   (*(volatile uint32_t *)((uint32_t)USBx + USB_OTG_PCGCCTL_BASE))

/* Bit positions that the CMSIS header only exposes as masks. */
#define TSIZ_PKTCNT_SHIFT    19U
#define TSIZ_STUPCNT_SHIFT   29U
#define EPCTL_TXFNUM_SHIFT   22U
#define EPCTL_EPTYP_SHIFT    18U

#define EPTYP_CONTROL        0U
#define EPTYP_ISOCHRONOUS    1U
#define EPTYP_BULK           2U
#define EPTYP_INTERRUPT      3U

/* GRXSTSP.PKTSTS values in device mode (RM0383 22.16.1). */
#define PKTSTS_GLOBAL_OUT_NAK   0x1U
#define PKTSTS_OUT_DATA         0x2U
#define PKTSTS_OUT_COMPLETE     0x3U
#define PKTSTS_SETUP_COMPLETE   0x4U
#define PKTSTS_SETUP_DATA       0x6U

/* ===========================================================================
 * Protocol constants
 * ======================================================================== */
#define EP0_MPS          64U
#define EP_BULK_MPS      64U
#define EP_NOTIFY_MPS     8U

#define EP_BULK_IN       1U      /* address 0x81 */
#define EP_BULK_OUT      1U      /* address 0x01 */
#define EP_NOTIFY_IN     2U      /* address 0x82 */

/* Standard request codes (USB 2.0 table 9-4). */
#define REQ_GET_STATUS         0x00U
#define REQ_CLEAR_FEATURE      0x01U
#define REQ_SET_FEATURE        0x03U
#define REQ_SET_ADDRESS        0x05U
#define REQ_GET_DESCRIPTOR     0x06U
#define REQ_SET_DESCRIPTOR     0x07U
#define REQ_GET_CONFIGURATION  0x08U
#define REQ_SET_CONFIGURATION  0x09U
#define REQ_GET_INTERFACE      0x0AU
#define REQ_SET_INTERFACE      0x0BU

/* CDC class requests (CDC 1.2, PSTN subclass table 13). */
#define CDC_SET_LINE_CODING          0x20U
#define CDC_GET_LINE_CODING          0x21U
#define CDC_SET_CONTROL_LINE_STATE   0x22U
#define CDC_SEND_BREAK               0x23U

#define DESC_DEVICE         1U
#define DESC_CONFIGURATION  2U
#define DESC_STRING         3U

/* Largest single IN transfer we hand to the core.  Three 68-byte orientation
 * frames fit; the EP1 TX FIFO is 256 bytes, so the whole staging buffer can be
 * pushed in one go without waiting on FIFO space.  192 = 3 x 64, so every USB
 * packet in a transfer is full.
 *
 * This is NOT a limit on protocol frame size: pkt_ring_pop() hands out a byte
 * stream, so a frame larger than this (a LiDAR ring is ~1.5 kB) is simply
 * drained across several transfers. */
#define MAX_IN_XFER  192U

/* ===========================================================================
 * Descriptors
 * ======================================================================== */

/* Device descriptor.
 *
 * VID 0x0483 / PID 0x5740 is STMicroelectronics' "Virtual COM Port" pair,
 * which is what every STM32 CDC example ships with.  It is fine for a bench
 * project and for anything that never leaves the workshop; a product would
 * need its own vendor ID. */
static const uint8_t k_device_desc[18] = {
    18,                  /* bLength                                          */
    DESC_DEVICE,         /* bDescriptorType                                  */
    0x00, 0x02,          /* bcdUSB          = 2.00                           */
    0x02,                /* bDeviceClass    = CDC (communications)           */
    0x00,                /* bDeviceSubClass                                  */
    0x00,                /* bDeviceProtocol                                  */
    EP0_MPS,             /* bMaxPacketSize0 = 64                             */
    0x83, 0x04,          /* idVendor        = 0x0483                         */
    0x40, 0x57,          /* idProduct       = 0x5740                         */
    0x00, 0x01,          /* bcdDevice       = 1.00                           */
    0x01,                /* iManufacturer                                    */
    0x02,                /* iProduct                                         */
    0x03,                /* iSerialNumber                                    */
    0x01                 /* bNumConfigurations                               */
};

/* Configuration descriptor: 9 + 9 + 5 + 5 + 4 + 5 + 7 + 9 + 7 + 7 = 67 bytes. */
#define CONFIG_DESC_LEN  67U
static const uint8_t k_config_desc[CONFIG_DESC_LEN] = {
    /* ---- Configuration ------------------------------------------------- */
    9, DESC_CONFIGURATION,
    (uint8_t)(CONFIG_DESC_LEN & 0xFFU), (uint8_t)(CONFIG_DESC_LEN >> 8),
    2,                   /* bNumInterfaces  = comm + data                    */
    1,                   /* bConfigurationValue                              */
    0,                   /* iConfiguration                                   */
    0x80,                /* bmAttributes: bus powered, no remote wakeup      */
    125,                 /* bMaxPower = 250 mA (headroom for the LiDAR later)*/

    /* ---- Interface 0: CDC communication class -------------------------- */
    9, 0x04,
    0,                   /* bInterfaceNumber                                 */
    0,                   /* bAlternateSetting                                */
    1,                   /* bNumEndpoints (the notification endpoint)        */
    0x02,                /* bInterfaceClass    = CDC                         */
    0x02,                /* bInterfaceSubClass = Abstract Control Model      */
    0x01,                /* bInterfaceProtocol = AT commands (V.25ter)       */
    0,                   /* iInterface                                       */

    /* CDC header functional descriptor, bcdCDC = 1.10 */
    5, 0x24, 0x00, 0x10, 0x01,
    /* CDC call management: no call management, data interface 1 */
    5, 0x24, 0x01, 0x00, 0x01,
    /* CDC ACM: bmCapabilities 0x02 = supports Set/Get_Line_Coding,
     * Set_Control_Line_State and Serial_State.  macOS checks this. */
    4, 0x24, 0x02, 0x02,
    /* CDC union: control interface 0, subordinate interface 1 */
    5, 0x24, 0x06, 0x00, 0x01,

    /* Notification endpoint 0x82, interrupt IN, 8 bytes, 255 ms interval.
     * Declared because the class requires it; this firmware never sends on
     * it, and the host does not need it to open the port. */
    7, 0x05, 0x82, 0x03, EP_NOTIFY_MPS, 0x00, 0xFF,

    /* ---- Interface 1: CDC data class ----------------------------------- */
    9, 0x04,
    1,                   /* bInterfaceNumber                                 */
    0,                   /* bAlternateSetting                                */
    2,                   /* bNumEndpoints                                    */
    0x0A,                /* bInterfaceClass = CDC data                       */
    0x00, 0x00,          /* subclass / protocol                              */
    0,                   /* iInterface                                       */

    /* Bulk OUT 0x01, 64 bytes */
    7, 0x05, 0x01, 0x02, EP_BULK_MPS, 0x00, 0x00,
    /* Bulk IN  0x81, 64 bytes */
    7, 0x05, 0x81, 0x02, EP_BULK_MPS, 0x00, 0x00
};

/* String descriptor 0: supported languages (0x0409 = English, US). */
static const uint8_t k_string_langid[4] = { 4, DESC_STRING, 0x09, 0x04 };

static const char k_str_manufacturer[] = "JEly613";
static const char k_str_product[]      = "Mini-LiDAR-Board IMU";

/* Scratch space for the UTF-16 string descriptors built at request time. */
static uint8_t s_string_buf[64];

/* ===========================================================================
 * Driver state
 * ======================================================================== */
typedef enum {
    EP0_IDLE = 0,
    EP0_DATA_IN,
    EP0_DATA_OUT
} ep0_stage_t;

static volatile bool     s_configured;
static volatile bool     s_dtr;            /* host asserted DTR              */
static ep0_stage_t       s_ep0_stage;
static uint8_t           s_setup[8];
static uint8_t           s_pending_address;
static bool              s_ep0_zlp_pending;

static uint8_t           s_ep0_out_buf[EP0_MPS];
static uint16_t          s_ep0_out_len;
static uint8_t           s_ep0_out_request;   /* bRequest of the OUT stage   */

static uint8_t           s_cdc_rx_buf[EP_BULK_MPS];
static uint16_t          s_cdc_rx_len;
static usb_cdc_rx_cb_t   s_rx_cb;

static volatile bool     s_ep1_in_busy;
static uint8_t           s_tx_stage[MAX_IN_XFER];

/* CDC line coding: 115200 8N1.  Meaningless for a USB-native link, but the
 * host will ask for it and expects a plausible answer. */
static uint8_t s_line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 };

/* ===========================================================================
 * Low-level FIFO access
 * ======================================================================== */

/* Push `len` bytes into endpoint `ep`'s transmit FIFO.  The core always
 * transfers whole 32-bit words; a trailing partial word is padded, and the
 * byte count programmed in DIEPTSIZ is what actually goes on the wire. */
static void fifo_write(uint32_t ep, const uint8_t *src, uint16_t len)
{
    const uint32_t words = ((uint32_t)len + 3U) / 4U;

    for (uint32_t i = 0U; i < words; i++) {
        uint32_t w = 0U;

        for (uint32_t b = 0U; b < 4U; b++) {
            const uint32_t idx = (i * 4U) + b;
            if (idx < len) {
                w |= (uint32_t)src[idx] << (8U * b);
            }
        }
        USBx_DFIFO(ep) = w;
    }
}

/* Pop `len` bytes out of the shared receive FIFO into `dst` (which may be
 * NULL to discard).  Reads always happen through the endpoint-0 FIFO window;
 * in device mode there is only one RX FIFO. */
static void fifo_read(uint8_t *dst, uint16_t len)
{
    const uint32_t words = ((uint32_t)len + 3U) / 4U;

    for (uint32_t i = 0U; i < words; i++) {
        const uint32_t w = USBx_DFIFO(0U);

        if (dst != 0) {
            for (uint32_t b = 0U; b < 4U; b++) {
                const uint32_t idx = (i * 4U) + b;
                if (idx < len) {
                    dst[idx] = (uint8_t)((w >> (8U * b)) & 0xFFU);
                }
            }
        }
    }
}

static void flush_tx_fifo(uint32_t fifo_num)
{
    USBx->GRSTCTL = USB_OTG_GRSTCTL_TXFFLSH | (fifo_num << 6U);
    while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) != 0U) {
        /* self-clearing */
    }
}

static void flush_rx_fifo(void)
{
    USBx->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;
    while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) != 0U) {
        /* self-clearing */
    }
}

/* ===========================================================================
 * Endpoint 0 (control) plumbing
 * ======================================================================== */

/* Arm EP0 OUT so the core will accept the next SETUP packet (and any control
 * OUT data or status packet).  STUPCNT = 3 lets the core absorb up to three
 * back-to-back SETUPs, which is what the USB spec requires a device to
 * tolerate. */
static void ep0_arm_out(uint16_t len)
{
    USBx_OUTEP(0)->DOEPTSIZ = (3UL << TSIZ_STUPCNT_SHIFT)
                            | (1UL << TSIZ_PKTCNT_SHIFT)
                            | (uint32_t)len;
    USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
}

static void ep0_arm_setup(void)
{
    s_ep0_stage = EP0_IDLE;
    ep0_arm_out(24U);   /* room for 3 SETUP packets */
}

/* Send up to 256 bytes on EP0 IN.  The core splits the transfer into
 * wMaxPacketSize chunks itself as long as PKTCNT is right; everything we ever
 * send from EP0 (67-byte configuration descriptor at most) fits in the
 * 256-byte EP0 TX FIFO, so the whole payload can be written in one shot. */
static void ep0_send(const uint8_t *data, uint16_t len)
{
    if (len == 0U) {
        /* Zero-length packet: status stage, or the terminator for a transfer
         * whose length was an exact multiple of the packet size. */
        USBx_INEP(0)->DIEPTSIZ = (1UL << TSIZ_PKTCNT_SHIFT);
        USBx_INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
        return;
    }

    const uint32_t pktcnt = ((uint32_t)len + (EP0_MPS - 1U)) / EP0_MPS;

    USBx_INEP(0)->DIEPTSIZ = (pktcnt << TSIZ_PKTCNT_SHIFT) | (uint32_t)len;
    USBx_INEP(0)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
    fifo_write(0U, data, len);
}

/* Reply to a control-IN request, truncated to what the host asked for.  If we
 * return exactly a multiple of the packet size but fewer bytes than
 * requested, USB requires a zero-length packet to end the transfer. */
static void ep0_reply(const uint8_t *data, uint16_t len, uint16_t wLength)
{
    if (len > wLength) {
        len = wLength;
    }
    s_ep0_zlp_pending = ((len < wLength) && (len != 0U) && ((len % EP0_MPS) == 0U));
    s_ep0_stage = EP0_DATA_IN;
    ep0_send(data, len);
}

static void ep0_status_ack(void)
{
    s_ep0_stage = EP0_IDLE;
    s_ep0_zlp_pending = false;
    ep0_send(0, 0U);
}

static void ep0_stall(void)
{
    USBx_INEP(0)->DIEPCTL  |= USB_OTG_DIEPCTL_STALL;
    USBx_OUTEP(0)->DOEPCTL |= USB_OTG_DOEPCTL_STALL;
    ep0_arm_setup();
}

/* Build a USB string descriptor (UTF-16LE) from an ASCII C string. */
static uint16_t make_string_desc(const char *ascii, uint8_t *out, uint16_t out_size)
{
    const uint16_t n = (uint16_t)strlen(ascii);
    uint16_t       len = (uint16_t)(2U + (n * 2U));

    if (len > out_size) {
        len = (uint16_t)(out_size & ~1U);
    }
    out[0] = (uint8_t)len;
    out[1] = DESC_STRING;
    for (uint16_t i = 0U; ((2U + (i * 2U) + 1U) < len) && (i < n); i++) {
        out[2U + (i * 2U)]      = (uint8_t)ascii[i];
        out[2U + (i * 2U) + 1U] = 0x00U;
    }
    return len;
}

/* Serial number: the 96-bit factory unique ID rendered as 24 hex digits, so
 * two boards plugged into the same host get distinct /dev nodes. */
static uint16_t make_serial_desc(uint8_t *out, uint16_t out_size)
{
    static const char hex[] = "0123456789ABCDEF";
    char              text[25];
    const uint32_t   *uid = (const uint32_t *)BOARD_UID_BASE;

    for (uint32_t w = 0U; w < 3U; w++) {
        const uint32_t v = uid[w];
        for (uint32_t n = 0U; n < 8U; n++) {
            text[(w * 8U) + n] = hex[(v >> (28U - (4U * n))) & 0xFU];
        }
    }
    text[24] = '\0';
    return make_string_desc(text, out, out_size);
}

/* ===========================================================================
 * Endpoint configuration
 * ======================================================================== */
static void ep_open_in(uint32_t ep, uint32_t mps, uint32_t type, uint32_t txfifo)
{
    if ((USBx_INEP(ep)->DIEPCTL & USB_OTG_DIEPCTL_USBAEP) == 0U) {
        USBx_INEP(ep)->DIEPCTL |= mps
                                | (type << EPCTL_EPTYP_SHIFT)
                                | (txfifo << EPCTL_TXFNUM_SHIFT)
                                | USB_OTG_DIEPCTL_SD0PID_SEVNFRM
                                | USB_OTG_DIEPCTL_USBAEP;
    }
    USBx_DEVICE->DAINTMSK |= (1UL << ep);
}

static void ep_open_out(uint32_t ep, uint32_t mps, uint32_t type)
{
    if ((USBx_OUTEP(ep)->DOEPCTL & USB_OTG_DOEPCTL_USBAEP) == 0U) {
        USBx_OUTEP(ep)->DOEPCTL |= mps
                                 | (type << EPCTL_EPTYP_SHIFT)
                                 | USB_OTG_DOEPCTL_SD0PID_SEVNFRM
                                 | USB_OTG_DOEPCTL_USBAEP;
    }
    USBx_DEVICE->DAINTMSK |= (1UL << (ep + 16U));
}

/* Arm the bulk OUT endpoint for one 64-byte packet from the host. */
static void ep_bulk_out_arm(void)
{
    s_cdc_rx_len = 0U;
    USBx_OUTEP(EP_BULK_OUT)->DOEPTSIZ = (1UL << TSIZ_PKTCNT_SHIFT) | EP_BULK_MPS;
    USBx_OUTEP(EP_BULK_OUT)->DOEPCTL |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
}

/* ===========================================================================
 * Bulk IN transmit path
 * ======================================================================== */

/* Start an IN transfer if the endpoint is idle and the ring has data.
 * Must be called with interrupts disabled or from the USB ISR. */
static void tx_try_start(void)
{
    if (!s_configured || s_ep1_in_busy) {
        return;
    }

    const uint16_t n = pkt_ring_pop(s_tx_stage, MAX_IN_XFER);

    if (n == 0U) {
        return;
    }

    s_ep1_in_busy = true;

    const uint32_t pktcnt = ((uint32_t)n + (EP_BULK_MPS - 1U)) / EP_BULK_MPS;

    USBx_INEP(EP_BULK_IN)->DIEPTSIZ = (pktcnt << TSIZ_PKTCNT_SHIFT) | (uint32_t)n;
    USBx_INEP(EP_BULK_IN)->DIEPCTL |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;
    fifo_write(EP_BULK_IN, s_tx_stage, n);

    /* Note on zero-length terminators: a bulk transfer whose length is an
     * exact multiple of 64 needs a ZLP so the host knows it ended.  We size
     * every transfer from whole protocol frames, and the frame sizes in
     * PROTOCOL.md (68 and 52 bytes) mean the host-side reader is a byte
     * stream reassembled by the sync/CRC parser rather than a message
     * boundary consumer -- so a missing ZLP only ever delays the tail by one
     * packet time, never loses it.  The next frame's transfer flushes it. */
}

/* ===========================================================================
 * Standard and class request handling
 * ======================================================================== */
static void handle_get_descriptor(uint16_t wValue, uint16_t wLength)
{
    const uint8_t type  = (uint8_t)(wValue >> 8);
    const uint8_t index = (uint8_t)(wValue & 0xFFU);

    switch (type) {
    case DESC_DEVICE:
        ep0_reply(k_device_desc, (uint16_t)sizeof(k_device_desc), wLength);
        break;

    case DESC_CONFIGURATION:
        ep0_reply(k_config_desc, CONFIG_DESC_LEN, wLength);
        break;

    case DESC_STRING:
        switch (index) {
        case 0U:
            ep0_reply(k_string_langid, (uint16_t)sizeof(k_string_langid), wLength);
            break;
        case 1U: {
            const uint16_t n = make_string_desc(k_str_manufacturer, s_string_buf,
                                                (uint16_t)sizeof(s_string_buf));
            ep0_reply(s_string_buf, n, wLength);
            break;
        }
        case 2U: {
            const uint16_t n = make_string_desc(k_str_product, s_string_buf,
                                                (uint16_t)sizeof(s_string_buf));
            ep0_reply(s_string_buf, n, wLength);
            break;
        }
        case 3U: {
            const uint16_t n = make_serial_desc(s_string_buf,
                                                (uint16_t)sizeof(s_string_buf));
            ep0_reply(s_string_buf, n, wLength);
            break;
        }
        default:
            ep0_stall();
            break;
        }
        break;

    default:
        /* Device qualifier, other-speed configuration, BOS, ...  A full-speed
         * only USB 2.0 device is required to STALL these, and hosts treat the
         * stall as the expected answer. */
        ep0_stall();
        break;
    }
}

static void handle_set_configuration(uint8_t value)
{
    if (value == 0U) {
        s_configured = false;
        ep0_status_ack();
        return;
    }
    if (value != 1U) {
        ep0_stall();
        return;
    }

    ep_open_in(EP_BULK_IN,   EP_BULK_MPS,   EPTYP_BULK,      1U);
    ep_open_in(EP_NOTIFY_IN, EP_NOTIFY_MPS, EPTYP_INTERRUPT, 2U);
    ep_open_out(EP_BULK_OUT, EP_BULK_MPS,   EPTYP_BULK);
    ep_bulk_out_arm();

    s_ep1_in_busy = false;
    s_configured  = true;
    ep0_status_ack();
}

static void process_setup(void)
{
    const uint8_t  bmRequestType = s_setup[0];
    const uint8_t  bRequest      = s_setup[1];
    const uint16_t wValue        = (uint16_t)(s_setup[2] | ((uint16_t)s_setup[3] << 8));
    const uint16_t wLength       = (uint16_t)(s_setup[6] | ((uint16_t)s_setup[7] << 8));
    const uint8_t  req_type      = (uint8_t)(bmRequestType & 0x60U);  /* 0=std, 0x20=class */
    const bool     dev_to_host   = ((bmRequestType & 0x80U) != 0U);

    s_ep0_zlp_pending = false;

    /* Host-to-device request WITH a data stage: we must collect the data
     * before we can act on it.  Park the request and arm EP0 OUT. */
    if (!dev_to_host && (wLength != 0U)) {
        if (wLength > sizeof(s_ep0_out_buf)) {
            ep0_stall();
            return;
        }
        s_ep0_stage       = EP0_DATA_OUT;
        s_ep0_out_len     = wLength;
        s_ep0_out_request = bRequest;
        ep0_arm_out(wLength);
        return;
    }

    if (req_type == 0x00U) {
        /* ---- Standard device requests ----------------------------------- */
        switch (bRequest) {
        case REQ_GET_DESCRIPTOR:
            handle_get_descriptor(wValue, wLength);
            break;

        case REQ_SET_ADDRESS:
            /* RM0383 22.17.5: the address must be programmed BEFORE the
             * status stage completes, otherwise the core answers the next
             * transaction at the old address. */
            s_pending_address = (uint8_t)(wValue & 0x7FU);
            USBx_DEVICE->DCFG = (USBx_DEVICE->DCFG & ~USB_OTG_DCFG_DAD)
                              | ((uint32_t)s_pending_address << 4U);
            ep0_status_ack();
            break;

        case REQ_SET_CONFIGURATION:
            handle_set_configuration((uint8_t)(wValue & 0xFFU));
            break;

        case REQ_GET_CONFIGURATION: {
            const uint8_t cfg = s_configured ? 1U : 0U;
            ep0_reply(&cfg, 1U, wLength);
            break;
        }

        case REQ_GET_STATUS: {
            /* Bus powered, no remote wakeup, no endpoint halted. */
            static const uint8_t status[2] = { 0x00U, 0x00U };
            ep0_reply(status, 2U, wLength);
            break;
        }

        case REQ_GET_INTERFACE: {
            const uint8_t alt = 0U;
            ep0_reply(&alt, 1U, wLength);
            break;
        }

        case REQ_SET_INTERFACE:
            /* Only alternate setting 0 exists; accept it, reject anything else. */
            if ((wValue & 0xFFU) == 0U) {
                ep0_status_ack();
            } else {
                ep0_stall();
            }
            break;

        case REQ_CLEAR_FEATURE:
        case REQ_SET_FEATURE:
            /* The only feature we could meaningfully implement is
             * ENDPOINT_HALT.  Nothing in this design ever halts an endpoint,
             * so acknowledge and move on. */
            ep0_status_ack();
            break;

        default:
            ep0_stall();
            break;
        }
    } else if (req_type == 0x20U) {
        /* ---- CDC class requests ----------------------------------------- */
        switch (bRequest) {
        case CDC_GET_LINE_CODING:
            ep0_reply(s_line_coding, (uint16_t)sizeof(s_line_coding), wLength);
            break;

        case CDC_SET_CONTROL_LINE_STATE:
            /* wValue bit 0 = DTR, bit 1 = RTS.  Tracked purely for
             * diagnostics; the firmware streams regardless of DTR so that
             * `cat /dev/cu.usbmodem*` works without any port setup. */
            s_dtr = ((wValue & 0x0001U) != 0U);
            ep0_status_ack();
            break;

        case CDC_SEND_BREAK:
            ep0_status_ack();
            break;

        default:
            ep0_stall();
            break;
        }
    } else {
        ep0_stall();
    }
}

/* A control OUT data stage finished; act on the bytes we collected. */
static void process_ep0_out_data(void)
{
    switch (s_ep0_out_request) {
    case CDC_SET_LINE_CODING:
        if (s_ep0_out_len >= sizeof(s_line_coding)) {
            memcpy(s_line_coding, s_ep0_out_buf, sizeof(s_line_coding));
        }
        ep0_status_ack();
        break;

    default:
        /* Unknown host-to-device request: accept and ignore rather than
         * stalling mid-transfer, which some hosts handle poorly. */
        ep0_status_ack();
        break;
    }
}

/* ===========================================================================
 * Interrupt sub-handlers
 * ======================================================================== */
static void on_usb_reset(void)
{
    USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;

    flush_tx_fifo(0x10U);   /* 0x10 = flush all transmit FIFOs */

    for (uint32_t i = 0U; i < 4U; i++) {
        USBx_INEP(i)->DIEPINT  = 0xFB7FU;   /* write-1-to-clear, all bits */
        USBx_OUTEP(i)->DOEPINT = 0xFB7FU;
    }

    /* Only EP0 is active until the host configures us. */
    USBx_DEVICE->DAINTMSK = (1UL << 0) | (1UL << 16);
    USBx_DEVICE->DOEPMSK  = USB_OTG_DOEPMSK_STUPM | USB_OTG_DOEPMSK_XFRCM;
    USBx_DEVICE->DIEPMSK  = USB_OTG_DIEPMSK_XFRCM | USB_OTG_DIEPMSK_TOM;

    /* Back to the default address. */
    USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;

    s_configured  = false;
    s_ep1_in_busy = false;
    s_dtr         = false;

    ep0_arm_setup();
}

static void on_enum_done(void)
{
    /* Full speed is the only possibility on this core, so the enumerated
     * speed is not interesting; what matters is setting EP0's maximum packet
     * size, which DIEPCTL0.MPSIZ encodes as 00 = 64 bytes. */
    USBx_INEP(0)->DIEPCTL &= ~USB_OTG_DIEPCTL_MPSIZ;
    USBx_DEVICE->DCTL |= USB_OTG_DCTL_CGINAK;
}

static void on_rxflvl(void)
{
    /* Mask the interrupt while draining, as RM0383 instructs, so a new packet
     * arriving mid-pop cannot re-enter this handler. */
    USBx->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;

    const uint32_t sts    = USBx->GRXSTSP;
    const uint32_t epnum  = sts & USB_OTG_GRXSTSP_EPNUM;
    const uint32_t bcnt   = (sts & USB_OTG_GRXSTSP_BCNT_Msk) >> USB_OTG_GRXSTSP_BCNT_Pos;
    const uint32_t pktsts = (sts & USB_OTG_GRXSTSP_PKTSTS_Msk) >> USB_OTG_GRXSTSP_PKTSTS_Pos;

    switch (pktsts) {
    case PKTSTS_SETUP_DATA:
        /* Always 8 bytes; anything else is a bus error, so drop it. */
        if (bcnt == 8U) {
            fifo_read(s_setup, 8U);
        } else {
            fifo_read(0, (uint16_t)bcnt);
        }
        break;

    case PKTSTS_OUT_DATA:
        if (epnum == 0U) {
            const uint16_t n = (bcnt > sizeof(s_ep0_out_buf))
                             ? (uint16_t)sizeof(s_ep0_out_buf)
                             : (uint16_t)bcnt;
            fifo_read(s_ep0_out_buf, n);
            if (bcnt > n) {
                fifo_read(0, (uint16_t)(bcnt - n));
            }
        } else if (epnum == EP_BULK_OUT) {
            const uint16_t n = (bcnt > sizeof(s_cdc_rx_buf))
                             ? (uint16_t)sizeof(s_cdc_rx_buf)
                             : (uint16_t)bcnt;
            fifo_read(s_cdc_rx_buf, n);
            if (bcnt > n) {
                fifo_read(0, (uint16_t)(bcnt - n));
            }
            s_cdc_rx_len = n;
        } else {
            fifo_read(0, (uint16_t)bcnt);
        }
        break;

    default:
        /* Transfer-complete and NAK-effective statuses carry no payload. */
        if (bcnt != 0U) {
            fifo_read(0, (uint16_t)bcnt);
        }
        break;
    }

    USBx->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
}

static void on_out_endpoint_int(void)
{
    const uint32_t pending = (USBx_DEVICE->DAINT & USBx_DEVICE->DAINTMSK) >> 16U;

    for (uint32_t ep = 0U; ep < 4U; ep++) {
        if ((pending & (1UL << ep)) == 0U) {
            continue;
        }
        const uint32_t doepint = USBx_OUTEP(ep)->DOEPINT;
        const bool     is_setup = ((doepint & USB_OTG_DOEPINT_STUP) != 0U);

        if ((doepint & USB_OTG_DOEPINT_XFRC) != 0U) {
            USBx_OUTEP(ep)->DOEPINT = USB_OTG_DOEPINT_XFRC;

            /* A SETUP transaction raises XFRC and STUP together.  Re-arming
             * the endpoint here would leave EPENA set while process_setup()
             * tries to reprogram DOEPTSIZ for a control OUT data stage, so
             * when STUP is pending we let the SETUP path below do the arming. */
            if (ep == 0U) {
                if (!is_setup) {
                    if (s_ep0_stage == EP0_DATA_OUT) {
                        process_ep0_out_data();
                    } else {
                        /* Status-stage ZLP from the host: transfer complete. */
                        ep0_arm_setup();
                    }
                }
            } else if (ep == EP_BULK_OUT) {
                if ((s_cdc_rx_len > 0U) && (s_rx_cb != 0)) {
                    s_rx_cb(s_cdc_rx_buf, s_cdc_rx_len);
                }
                ep_bulk_out_arm();
            } else {
                /* no other OUT endpoints exist */
            }
        }

        if (is_setup) {
            USBx_OUTEP(ep)->DOEPINT = USB_OTG_DOEPINT_STUP;
            if (ep == 0U) {
                process_setup();
            }
        }
    }
}

static void on_in_endpoint_int(void)
{
    const uint32_t pending = USBx_DEVICE->DAINT & USBx_DEVICE->DAINTMSK & 0xFFFFU;

    for (uint32_t ep = 0U; ep < 4U; ep++) {
        if ((pending & (1UL << ep)) == 0U) {
            continue;
        }
        const uint32_t diepint = USBx_INEP(ep)->DIEPINT;

        if ((diepint & USB_OTG_DIEPINT_XFRC) != 0U) {
            USBx_INEP(ep)->DIEPINT = USB_OTG_DIEPINT_XFRC;

            if (ep == 0U) {
                if (s_ep0_zlp_pending) {
                    s_ep0_zlp_pending = false;
                    ep0_send(0, 0U);
                } else if (s_ep0_stage == EP0_DATA_IN) {
                    /* Data stage finished; the host now sends a status OUT. */
                    ep0_arm_setup();
                } else {
                    ep0_arm_setup();
                }
            } else if (ep == EP_BULK_IN) {
                s_ep1_in_busy = false;
                tx_try_start();     /* keep the pipe full back-to-back */
            } else {
                /* notification endpoint: never used */
            }
        }

        if ((diepint & USB_OTG_DIEPINT_TOC) != 0U) {
            USBx_INEP(ep)->DIEPINT = USB_OTG_DIEPINT_TOC;
        }
    }
}

/* ===========================================================================
 * The one USB interrupt
 * ======================================================================== */
void OTG_FS_IRQHandler(void);
void OTG_FS_IRQHandler(void)
{
    /* In device mode the host-mode interrupt bits are meaningless; masking
     * with GINTMSK also stops us reacting to sources we never enabled. */
    const uint32_t gintsts = USBx->GINTSTS & USBx->GINTMSK;

    if (gintsts == 0U) {
        return;
    }

    if ((gintsts & USB_OTG_GINTSTS_USBRST) != 0U) {
        USBx->GINTSTS = USB_OTG_GINTSTS_USBRST;
        on_usb_reset();
    }
    if ((gintsts & USB_OTG_GINTSTS_ENUMDNE) != 0U) {
        USBx->GINTSTS = USB_OTG_GINTSTS_ENUMDNE;
        on_enum_done();
    }
    if ((gintsts & USB_OTG_GINTSTS_RXFLVL) != 0U) {
        /* RXFLVL is not write-1-to-clear: it clears when the FIFO drains. */
        on_rxflvl();
    }
    if ((gintsts & USB_OTG_GINTSTS_OEPINT) != 0U) {
        on_out_endpoint_int();
    }
    if ((gintsts & USB_OTG_GINTSTS_IEPINT) != 0U) {
        on_in_endpoint_int();
    }
    if ((gintsts & USB_OTG_GINTSTS_USBSUSP) != 0U) {
        USBx->GINTSTS = USB_OTG_GINTSTS_USBSUSP;
        s_configured = false;
    }
    if ((gintsts & USB_OTG_GINTSTS_WKUINT) != 0U) {
        USBx->GINTSTS = USB_OTG_GINTSTS_WKUINT;
    }
    if ((gintsts & USB_OTG_GINTSTS_SOF) != 0U) {
        USBx->GINTSTS = USB_OTG_GINTSTS_SOF;
    }
}

/* ===========================================================================
 * Public API
 * ======================================================================== */
static void usb_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;

    /* PA11 (DM) and PA12 (DP): alternate function 10, very high speed, no
     * pull.  The internal 1.5k pull-up on DP is inside the OTG PHY and is
     * controlled by the core, not by GPIO. */
    const uint32_t pins[2] = { USB_DM_PIN, USB_DP_PIN };

    for (uint32_t i = 0U; i < 2U; i++) {
        const uint32_t p = pins[i];

        GPIOA->MODER   = (GPIOA->MODER   & ~(3UL << (p * 2U))) | (2UL << (p * 2U));
        GPIOA->OTYPER &= ~(1UL << p);
        GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3UL << (p * 2U))) | (3UL << (p * 2U));
        GPIOA->PUPDR  &= ~(3UL << (p * 2U));
        GPIOA->AFR[1]  = (GPIOA->AFR[1] & ~(0xFUL << ((p - 8U) * 4U)))
                       | (10UL << ((p - 8U) * 4U));
    }
}

void usb_cdc_init(void)
{
    usb_gpio_init();

    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    (void)RCC->AHB2ENR;

    /* ---- Core reset ----------------------------------------------------
     * Select the embedded full-speed transceiver first: RM0383 requires a
     * core soft reset after changing PHYSEL. */
    USBx->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

    while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U) {
        /* wait for the AHB master to go idle before resetting */
    }
    USBx->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
    while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_CSRST) != 0U) {
        /* self-clearing when the reset completes */
    }

    /* ---- Transceiver power and VBUS sensing ------------------------------
     * PWRDWN = 1 actually means "transceiver active" (the bit is named for
     * the power-down condition it releases).  NOVBUSSENS makes the core treat
     * VBUS as permanently valid -- mandatory here, see the file header. */
    USBx->GCCFG = USB_OTG_GCCFG_PWRDWN | USB_OTG_GCCFG_NOVBUSSENS;
    USBx->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;

    /* ---- Force device mode ----------------------------------------------
     * TRDT is the USB turnaround time in PHY clocks; RM0383 tabulates 0x6 for
     * an AHB frequency of 32 MHz and above (ours is 96 MHz).  The core needs
     * up to 25 ms to settle after a mode change. */
    USBx->GUSBCFG = (USBx->GUSBCFG & ~USB_OTG_GUSBCFG_TRDT)
                  | USB_OTG_GUSBCFG_FDMOD
                  | USB_OTG_GUSBCFG_PHYSEL
                  | (6UL << USB_OTG_GUSBCFG_TRDT_Pos);
    delay_ms(25U);

    /* ---- Device configuration ------------------------------------------- */
    USBx_PCGCCTL = 0U;                      /* no clock gating, no PHY suspend */
    USBx_DEVICE->DCFG = 3UL;                /* DSPD = 11b: full speed          */

    /* ---- FIFO partitioning (see the map in the file header) -------------- */
    USBx->GRXFSIZ = 128U;
    USBx->DIEPTXF0_HNPTXFSIZ = (64UL << 16) | 128UL;   /* EP0 IN */
    USBx->DIEPTXF[0]         = (64UL << 16) | 192UL;   /* EP1 IN */
    USBx->DIEPTXF[1]         = (16UL << 16) | 256UL;   /* EP2 IN */

    flush_tx_fifo(0x10U);
    flush_rx_fifo();

    /* ---- Put every endpoint into a known disabled state ------------------ */
    for (uint32_t i = 0U; i < 4U; i++) {
        if ((USBx_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) != 0U) {
            USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
        } else {
            USBx_INEP(i)->DIEPCTL = 0U;
        }
        USBx_INEP(i)->DIEPTSIZ = 0U;
        USBx_INEP(i)->DIEPINT  = 0xFB7FU;

        if ((USBx_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) != 0U) {
            USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
        } else {
            USBx_OUTEP(i)->DOEPCTL = 0U;
        }
        USBx_OUTEP(i)->DOEPTSIZ = 0U;
        USBx_OUTEP(i)->DOEPINT  = 0xFB7FU;
    }

    USBx_DEVICE->DIEPMSK  = 0U;
    USBx_DEVICE->DOEPMSK  = 0U;
    USBx_DEVICE->DAINTMSK = 0U;

    /* ---- Interrupts ------------------------------------------------------ */
    USBx->GINTSTS = 0xFFFFFFFFUL;       /* discard anything latched so far */
    USBx->GINTMSK = USB_OTG_GINTMSK_USBRST
                  | USB_OTG_GINTMSK_ENUMDNEM
                  | USB_OTG_GINTMSK_RXFLVLM
                  | USB_OTG_GINTMSK_IEPINT
                  | USB_OTG_GINTMSK_OEPINT
                  | USB_OTG_GINTMSK_USBSUSPM
                  | USB_OTG_GINTMSK_WUIM;

    USBx->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

    /* Priority 3: strictly below the IMU sample path (priority 1), so a USB
     * transfer can never delay a data-ready burst. */
    NVIC_SetPriority(OTG_FS_IRQn, 3U);
    NVIC_EnableIRQ(OTG_FS_IRQn);

    /* ---- Attach to the bus ----------------------------------------------- */
    USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;   /* clear soft disconnect */
}

void usb_cdc_set_rx_callback(usb_cdc_rx_cb_t cb)
{
    s_rx_cb = cb;
}

bool usb_cdc_configured(void)
{
    return s_configured;
}

bool usb_cdc_send_frame(const uint8_t *frame, uint16_t len)
{
    if ((len == 0U) || (len > PKT_MAX_FRAME)) {
        return false;
    }
    if (!pkt_ring_push(frame, len)) {
        return false;
    }
    usb_cdc_poll();
    return true;
}

uint32_t usb_cdc_tx_drops(void)
{
    return pkt_ring_drops();
}

void usb_cdc_poll(void)
{
    /* tx_try_start() touches the ring and the endpoint registers, both of
     * which the USB ISR also touches, so this short critical section is what
     * keeps the two callers from interleaving. */
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    tx_try_start();
    __set_PRIMASK(primask);
}
