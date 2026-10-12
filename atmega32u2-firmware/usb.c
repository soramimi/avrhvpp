// USB CDC-ACM for ATMEGA32U2
// Copyright (C) 2022 S.Fuchita (@soramimi_jp)

#define USB_SERIAL_PRIVATE_INCLUDE
#include <avr/io.h>
#include "usb.h"

void clear_buffers();

/**************************************************************************
 *
 *  Configurable Options
 *
 **************************************************************************/

// You can change these to give your code its own name.
#define STR_MANUFACTURER L"soramimi@soramimi.jp"
#define STR_PRODUCT L"CDC testing"

// Mac OS-X and Linux automatically load the correct drivers.  On
// Windows, even though the driver is supplied by Microsoft, an
// INF file is needed to load the driver.  These numbers need to
// match the INF file.
#define VENDOR_ID 0x16c0
#define PRODUCT_ID 0x05e1

// USB devices are supposed to implment a halt feature, which is
// rarely (if ever) used.  If you comment this line out, the halt
// code will be removed, saving 102 bytes of space (gcc 4.3.0).
// This is not strictly USB compliant, but works with all major
// operating systems.
#define SUPPORT_ENDPOINT_HALT

/**************************************************************************
 *
 *  Endpoint Buffer Configuration
 *
 **************************************************************************/

#define ENDPOINT0_SIZE 8

#define CDC_COMM_INTERFACE 0
#define COMM_IN_ENDPOINT 2

#define CDC_DATA_INTERFACE 1
#define DATA_OUT_ENDPOINT 3
#define DATA_IN_ENDPOINT 4

static const uint8_t PROGMEM endpoint_config_table[] = {
	COMM_IN_ENDPOINT, EP_TYPE_INTERRUPT_IN, EP_SIZE(COMM_EP_SIZE) | EP_DOUBLE_BUFFER,
	DATA_OUT_ENDPOINT, EP_TYPE_BULK_OUT, EP_SIZE(RX_EP_SIZE) | EP_DOUBLE_BUFFER,
	DATA_IN_ENDPOINT, EP_TYPE_BULK_IN, EP_SIZE(TX_EP_SIZE) | EP_DOUBLE_BUFFER,
	0,
};

/**************************************************************************
 *
 *  Descriptor Data
 *
 **************************************************************************/

// Descriptors are the data that your computer reads when it auto-detects
// this USB device (called "enumeration" in USB lingo).  The most commonly
// changed items are editable at the top of this file.  Changing things
// in here should only be done by those who've read chapter 9 of the USB
// spec and relevant portions of any USB class specifications!

static const uint8_t PROGMEM device_descriptor[] = {
	18, // bLength
	1, // bDescriptorType
	0x00, 0x02, // bcdUSB
	2, // bDeviceClass
	2, // bDeviceSubClass
	0, // bDeviceProtocol
	ENDPOINT0_SIZE, // bMaxPacketSize0
	LSB(VENDOR_ID), MSB(VENDOR_ID), // idVendor
	LSB(PRODUCT_ID), MSB(PRODUCT_ID), // idProduct
	0x00, 0x01, // bcdDevice
	0, // iManufacturer
	0, // iProduct
	0, // iSerialNumber
	1 // bNumConfigurations
};

#define CDC_COMM_IF_DESC_OFFSET (9)
#define CDC_DATA_IF_DESC_OFFSET (9 + 9 + 5 + 4 + 5 + 7)
PROGMEM const uint8_t config1_descriptor[] = {
	// USB configuration descriptor
	9, // sizeof(usbDescriptorConfiguration): length of descriptor in bytes
	2, // descriptor type: USBDESCR_CONFIG
	9 + 9 + 5 + 4 + 5 + 7 + 9 + 7 + 7, 0, // total length of data returned (including inlined descriptors)
	2, // number of interfaces in this configuration
	1, // index of this configuration
	0, // configuration name string index
#if USB_CFG_IS_SELF_POWERED
	(1 << 7) | USBATTR_SELFPOWER, // attributes
#else
	(1 << 7), // attributes
#endif
	100 / 2, // max USB current in 2mA units

	// comm interface

	// interface descriptor follows inline:
	9, // sizeof(usbDescrInterface): length of descriptor in bytes
	4, // USBDESCR_INTERFACE
	CDC_COMM_INTERFACE, // index of this interface
	0, // alternate setting for this interface
	1, // USB_CFG_HAVE_INTRIN_ENDPOINT
	2, // USB_CFG_INTERFACE_CLASS
	2, // USB_CFG_INTERFACE_SUBCLASS
	0, // USB_CFG_INTERFACE_PROTOCOL
	0, // string index for interface

	// CDC Header
	5,
	0x24,
	0,
	0x01, 0x10,

	// CDC ACM
	4,
	0x24,
	2,
	0x06,

	// CDC Union
	5,
	0x24,
	6,
	0,
	1,

	7, // sizeof(usbDescrEndpoint)
	5, // USBDESCR_ENDPOINT
	COMM_IN_ENDPOINT | 0x80, // IN endpoint
	0x03, // attrib: Interrupt endpoint
	COMM_EP_SIZE, 0, // maximum packet size
	64, // USB_CFG_INTR_POLL_INTERVAL

	// data interface

	9, // sizeof(usbDescrInterface): length of descriptor in bytes
	4, // USBDESCR_INTERFACE
	CDC_DATA_INTERFACE, // index of this interface
	0, // alternate setting for this interface
	2, // USB_CFG_HAVE_INTRIN_ENDPOINT
	0x0a, // USB_CFG_INTERFACE_CLASS
	0, // USB_CFG_INTERFACE_SUBCLASS
	0, // USB_CFG_INTERFACE_PROTOCOL,
	0, // string index for interface

	7, // sizeof(usbDescrEndpoint)
	5,
	DATA_OUT_ENDPOINT, // OUT
	0x02, // bulk
	RX_EP_SIZE, 0, // maximum packet size
	0, // USB_CFG_INTR_POLL_INTERVAL

	7, // sizeof(usbDescrEndpoint)
	5,
	DATA_IN_ENDPOINT | 0x80, // IN
	0x02, // bulk
	TX_EP_SIZE, 0, // maximum packet size
	0, // USB_CFG_INTR_POLL_INTERVAL
};

// If you're desperate for a little extra code memory, these strings
// can be completely removed if iManufacturer, iProduct, iSerialNumber
// in the device desciptor are changed to zeros.
struct usb_string_descriptor_struct {
	uint8_t bLength;
	uint8_t bDescriptorType;
	int16_t wString[];
};
static const struct usb_string_descriptor_struct PROGMEM string0 = {
	4,
	3,
	{ 0x0409 }
};
static const struct usb_string_descriptor_struct PROGMEM string1 = {
	sizeof(STR_MANUFACTURER),
	3,
	STR_MANUFACTURER
};
static const struct usb_string_descriptor_struct PROGMEM string2 = {
	sizeof(STR_PRODUCT),
	3,
	STR_PRODUCT
};

// This table defines which descriptor data is sent for each specific
// request from the host (in wValue and wIndex).
static struct descriptor_list_struct {
	uint16_t wValue;
	uint16_t wIndex;
	const uint8_t *addr;
	uint8_t length;
} PROGMEM const descriptor_list[] = {
	{ 0x0100, 0x0000, device_descriptor, sizeof(device_descriptor) },
	{ 0x0200, 0x0000, config1_descriptor, sizeof(config1_descriptor) },
	{ 0x2100, CDC_COMM_INTERFACE, config1_descriptor + CDC_COMM_IF_DESC_OFFSET, 9 },
	{ 0x2100, CDC_DATA_INTERFACE, config1_descriptor + CDC_DATA_IF_DESC_OFFSET, 9 },
	{ 0x0300, 0x0000, (const uint8_t *)&string0, 4 },
	{ 0x0301, 0x0409, (const uint8_t *)&string1, sizeof(STR_MANUFACTURER) },
	{ 0x0302, 0x0409, (const uint8_t *)&string2, sizeof(STR_PRODUCT) }
};
#define NUM_DESC_LIST (sizeof(descriptor_list) / sizeof(struct descriptor_list_struct))

/**************************************************************************
 *
 *  Variables - these are the only non-stack RAM usage
 *
 **************************************************************************/

static volatile uint8_t usb_configuration = 0;
static volatile uint8_t data_flush_timer = 0;
static uint8_t idle_count = 0;

/**************************************************************************
 *
 *  Public Functions - these are the API intended for the user
 *
 **************************************************************************/

// initialize USB
void usb_init()
{
	HW_CONFIG();
	USB_FREEZE(); // enable USB
	PLL_CONFIG(); // config PLL
	while (!(PLLCSR & (1 << PLOCK))); // wait for PLL lock
	USB_CONFIG(); // start USB clock
	UDCON = 0; // enable attach resistor
	usb_configuration = 0;
	clear_buffers();
	UDIEN = (1 << EORSTE) | (1 << SOFE);
	sei();
}

uint8_t is_usb_configured()
{
	return usb_configuration;
}

static inline void usb_release_tx()
{
	UEINTX = 0x3a; // FIFOCON=0 NAKINI=0 RWAL=1 NAKOUTI=1 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=0
}

static inline void usb_release_rx()
{
	UEINTX = 0x6b; // FIFOCON=0 NAKINI=1 RWAL=1 NAKOUTI=0 RXSTPI=1 RXOUTI=0 STALLEDI=1 TXINI=1
}

static void usb_flush_rx(uint8_t ep)
{
	UENUM = ep;
	if (UEBCLX != 0) {
		usb_release_rx();
	}
}

int8_t usb_send_to_host(uint8_t ep, uint8_t const *ptr, uint8_t len)
{
	int8_t r = -1;
	if (!usb_configuration) return r;
	uint8_t intr_state = SREG;
	cli();
	UENUM = ep;
	uint8_t timeout = UDFNUML + 50;
	while (1) {
		if (UEINTX & (1 << RWAL)) {
			for (uint8_t i = 0; i < len; i++) {
				UEDATX = ptr[i];
			}
			usb_release_tx();
			idle_count = 0;
			r = 0;
			break;
		}
		if (UDFNUML == timeout) break;
	}
	SREG = intr_state;
	return r;
}

void usb_data_tx(uint8_t const *ptr, uint8_t len)
{
	if (ptr && len > 0) {
		usb_send_to_host(DATA_IN_ENDPOINT, ptr, len);
	}
}

uint8_t usb_data_rx(uint8_t *ptr, uint8_t len)
{
	const uint8_t ep = DATA_OUT_ENDPOINT;
	uint8_t n = 0;
	uint8_t intr_state = SREG;
	cli();
	UENUM = ep;
	n = UEBCLX;
	if (n > len) {
		n = len;
	}
	for (uint8_t i = 0; i < n; i++) {
		ptr[i] = UEDATX;
	}
	if (n > 0 && UEBCLX == 0) {
		usb_release_rx();
	}
	SREG = intr_state;
	return n;
}

uint8_t usb_read_available_()
{
	if (!usb_configuration) return 0;
	const uint8_t ep = DATA_OUT_ENDPOINT;
	uint8_t intr_state = SREG;
	cli();
	UENUM = ep;
	uint8_t n = UEBCLX;
	SREG = intr_state;
	return n;
}

uint8_t usb_read_byte_()
{
	uint8_t c;
	usb_data_rx(&c, 1);
	return c;
}

/**************************************************************************
 *
 *  Private Functions - not intended for general user consumption....
 *
 **************************************************************************/

// USB Device Interrupt - handle all device-level events
// the transmit buffer flushing is triggered by the start of frame
//
void usb_gen_vect()
{
	uint8_t udint = UDINT;
	UDINT = 0;

	if (udint & (1 << EORSTI)) {
		UENUM = 0;
		UECONX = 1;
		UECFG0X = EP_TYPE_CONTROL;
		UECFG1X = EP_SIZE(ENDPOINT0_SIZE) | EP_SINGLE_BUFFER;
		UEIENX = (1 << RXSTPE);
		usb_configuration = 0;
	}

	if (udint & (1 << SOFI)) {
		usb_flush_rx(DATA_IN_ENDPOINT);
	}
}
ISR(USB_GEN_vect)
{
	usb_gen_vect();
}

// Misc functions to wait for ready and send/receive packets
static inline void usb_wait_in_ready()
{
	while (!(UEINTX & (1 << TXINI)));
}

static inline void usb_send_in()
{
	UEINTX = ~(1 << TXINI);
}

static inline void usb_wait_receive_out()
{
	while (!(UEINTX & (1 << RXOUTI)));
}

static inline void usb_ack_out()
{
	UEINTX = ~(1 << RXOUTI);
}

void usb_com_vect()
{
	uint8_t intbits;
	const uint8_t *list;
	uint8_t i, n, len;
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
	uint16_t desc_val;
	const uint8_t *desc_addr;
	uint8_t desc_length;

	UENUM = 0;
	intbits = UEINTX;
	if (intbits & (1 << RXSTPI)) {
		bmRequestType = UEDATX;
		bRequest = UEDATX;
		wValue = UEDATX;
		wValue |= (UEDATX << 8);
		wIndex = UEDATX;
		wIndex |= (UEDATX << 8);
		wLength = UEDATX;
		wLength |= (UEDATX << 8);
		UEINTX = ~((1 << RXSTPI) | (1 << RXOUTI) | (1 << TXINI));
		if (bRequest == GET_DESCRIPTOR) {
			list = (const uint8_t *)descriptor_list;
			for (i = 0;; i++) {
				if (i >= NUM_DESC_LIST) {
					UECONX = (1 << STALLRQ) | (1 << EPEN); // stall
					return;
				}
				desc_val = pgm_read_word(list);
				if (desc_val != wValue) {
					list += sizeof(struct descriptor_list_struct);
					continue;
				}
				list += 2;
				desc_val = pgm_read_word(list);
				if (desc_val != wIndex) {
					list += sizeof(struct descriptor_list_struct) - 2;
					continue;
				}
				list += 2;
				desc_addr = (const uint8_t *)pgm_read_word(list);
				list += 2;
				desc_length = pgm_read_byte(list);
				break;
			}
			len = (wLength < 256) ? wLength : 255;
			if (len > desc_length) len = desc_length;
			do {
				// wait for host ready for IN packet
				do {
					i = UEINTX;
				} while (!(i & ((1 << TXINI) | (1 << RXOUTI))));
				if (i & (1 << RXOUTI)) return; // abort
				// send IN packet
				n = len < ENDPOINT0_SIZE ? len : ENDPOINT0_SIZE;
				for (i = n; i; i--) {
					UEDATX = pgm_read_byte(desc_addr++);
				}
				len -= n;
				usb_send_in();
			} while (len || n == ENDPOINT0_SIZE);
			return;
		}
		if (bRequest == SET_ADDRESS) {
			usb_send_in();
			usb_wait_in_ready();
			UDADDR = wValue | (1 << ADDEN);
			return;
		}
		if (bRequest == SET_CONFIGURATION && bmRequestType == 0) {
			usb_configuration = wValue;
			usb_send_in();
			for (i = 0; i < 4; i++) {
				UENUM = i + 1;
				UECONX = 0;
			}
			const uint8_t *p = endpoint_config_table;
			while (1) {
				uint8_t ep = pgm_read_byte(p);
				if (!ep) break;
				p++;
				UENUM = ep;
				UECONX = 1 << EPEN;
				UECFG0X = pgm_read_byte(p++);
				UECFG1X = pgm_read_byte(p++);
			}
			UERST = 0x1E;
			UERST = 0;
			return;
		}
		if (bRequest == GET_CONFIGURATION && bmRequestType == 0x80) {
			usb_wait_in_ready();
			UEDATX = usb_configuration;
			usb_send_in();
			return;
		}

		if (bRequest == GET_STATUS) {
			usb_wait_in_ready();
			i = 0;
#ifdef SUPPORT_ENDPOINT_HALT
			if (bmRequestType == 0x82) {
				UENUM = wIndex;
				if (UECONX & (1 << STALLRQ)) i = 1;
				UENUM = 0;
			}
#endif
			UEDATX = i;
			UEDATX = 0;
			usb_send_in();
			return;
		}
#ifdef SUPPORT_ENDPOINT_HALT
		if ((bRequest == CLEAR_FEATURE || bRequest == SET_FEATURE)
			&& bmRequestType == 0x02 && wValue == 0) {
			i = wIndex & 0x7F;
			if (i >= 1 && i <= MAX_ENDPOINT) {
				usb_send_in();
				UENUM = i;
				if (bRequest == SET_FEATURE) {
					UECONX = (1 << STALLRQ) | (1 << EPEN);
				} else {
					UECONX = (1 << STALLRQC) | (1 << RSTDT) | (1 << EPEN);
					UERST = (1 << i);
					UERST = 0;
				}
				return;
			}
		}
#endif
		if (wIndex == CDC_COMM_INTERFACE) {
			static char line[7] = {0x00, 0x4b, 0x00, 0x00, 0, 0, 8}; // default: 19200bps
			if (bmRequestType == 0x21) { // recv from host
				if (bRequest == CDC_SET_LINE_CODING) {
					clear_buffers();
					usb_wait_receive_out();
					for (uint8_t i = 0; i < 7; i++) {
						line[i] = UEDATX;
					}
					usb_ack_out();
					usb_send_in();
					return;
				}
			}
			if (bmRequestType == 0xa1) { // send to host
				if (bRequest == CDC_GET_LINE_CODING) {
					len = 7;
					i = 0;
					while (i < len) {
						uint8_t ueintx = UEINTX;
						if (ueintx & (1 << RXOUTI)) return;
						if (ueintx & (1 << TXINI)) {
							n = len - i;
							if (n > ENDPOINT0_SIZE) {
								n = ENDPOINT0_SIZE;
							}
							while (n > 0) {
								UEDATX = line[i++];
								len--;
								n--;
							}
							usb_send_in();
						}
					}
					return;
				}
			}
			if (bmRequestType == CDC_SET_CONTROL_LINE_STATE) {
				return;
			}
		}
	}
	UECONX = (1 << STALLRQ) | (1 << EPEN); // stall
}
ISR(USB_COM_vect)
{
	usb_com_vect();
}
