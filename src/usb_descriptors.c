/* usb_descriptors.c — FreeBSD style, long lines allowed */

#include "tusb.h"

#define USB_VID 0xCAFE
#define USB_PID 0x4001
#define USB_BCD 0x0200

enum {
	ITF_NUM_CDC_0 = 0,
	ITF_NUM_CDC_0_DATA,
	ITF_NUM_VENDOR,
	ITF_NUM_TOTAL
};

#define EPNUM_CDC_0_NOTIF	0x81	/* EP1 IN  (interrupt) */
#define EPNUM_CDC_0_OUT		0x02	/* EP2 OUT (bulk)      */
#define EPNUM_CDC_0_IN		0x82	/* EP2 IN  (bulk)      */

#define EPNUM_VENDOR_OUT	0x03	/* EP3 OUT */
#define EPNUM_VENDOR_IN		0x83	/* EP3 IN  */

tusb_desc_device_t const desc_device = {
	.bLength = sizeof(tusb_desc_device_t),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = USB_BCD,
	.bDeviceClass = TUSB_CLASS_MISC,
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol = MISC_PROTOCOL_IAD,
	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
	.idVendor = USB_VID,
	.idProduct = USB_PID,
	.bcdDevice = 0x0100,

	.iManufacturer = 0x01,
	.iProduct = 0x02,
	.iSerialNumber = 0x03,

	.bNumConfigurations = 1
};

uint8_t const *
tud_descriptor_device_cb(void)
{
	return (uint8_t const *) &desc_device;
}

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN)

uint8_t const desc_configuration[] = {
	/* Configuration descriptor */
	TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

	/* CDC ACM interface (control + data) */
	TUD_CDC_DESCRIPTOR(
		ITF_NUM_CDC_0,		/* interface number */
		4,			/* string index     */
		EPNUM_CDC_0_NOTIF,	/* notification ep  */
		8,
		EPNUM_CDC_0_OUT,	/* OUT EP   */
		EPNUM_CDC_0_IN,		/* IN EP    */
		64
	),

	/* Vendor interface (bulk in/out) */
	TUD_VENDOR_DESCRIPTOR(
		ITF_NUM_VENDOR,		/* interface number */
		5,			/* string index     */
		EPNUM_VENDOR_OUT,
		EPNUM_VENDOR_IN,
		64
	)
};

uint8_t const *
tud_descriptor_configuration_cb(uint8_t index)
{
	(void) index;
	return desc_configuration;
}

#define DESC_STR_MAXLEN 32u
static uint16_t desc_str[DESC_STR_MAXLEN];

char const *string_desc_arr[] = {
	(const char[]){0x09, 0x04}, /* supported language: English */
	"MarkM",                    /* 1: Manufacturer */
	"8-bit Overlord",           /* 2: Product */
	"6809",                     /* 3: Serial number */
	"MC6809 Serial",            /* 4 */
	"MC6809 Bulk"               /* 5 */
};

uint16_t const *
tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	(void) langid;

	/* language descriptor */
	if (index == 0)
		return (uint16_t const *) string_desc_arr[0];

	if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
		return NULL;

	const char *str = string_desc_arr[index];
	size_t len = MIN(strlen(str), DESC_STR_MAXLEN - 1);
	desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 + (len * 2)));

	for (size_t i = 0; i < len; i++)
		desc_str[1 + i] = str[i];

	return desc_str;
}