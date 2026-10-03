/* usb_descriptors.c — FreeBSD style, long lines allowed */

#include "tusb.h"

#define USB_VID 0xCAFE
#define USB_PID 0x4001
#define USB_BCD 0x0200

enum {
	ITF_NUM_CDC_0 = 0,
	ITF_NUM_CDC_0_DATA,
	ITF_NUM_CDC_1,
	ITF_NUM_CDC_1_DATA,
	ITF_NUM_VENDOR,
	ITF_NUM_TOTAL
};

enum {
	STRID_LANGID = 0,
	STRID_MANUFACTURER,
	STRID_PRODUCT,
	STRID_SERIAL,
	STRID_CDC_0,
	STRID_CDC_1,
	STRID_VENDOR
};

#define EPNUM_CDC_0_NOTIF	0x81	/* EP1 IN  (interrupt) */
#define EPNUM_CDC_0_OUT		0x02	/* EP2 OUT (bulk)      */
#define EPNUM_CDC_0_IN		0x82	/* EP2 IN  (bulk)      */

#define EPNUM_CDC_1_NOTIF	0x83	/* EP3 IN  (interrupt) */
#define EPNUM_CDC_1_OUT		0x04	/* EP4 OUT (bulk)      */
#define EPNUM_CDC_1_IN		0x84	/* EP4 IN  (bulk)      */

#define EPNUM_VENDOR_OUT	0x05	/* EP5 OUT */
#define EPNUM_VENDOR_IN		0x85	/* EP5 IN  */

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

	.iManufacturer = STRID_MANUFACTURER,
	.iProduct = STRID_PRODUCT,
	.iSerialNumber = STRID_SERIAL,

	.bNumConfigurations = 1
};

uint8_t const *
tud_descriptor_device_cb(void)
{
	return (uint8_t const *) &desc_device;
}

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 2 * TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN)

uint8_t const desc_configuration[] = {
	/*
	 * Configuration descriptor.  Two of these arguments are not what
	 * reaches the wire: the macro ORs bit 7 into bmAttributes itself
	 * (bus powered, no remote wakeup), and it halves bMaxPower, which
	 * the spec encodes in 2 mA units - so 100 means 100 mA, sent as 50.
	 */
	TUD_CONFIG_DESCRIPTOR(
		1,			/* config number    */
		ITF_NUM_TOTAL,		/* interface count  */
		0,			/* no name string   */
		CONFIG_TOTAL_LEN,	/* total length     */
		0x00,			/* bmAttributes     */
		100			/* bMaxPower in mA  */
	),

	/* CDC 0 — console UART */
	TUD_CDC_DESCRIPTOR(
		ITF_NUM_CDC_0,		/* interface number */
		STRID_CDC_0,		/* string index     */
		EPNUM_CDC_0_NOTIF,	/* notification ep  */
		8,			/* notify ep size   */
		EPNUM_CDC_0_OUT,	/* OUT EP   */
		EPNUM_CDC_0_IN,		/* IN EP    */
		CFG_TUD_CDC_EP_BUFSIZE
	),

	/* CDC 1 — auxiliary UART */
	TUD_CDC_DESCRIPTOR(
		ITF_NUM_CDC_1,		/* interface number */
		STRID_CDC_1,		/* string index     */
		EPNUM_CDC_1_NOTIF,	/* notification ep  */
		8,			/* notify ep size   */
		EPNUM_CDC_1_OUT,	/* OUT EP   */
		EPNUM_CDC_1_IN,		/* IN EP    */
		CFG_TUD_CDC_EP_BUFSIZE
	),

	/* Vendor interface (bulk in/out) */
	TUD_VENDOR_DESCRIPTOR(
		ITF_NUM_VENDOR,		/* interface number */
		STRID_VENDOR,		/* string index     */
		EPNUM_VENDOR_OUT,
		EPNUM_VENDOR_IN,
		CFG_TUD_VENDOR_EPSIZE
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

/*
 * Designated, so that the STRID_ enum IS the array order: reordering these
 * lines is a no-op, and an index cannot disagree with the string it names.
 * Positional initialisers with the index in a comment are exactly what put
 * CDC 1's name on the vendor interface and the vendor's name on CDC 1.
 */
char const *string_desc_arr[] = {
	[STRID_LANGID]       = (const char[]){0x09, 0x04},	/* English */
	[STRID_MANUFACTURER] = "MarkM",
	[STRID_PRODUCT]      = "Pico-Thing",
	[STRID_SERIAL]       = "6809",
	[STRID_CDC_0]        = "MC6809 Serial",
	[STRID_CDC_1]        = "MC6809 Aux Serial",
	[STRID_VENDOR]       = "MC6809 Bulk",
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

	/*
	 * A designated initialiser leaves a hole if an enum member is ever added
	 * without its string, and the bound check above waves that through.
	 * strlen(NULL) here faults inside a control transfer, which is an awkward
	 * place to debug; refusing the request stalls it harmlessly instead.
	 */
	const char *str = string_desc_arr[index];
	if (str == NULL)
		return NULL;

	size_t len = MIN(strlen(str), DESC_STR_MAXLEN - 1);
	desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 + (len * 2)));

	for (size_t i = 0; i < len; i++)
		desc_str[1 + i] = str[i];

	return desc_str;
}
