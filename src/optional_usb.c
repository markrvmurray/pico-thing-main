/* optional_usb.c – FreeBSD style */

#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "bsp/board.h"
#include "tusb.h"
#include "optional_usb.h"
//#include "uart.h"

// --- USB optional init / force enumeration ---
void
optional_usb_init(void)
{
	// Brief delay for host detection (macOS especially)
	sleep_ms(100);

	// Reset USB PHY to ensure clean state after debugger flash
	reset_block(RESETS_RESET_USBCTRL_BITS);
	unreset_block(RESETS_RESET_USBCTRL_BITS);
	sleep_ms(50);

	// Initialize board pins (USB DP/DM, etc.)
	board_init();

	// Check USB clock frequency
	uint usb_freq = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY);
	printf("USB clock frequency: %u kHz\n", usb_freq);
	if (usb_freq != 48000)
		printf("ERROR: USB clock is not 48MHz\n");

	// TinyUSB device stack
	tusb_rhport_init_t dev_init = {
		.role = TUSB_ROLE_DEVICE,
		.speed = TUSB_SPEED_AUTO
	};
	tusb_init(0, &dev_init);

	// Optional board callback after TinyUSB init
	if (board_init_after_tusb)
		board_init_after_tusb();

	// Force clean enumeration
	sleep_ms(100);
	tud_disconnect();
	sleep_ms(100);
	tud_connect();

	// Optional extra delay for host
	sleep_ms(50);
}

static void
cdc_loopback(void)
{
	uint8_t buf[64];
	uint32_t count;

	if (!tud_cdc_connected())
		return;

	if (tud_cdc_available() > 0) {
		count = tud_cdc_read(buf, sizeof(buf));
		if (count > 0) {
			tud_cdc_write(buf, count);
			tud_cdc_write_flush();
		}
	}
}

/* Called when device mounted (enumerated) */
void
tud_mount_cb(void)
{
	printf("USB mounted\n");
}

/* Called when device unmounted (disconnected) */
void
tud_umount_cb(void)
{
	printf("USB unmounted\n");
}

/* CDC handled by polling; no extra callbacks required for simple loopback */

static absolute_time_t g_next_originate;

// Called when the user sends serial data down the Pico's main USB/Power route.
// Hand the virtual UART to the guest CPU.
void
tud_vendor_rx_cb(uint8_t itf, uint8_t const *buf, uint16_t bufsize)
{
	(void) itf;
	// printf("USB Rx callback: got %u bytes\n", bufsize);

	// MUST tell TinyUSB the data was consumed
	tud_vendor_read(NULL, bufsize);

	// Send it to the guest
#if 1
	if (bufsize > 0) {
		tud_vendor_write(buf, bufsize);
		tud_vendor_write_flush();
	}
#endif
}

#ifdef NOT_USED_JUST_YET
/* Optional vendor tx complete callback */
void
tud_vendor_tx_cb(uint8_t itf, uint32_t sent_bytes)
{
	(void) itf;
	(void) sent_bytes;
	// printf("USB Tx callback\n");
	/* Could queue more data here */
}
#endif

/* Periodic originate interval (ms) */
#define VENDOR_ORIGINATE_MS   5000
/* Utility: send periodic vendor-originated message */
static void
vendor_maybe_originate(void)
{
	if (!tud_mounted())
		return;

	if (!time_reached(g_next_originate))
		return;

	const uint8_t msg[] = "device-initiated-ping\n";
	if (tud_vendor_write(msg, sizeof(msg) - 1) > 0)
		tud_vendor_write_flush();

	g_next_originate = make_timeout_time_ms(VENDOR_ORIGINATE_MS);
}

void
optional_usb_poll(void)
{
	tud_task();
	cdc_loopback();
	vendor_maybe_originate();
}

bool
optional_usb_is_active(void)
{
	return (tud_ready());
}
