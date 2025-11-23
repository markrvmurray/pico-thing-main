#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <libusb-1.0/libusb.h>

#define TIMEOUT_MS 100

static struct termios orig_termios;

static void restore_terminal(void)
{
	tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

static void setup_terminal_nonblocking(void)
{
	struct termios t;
	tcgetattr(STDIN_FILENO, &orig_termios);
	atexit(restore_terminal);

	t = orig_termios;
	t.c_lflag &= ~(ICANON | ECHO); // raw mode, no echo
	tcsetattr(STDIN_FILENO, TCSANOW, &t);

	int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

int main(void)
{
	libusb_context *ctx;
	libusb_device_handle *dev_handle;
	int r = libusb_init(&ctx);
	if (r < 0) {
		fprintf(stderr,"libusb_init failed\n");
		return 1;
	}

	dev_handle = libusb_open_device_with_vid_pid(ctx, 0xCAFE, 0x4001);
	if (!dev_handle) {
		fprintf(stderr,"Device not found\n");
		return 1;
	}

	libusb_set_auto_detach_kernel_driver(dev_handle, 1);

	libusb_device *dev = libusb_get_device(dev_handle);
	struct libusb_config_descriptor *cfg;
	libusb_get_active_config_descriptor(dev, &cfg);

	int cdc_if = -1, vendor_if = -1;
	uint8_t cdc_ep_in=0, cdc_ep_out=0, vendor_ep_in=0, vendor_ep_out=0;

	for (int i = 0; i < cfg->bNumInterfaces; i++) {
		const struct libusb_interface *itf = &cfg->interface[i];
		for (int j=0; j < itf->num_altsetting; j++) {
			const struct libusb_interface_descriptor *desc = &itf->altsetting[j];
			if (desc->bInterfaceClass == 0x02 && desc->bInterfaceSubClass == 0x02) { /* CDC data */
				cdc_if = desc->bInterfaceNumber;
				for (int k=0;k<desc->bNumEndpoints;k++) {
					if ((desc->endpoint[k].bEndpointAddress & LIBUSB_ENDPOINT_IN))
						cdc_ep_in = desc->endpoint[k].bEndpointAddress;
					else
						cdc_ep_out = desc->endpoint[k].bEndpointAddress;
				}
			} else if (desc->bInterfaceClass == 0xFF) { /* Vendor */
				vendor_if = desc->bInterfaceNumber;
				for (int k=0;k<desc->bNumEndpoints;k++) {
					if ((desc->endpoint[k].bEndpointAddress & LIBUSB_ENDPOINT_IN))
						vendor_ep_in = desc->endpoint[k].bEndpointAddress;
					else
						vendor_ep_out = desc->endpoint[k].bEndpointAddress;
				}
			}
		}
	}

	if (cdc_if >= 0) libusb_claim_interface(dev_handle, cdc_if);
	if (vendor_if >= 0) libusb_claim_interface(dev_handle, vendor_if);

	printf("CDC interface %d EP IN=0x%02X OUT=0x%02X\n", cdc_if, cdc_ep_in, cdc_ep_out);
	printf("Vendor interface %d EP IN=0x%02X OUT=0x%02X\n", vendor_if, vendor_ep_in, vendor_ep_out);

	uint8_t buf[64];
	int transferred;

	setup_terminal_nonblocking();

	for (;;) {
		int c = getchar();
		if (c == '\x1B')
			break;
		if (c != EOF) {
			putchar(c);
			buf[0] = (unsigned char)c;
			libusb_bulk_transfer(dev_handle, cdc_ep_out, buf, 1, &transferred, TIMEOUT_MS);
		}

		/* CDC: read + echo */
		r = libusb_bulk_transfer(dev_handle, cdc_ep_in, buf, sizeof(buf), &transferred, TIMEOUT_MS);
		if (r == 0 && transferred) {
			fprintf(stderr, "CDC: %s\n", buf);
			libusb_bulk_transfer(dev_handle, cdc_ep_out, buf, transferred, &transferred, TIMEOUT_MS);
		} else
			printf("CDC = %d - %s\n", r, libusb_error_name(r));


		/* Vendor: read + echo */
		r = libusb_bulk_transfer(dev_handle, vendor_ep_in, buf, sizeof(buf), &transferred, TIMEOUT_MS);
		if (r == 0 && transferred) {
			fprintf(stderr, "VENDOR: %s\n", buf);
			libusb_bulk_transfer(dev_handle, vendor_ep_out, buf, transferred, &transferred, TIMEOUT_MS);
		} else
			printf("VENDOR = %d - %s\n", r, libusb_error_name(r));

	}

	restore_terminal();

	libusb_release_interface(dev_handle, cdc_if);
	libusb_release_interface(dev_handle, vendor_if);
	libusb_close(dev_handle);
	libusb_exit(ctx);
	return 0;
}
