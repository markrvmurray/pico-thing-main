/*
 * Copyright (c) 2025 Mark R V Murray
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <fcntl.h>
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define USB_VID		0xCAFE
#define USB_PID		0x4001

#define CDC_BUF_SIZE	64
#define VENDOR_BUF_SIZE	64

struct usb_device_info {
	int	cdc_interface;
	uint8_t	cdc_ep_in;
	uint8_t	cdc_ep_out;
	int	vendor_interface;
	uint8_t	vendor_ep_in;
	uint8_t	vendor_ep_out;
};

static struct usb_device_info dev_info = { -1, 0, 0, -1, 0, 0 };
static libusb_context *ctx = NULL;
static libusb_device_handle *devh = NULL;
static struct termios orig_termios;

static void
restore_terminal(void)
{
	tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

static void
setup_terminal_nonblocking(void)
{
	struct termios t;

	tcgetattr(STDIN_FILENO, &orig_termios);
	atexit(restore_terminal);

	t = orig_termios;
	t.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &t);

	int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

static void
probe_device(libusb_device_handle *handle)
{
	libusb_device *dev = libusb_get_device(handle);
	struct libusb_device_descriptor desc;

	if (libusb_get_device_descriptor(dev, &desc) != 0) {
		fprintf(stderr, "Failed to get device descriptor\n");
		return;
	}

	printf("Device Descriptor:\n");
	printf("  USB Version:\t\t%x.%02x\n", desc.bcdUSB >> 8, desc.bcdUSB & 0xff);
	printf("  Vendor ID:\t\t0x%04x\n", desc.idVendor);
	printf("  Product ID:\t\t0x%04x\n", desc.idProduct);
	printf("  Device Class:\t\t0x%02x\n", desc.bDeviceClass);
	printf("  Max Packet Size0:\t%d\n", desc.bMaxPacketSize0);
	printf("  Num Configurations:\t%d\n\n", desc.bNumConfigurations);

	struct libusb_config_descriptor *config;
	if (libusb_get_active_config_descriptor(dev, &config) != 0) {
		fprintf(stderr, "Failed to get config descriptor\n");
		return;
	}

	printf("Configuration Descriptor:\n");
	printf("  Total Length:\t\t%d\n", config->wTotalLength);
	printf("  Num Interfaces:\t%d\n\n", config->bNumInterfaces);

	for (int i = 0; i < config->bNumInterfaces; i++) {
		const struct libusb_interface *iface = &config->interface[i];

		for (int alt = 0; alt < iface->num_altsetting; alt++) {
			const struct libusb_interface_descriptor *id = &iface->altsetting[alt];

			printf("  Interface %d (alt %d): class=0x%02x/0x%02x proto=0x%02x endpoints=%d\n",
			    id->bInterfaceNumber, id->bAlternateSetting,
			    id->bInterfaceClass, id->bInterfaceSubClass,
			    id->bInterfaceProtocol, id->bNumEndpoints);

			for (int e = 0; e < id->bNumEndpoints; e++) {
				const struct libusb_endpoint_descriptor *ep = &id->endpoint[e];
				printf("    EP 0x%02x (%s) attr=0x%02x pkt=%d interval=%d\n",
				    ep->bEndpointAddress,
				    (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) ? "IN" : "OUT",
				    ep->bmAttributes, ep->wMaxPacketSize, ep->bInterval);
			}

			/*
			 * CDC Data interface (class 0x0A) carries the bulk data
			 * endpoints.  The CDC Communication interface (class 0x02)
			 * carries only the notification endpoint and is left to the
			 * kernel driver.
			 */
			if (id->bInterfaceClass == 0x0A && dev_info.cdc_interface < 0) {
				dev_info.cdc_interface = id->bInterfaceNumber;
				for (int e = 0; e < id->bNumEndpoints; e++) {
					uint8_t addr = id->endpoint[e].bEndpointAddress;
					if (addr & LIBUSB_ENDPOINT_IN)
						dev_info.cdc_ep_in = addr;
					else
						dev_info.cdc_ep_out = addr;
				}
			} else if (id->bInterfaceClass == 0xFF && dev_info.vendor_interface < 0) {
				dev_info.vendor_interface = id->bInterfaceNumber;
				for (int e = 0; e < id->bNumEndpoints; e++) {
					uint8_t addr = id->endpoint[e].bEndpointAddress;
					if (addr & LIBUSB_ENDPOINT_IN)
						dev_info.vendor_ep_in = addr;
					else
						dev_info.vendor_ep_out = addr;
				}
			}
		}
	}

	printf("\nDetected CDC interface %d EPs IN=0x%02x OUT=0x%02x\n",
	    dev_info.cdc_interface, dev_info.cdc_ep_in, dev_info.cdc_ep_out);
	printf("Detected Vendor interface %d EPs IN=0x%02x OUT=0x%02x\n",
	    dev_info.vendor_interface, dev_info.vendor_ep_in, dev_info.vendor_ep_out);

	libusb_free_config_descriptor(config);
}

int
main(void)
{
	if (libusb_init(&ctx) != 0) {
		fprintf(stderr, "libusb_init failed\n");
		return 1;
	}

	devh = libusb_open_device_with_vid_pid(ctx, USB_VID, USB_PID);
	if (!devh) {
		fprintf(stderr, "Device not found (VID=0x%04X PID=0x%04X)\n", USB_VID, USB_PID);
		return 1;
	}

	libusb_set_auto_detach_kernel_driver(devh, 1);

	probe_device(devh);

	if (dev_info.vendor_interface < 0) {
		fprintf(stderr, "No vendor interface found\n");
		return 1;
	}
	if (dev_info.cdc_interface >= 0 && libusb_claim_interface(devh, dev_info.cdc_interface) != 0)
		fprintf(stderr, "Warning: cannot claim CDC interface %d\n", dev_info.cdc_interface);
	if (libusb_claim_interface(devh, dev_info.vendor_interface) != 0) {
		fprintf(stderr, "Cannot claim vendor interface %d\n", dev_info.vendor_interface);
		return 1;
	}

	setup_terminal_nonblocking();

	unsigned char cdc_buf[CDC_BUF_SIZE];
	unsigned char v_buf[VENDOR_BUF_SIZE];
	int transferred;

	for (;;) {
		/* Keyboard → CDC OUT; ESC to quit */
		int c = getchar();
		if (c == '\x1B')
			break;
		if (c != EOF && dev_info.cdc_ep_out) {
			cdc_buf[0] = (unsigned char)c;
			libusb_bulk_transfer(devh, dev_info.cdc_ep_out, cdc_buf, 1, &transferred, 0);
		}

		/* CDC IN → stdout */
		if (dev_info.cdc_ep_in) {
			int r = libusb_bulk_transfer(devh, dev_info.cdc_ep_in, cdc_buf, CDC_BUF_SIZE, &transferred, 1);
			if (r == 0 && transferred > 0) {
				fwrite(cdc_buf, 1, transferred, stdout);
				fflush(stdout);
			}
		}

		/* Vendor IN → stdout */
		if (dev_info.vendor_ep_in) {
			int r = libusb_bulk_transfer(devh, dev_info.vendor_ep_in, v_buf, VENDOR_BUF_SIZE, &transferred, 1);
			if (r == 0 && transferred > 0) {
				fwrite(v_buf, 1, transferred, stdout);
				fflush(stdout);
			}
		}

		usleep(1000);
	}

	restore_terminal();
	if (dev_info.cdc_interface >= 0)
		libusb_release_interface(devh, dev_info.cdc_interface);
	libusb_release_interface(devh, dev_info.vendor_interface);
	libusb_close(devh);
	libusb_exit(ctx);
	return 0;
}
