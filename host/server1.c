#include <fcntl.h>
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define CDC_BUF_SIZE	64
#define VENDOR_BUF_SIZE 64

struct usb_device_info {
	int cdc_interface;
	int cdc_ep_in;
	int cdc_ep_out;
	int vendor_interface;
	int vendor_ep_in;
	int vendor_ep_out;
};

static struct usb_device_info dev_info = { -1, 0, 0, -1, 0, 0 };
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
	t.c_lflag &= ~(ICANON | ECHO); // raw mode, no echo
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
	printf("  Vendor ID:\t0x%04x\n", desc.idVendor);
	printf("  Product ID:\t0x%04x\n", desc.idProduct);
	printf("  Device Class:\t0x%02x\n", desc.bDeviceClass);
	printf("  Max Packet Size0:\t%d\n", desc.bMaxPacketSize0);
	printf("  Num Configurations:\t%d\n\n", desc.bNumConfigurations);

	struct libusb_config_descriptor *config;
	if (libusb_get_active_config_descriptor(dev, &config) != 0) {
		fprintf(stderr, "Failed to get config descriptor\n");
		return;
	}

	printf("Configuration Descriptor:\n  Total Length:\t%d\n  Num Interfaces:\t%d\n\n", config->wTotalLength, config->bNumInterfaces);

	for (int i = 0; i < config->bNumInterfaces; i++) {
		const struct libusb_interface *iface = &config->interface[i];
		for (int alt = 0; alt < iface->num_altsetting; alt++) {
			const struct libusb_interface_descriptor *desc_if = &iface->altsetting[alt];
			printf(
			    "  Interface:\n\tNumber:\t%d\n\tAlt Setting:\t%d\n\tClass/Subclass:\t0x%02x / 0x%02x\n\tProtocol:\t0x%02x\n\tEndpoints:		  %d\n",
			    desc_if->bInterfaceNumber, desc_if->bAlternateSetting, desc_if->bInterfaceClass, desc_if->bInterfaceSubClass,
			    desc_if->bInterfaceProtocol, desc_if->bNumEndpoints);

			for (int e = 0; e < desc_if->bNumEndpoints; e++) {
				const struct libusb_endpoint_descriptor *ep = &desc_if->endpoint[e];
				printf("\tEndpoint:\n\tAddress:\t0x%02x (%s)\n\tAttributes:\t0x%02x\n\tMax Packet Size:\t%d\n\tInterval:\t%d\n",
				    ep->bEndpointAddress, (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) ? "IN" : "OUT", ep->bmAttributes, ep->wMaxPacketSize,
				    ep->bInterval);

				// Detect CDC interface (class 0x02) and vendor
				// interface (class 0xFF)
				if (desc_if->bInterfaceClass == 0x02 && dev_info.cdc_interface == -1) {
					dev_info.cdc_interface = desc_if->bInterfaceNumber;
					if (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN)
						dev_info.cdc_ep_in = ep->bEndpointAddress;
					else
						dev_info.cdc_ep_out = ep->bEndpointAddress;
				} else if (desc_if->bInterfaceClass == 0xFF && dev_info.vendor_interface == -1) {
					dev_info.vendor_interface = desc_if->bInterfaceNumber;
					if (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN)
						dev_info.vendor_ep_in = ep->bEndpointAddress;
					else
						dev_info.vendor_ep_out = ep->bEndpointAddress;
				}
			}
			printf("\n");
		}
	}

	printf("Detected CDC interface %d EPs IN=0x%02x OUT=0x%02x\n", dev_info.cdc_interface, dev_info.cdc_ep_in, dev_info.cdc_ep_out);
	printf("Detected Vendor interface %d EPs IN=0x%02x OUT=0x%02x\n", dev_info.vendor_interface, dev_info.vendor_ep_in, dev_info.vendor_ep_out);

	libusb_free_config_descriptor(config);
}

int
main(void)
{
	if (libusb_init(NULL) != 0) {
		fprintf(stderr, "libusb_init failed\n");
		return 1;
	}

	devh = libusb_open_device_with_vid_pid(NULL, 0xCAFE, 0x4001);
	if (!devh) {
		fprintf(stderr, "Device not found\n");
		return 1;
	}

	if (libusb_claim_interface(devh, dev_info.cdc_interface) < 0 || libusb_claim_interface(devh, dev_info.vendor_interface) < 0) {
		fprintf(stderr, "Cannot claim interfaces\n");
		return 1;
	}

	probe_device(devh);
	setup_terminal_nonblocking();

	unsigned char cdc_buf[CDC_BUF_SIZE];
	unsigned char v_buf[VENDOR_BUF_SIZE];
	int transferred;

	while (1) {
		// --- Keyboard input (non-blocking) ---
		int c = getchar();
		if (c != EOF && dev_info.cdc_ep_out) {
			cdc_buf[0] = (unsigned char)c;
			libusb_bulk_transfer(devh, dev_info.cdc_ep_out, cdc_buf, 1, &transferred, 0);
		}

		// --- CDC IN ---
		if (dev_info.cdc_ep_in) {
			int r = libusb_bulk_transfer(devh, dev_info.cdc_ep_in, cdc_buf, CDC_BUF_SIZE, &transferred, 1);
			if (r == 0 && transferred > 0) {
				fwrite(cdc_buf, 1, transferred, stdout);
				fflush(stdout);
			}
		}

		// --- Vendor IN ---
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
	libusb_release_interface(devh, dev_info.cdc_interface);
	libusb_release_interface(devh, dev_info.vendor_interface);
	libusb_close(devh);
	libusb_exit(NULL);
	return 0;
}
