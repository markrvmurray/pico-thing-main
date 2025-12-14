#include <libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define USB_VID	   0xCAFE
#define USB_PID	   0x4001

#define TIMEOUT_MS 10

static void
print_device_descriptor(struct libusb_device_descriptor *d)
{
	printf("Device Descriptor:\n");
	printf("  USB Version:\t\t%x.%02X\n", d->bcdUSB >> 8, d->bcdUSB & 0xFF);
	printf("  Vendor ID:\t\t0x%04X\n", d->idVendor);
	printf("  Product ID:\t\t0x%04X\n", d->idProduct);
	printf("  Device Class:\t\t0x%02X\n", d->bDeviceClass);
	printf("  Max Packet Size0:\t%u\n", d->bMaxPacketSize0);
	printf("  Num Configurations:\t%u\n\n", d->bNumConfigurations);
}

static void
print_endpoint(const struct libusb_endpoint_descriptor *ep)
{
	printf("      Endpoint:\n");
	printf("\tAddress:\t\t0x%02X (%s)\n", ep->bEndpointAddress, (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) ? "IN" : "OUT");
	printf("\tAttributes:\t0x%02X\n", ep->bmAttributes);
	printf("\tMax Packet Size:\t%u\n", ep->wMaxPacketSize);
	printf("\tInterval:\t\t%u\n", ep->bInterval);
}

static void
print_interface(const struct libusb_interface *iface)
{
	for (int a = 0; a < iface->num_altsetting; a++) {
		const struct libusb_interface_descriptor *id = &iface->altsetting[a];

		printf("    Interface:\n");
		printf("      Number:\t\t%u\n", id->bInterfaceNumber);
		printf("      Alt Setting:\t%u\n", id->bAlternateSetting);
		printf("      Class/Subclass:\t0x%02X / 0x%02X\n", id->bInterfaceClass, id->bInterfaceSubClass);
		printf("      Protocol:\t0x%02X\n", id->bInterfaceProtocol);
		printf("      Endpoints:\t\t%u\n", id->bNumEndpoints);

		for (int e = 0; e < id->bNumEndpoints; e++)
			print_endpoint(&id->endpoint[e]);

		printf("\n");
	}
}

static void
print_configuration(libusb_device_handle *h, struct libusb_config_descriptor *cfg)
{
	printf("Configuration Descriptor:\n");
	printf("  Total Length:		%u\n", cfg->wTotalLength);
	printf("  Num Interfaces:		%u\n\n", cfg->bNumInterfaces);

	for (int i = 0; i < cfg->bNumInterfaces; i++)
		print_interface(&cfg->interface[i]);
}

int
main(void)
{
	libusb_context *ctx = NULL;
	libusb_device_handle *devh = NULL;
	libusb_device *dev = NULL;
	struct libusb_device_descriptor ddesc;
	struct libusb_config_descriptor *cfg = NULL;

	int vendor_ifnum = -1;
	uint8_t ep_in = 0, ep_out = 0;

	if (libusb_init(&ctx) < 0) {
		printf("Failed to init libusb\n");
		return 1;
	}

	devh = libusb_open_device_with_vid_pid(ctx, USB_VID, USB_PID);
	if (devh == NULL) {
		printf("Device not found.\n");
		return 1;
	}

	dev = libusb_get_device(devh);

	if (libusb_get_device_descriptor(dev, &ddesc) != 0) {
		printf("Failed to get device descriptor\n");
		return 1;
	}

	print_device_descriptor(&ddesc);

	if (libusb_get_active_config_descriptor(dev, &cfg) != 0) {
		printf("Failed to get configuration descriptor\n");
		return 1;
	}

	print_configuration(devh, cfg);

	/* ---- Find vendor interface and endpoints ---- */
	for (int i = 0; i < cfg->bNumInterfaces; i++) {
		const struct libusb_interface *iface = &cfg->interface[i];

		for (int a = 0; a < iface->num_altsetting; a++) {
			const struct libusb_interface_descriptor *id = &iface->altsetting[a];

			/* TinyUSB vendor class = 0xFF */
			if (id->bInterfaceClass == 0xFF) {
				vendor_ifnum = id->bInterfaceNumber;

				for (int e = 0; e < id->bNumEndpoints; e++) {
					uint8_t addr = id->endpoint[e].bEndpointAddress;

					if (addr & LIBUSB_ENDPOINT_IN)
						ep_in = addr;
					else
						ep_out = addr;
				}
			}
		}
	}

	printf("Vendor interface:\t\t%d\n", vendor_ifnum);
	printf("Vendor EP IN:\t\t0x%02X\n", ep_in);
	printf("Vendor EP OUT:\t\t0x%02X\n\n", ep_out);

	if (vendor_ifnum < 0 || ep_in == 0 || ep_out == 0) {
		printf("ERROR: Vendor interface or endpoints not found\n");
		return 1;
	}

	if (libusb_claim_interface(devh, vendor_ifnum) != 0) {
		printf("Failed to claim interface %d\n", vendor_ifnum);
		return 1;
	}

	for (;;) {
		/* ---- Test write -> read ---- */
		uint8_t txbuf[8] = "HELLO!\n";
		uint8_t rxbuf[64];
		int r, transferred = 0;

		printf("Sending test packet...\n");
		r = libusb_bulk_transfer(devh, ep_out, txbuf, sizeof(txbuf), &transferred, TIMEOUT_MS);
		if (r != 0 && r != LIBUSB_ERROR_TIMEOUT)
			printf("Write failed: %d (%s)\n", r, libusb_error_name(r));
		printf("Wrote %d bytes\n", transferred);

		r = libusb_bulk_transfer(devh, ep_in, rxbuf, sizeof(rxbuf), &transferred, TIMEOUT_MS);
		if (r != 0 && r != LIBUSB_ERROR_TIMEOUT)
			printf("Read failed: %d (%s)\n", r, libusb_error_name(r));
		if (transferred)
			printf("Read %d bytes: '%.*s'\n", transferred, transferred, rxbuf);
	}

	libusb_release_interface(devh, vendor_ifnum);
	libusb_close(devh);
	libusb_exit(ctx);

	return 0;
}
