//
// build with:
//   clang $(pkg-config --cflags --libs libusb-1.0) -Wall pio/mc6809/server.c -o server
//

#include <libusb.h>
#include <stdio.h>
#include <string.h>

#define VENDOR_ID	0xCAFE
#define PRODUCT_ID	0x4001
#define INTERFACE	2		// Vendor interface index in the descriptor
#define EP_OUT		0x03
#define EP_IN		0x83

int
main(void)
{
	libusb_device_handle *handle;
	int r = libusb_init(NULL);

	if (r < 0)
		return r;

	handle = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
	if (!handle) {
		fprintf(stderr, "Device not found\n");
		return 1;
	}

	libusb_claim_interface(handle, INTERFACE);

	unsigned char msg[] = "ping";
	unsigned char buf[64];
	int transferred;

	// send
	r = libusb_bulk_transfer(handle, EP_OUT, msg, sizeof(msg), &transferred, 1000);
	printf("Sent %d bytes\n", transferred);

	// receive
	r = libusb_bulk_transfer(handle, EP_IN, buf, sizeof(buf), &transferred, 1000);
	if (r == 0) {
		printf("Received %d bytes: %.*s\n", transferred, transferred, buf);
	} else {
		printf("Read failed: %d\n", r);
	}

	libusb_release_interface(handle, INTERFACE);
	libusb_close(handle);
	libusb_exit(NULL);
	return 0;
}
