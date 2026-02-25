CC              = clang
CFLAGS          = -Wall -O2
PKGCFG          = pkg-config
PKGCFG_LIBUSB   = libusb-1.0

# Use pkg-config if available, otherwise fall back to -lusb-1.0
PKG_CFLAGS      := $(shell $(PKGCFG) --cflags $(PKGCFG_LIBUSB) 2>/dev/null)
PKG_LIBS        := $(shell $(PKGCFG) --libs $(PKGCFG_LIBUSB) 2>/dev/null)

# Fallback if pkg-config failed
ifeq ($(PKG_CFLAGS),)
    $(warning pkg-config not found or libusb-1.0 missing; using fallback -lusb-1.0)
    PKG_CFLAGS =
    PKG_LIBS   = -lusb-1.0
endif

all: vendor_host server vendor_server

vendor_host: vendor_host.c
	$(CC) $(CFLAGS) $(PKG_CFLAGS) -o vendor_host vendor_host.c $(PKG_LIBS)

server: server.c
	$(CC) $(CFLAGS) $(PKG_CFLAGS) -o server server.c $(PKG_LIBS)

vendor_server: vendor_server.c
	$(CC) $(CFLAGS) $(PKG_CFLAGS) -o vendor_server vendor_server.c $(PKG_LIBS)

clean:
	rm -f vendor_host server vendor_server
