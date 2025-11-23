/* vendor_if.c — FreeBSD style */

#include "tusb.h"
#include "vendor_if.h"

bool
vendor_if_send(const void *buf, uint32_t len)
{
        if (!tud_vendor_available()) {
                return false;
        }

        uint32_t w = tud_vendor_write(buf, len);
        tud_vendor_flush();
        return (w == len);
}

int
vendor_if_recv(void *buf, uint32_t maxlen)
{
        if (!tud_vendor_available()) {
                return 0;
        }

        return (int) tud_vendor_read(buf, maxlen);
}

void
vendor_if_task(void)
{
        /* No background logic; user pulls/pushes data directly */
}