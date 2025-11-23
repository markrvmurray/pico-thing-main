/* vendor_if.h */

#ifndef VENDOR_IF_H
#define VENDOR_IF_H

#include <stdint.h>
#include <stdbool.h>

void vendor_if_task(void);
bool vendor_if_send(const void *buf, uint32_t len);
int  vendor_if_recv(void *buf, uint32_t maxlen);

#endif /* VENDOR_IF_H */