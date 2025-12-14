/* optional_usb.h */

#ifndef OPTIONAL_USB_H
#define OPTIONAL_USB_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void optional_usb_init(void);
void optional_usb_start(void);
void optional_usb_poll(void);
bool optional_usb_is_active(void);

#ifdef __cplusplus
}
#endif

#endif /* OPTIONAL_USB_H */