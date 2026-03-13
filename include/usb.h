/* usb.h */

#ifndef USB_H
#define USB_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void usb_init(void);
void usb_start(void);
void usb_poll(void);
bool usb_is_active(void);
uint16_t usb_cdc_read(uint8_t *buf, uint16_t buf_len);
uint16_t usb_cdc_write(const uint8_t *buf, uint16_t buf_len);
uint16_t usb_cdc_read_n(uint8_t itf, uint8_t *buf, uint16_t buf_len);
uint16_t usb_cdc_write_n(uint8_t itf, const uint8_t *buf, uint16_t buf_len);
void usb_cdc_flush(void);
void usb_cdc_flush_n(uint8_t itf);

#ifdef __cplusplus
}
#endif

#endif /* USB_H */
