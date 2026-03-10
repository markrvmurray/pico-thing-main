#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//#define CFG_TUSB_MCU             OPT_MCU_RP2350
//#define CFG_TUSB_OS              OPT_OS_NONE
//#define CFG_TUSB_DEBUG           0

#define CFG_TUD_ENABLED            1
#define CFG_TUD_MAX_SPEED          OPT_MODE_FULL_SPEED

#define CFG_TUD_ENDPOINT0_SIZE     64

/* ------ Device Classes Enabled ------ */
#define CFG_TUD_CDC                 2           /* two CDC interfaces (console + aux) */
#define CFG_TUD_VENDOR              1           /* one vendor interface               */

#define CFG_TUD_MSC                 0
#define CFG_TUD_HID                 0
#define CFG_TUD_MIDI                0

/* ------ CDC Buffer Sizes ------ */
#define CFG_TUD_CDC_RX_BUFSIZE     512
#define CFG_TUD_CDC_TX_BUFSIZE     512
#define CFG_TUD_CDC_EP_BUFSIZE     64

/* ------ Vendor Buffer Sizes ------ */
#define CFG_TUD_VENDOR_RX_BUFSIZE  64
#define CFG_TUD_VENDOR_TX_BUFSIZE  64
#define CFG_TUD_VENDOR_EPSIZE      64

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN          __attribute__((aligned(4)))
#endif

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
