#include "pico.h"
#include "ide_lib.h"

/*
 * diskload.c - Write a disk image to the PATA drive over serial.
 *
 * Protocol (host -> 6809, per 512-byte ATA sector):
 *   SOH  LBA[23:16]  LBA[15:8]  LBA[7:0]  [512 bytes]  XOR-checksum
 *   <- ACK (0x06) on success, NAK (0x15) on bad checksum or write error
 *
 * End of transfer:
 *   EOT
 *   <- ACK
 *
 * The host-side counterpart is scripts/send_disk.py in the nitros9 repo.
 *
 * Build:
 *   make diskload.srec   (or diskload.raw)
 */

#define SOH  ((uint8_t)0x01)
#define EOT  ((uint8_t)0x04)
#define ACK  ((uint8_t)0x06)
#define NAK  ((uint8_t)0x15)

#define BLOCK_WORD_COUNT  256u    /* 256 16-bit words = 512 bytes per ATA sector */

/* Blocking receive: spins on INCH until a byte arrives. */
static uint8_t
rx_byte(void)
{
    uint8_t c;
    asm {
loop@:
        EXTERN INCH
        lbsr  INCH      ; byte in A, carry set if available
        bcc   loop@
        sta   :c
    }
    return c;
}

/* Transmit one byte via OUTCH. */
static void
tx_byte(uint8_t c)
{
    asm {
        EXTERN OUTCH
        lda   :c
        lbsr  OUTCH
    }
}

int
main(void)
{
    uint16_t buf[BLOCK_WORD_COUNT];
    uint8_t *p;
    uint8_t b, chk, got, c;
    uint32_t lba;
    unsigned i;

    initialiseFastSerial();

    /* Ready signal: send_disk.py waits for this before sending frames. */
    tx_byte('R');

    for (;;) {
        c = rx_byte();

        if (c == EOT) {
            tx_byte(ACK);
            break;
        }

        if (c != SOH) {
            tx_byte(NAK);
            continue;
        }

        /* 3-byte LBA, big-endian */
        lba  = (uint32_t)rx_byte() << 16;
        lba |= (uint32_t)rx_byte() << 8;
        lba |= (uint32_t)rx_byte();

        /* Receive 512 bytes and accumulate XOR checksum */
        p   = (uint8_t *)buf;
        chk = 0;
        for (i = 0; i < 512u; i++) {
            b    = rx_byte();
            *p++ = b;
            chk ^= b;
        }

        /* Verify checksum */
        got = rx_byte();
        if (got != chk) {
            tx_byte(NAK);
            continue;
        }

        /* Write to PATA */
        if (ide_write_sector(lba, buf) != 0) {
            tx_byte(NAK);
            continue;
        }

        tx_byte(ACK);
    }

    SYNC;
    return 0;
}
