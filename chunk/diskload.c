#include <cmoc.h>
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

#define USE_PACKBITS

#define SOH  ((uint8_t)0x01)
#define EOT  ((uint8_t)0x04)
#define ACK  ((uint8_t)0x06)
#define NAK  ((uint8_t)0x15)

#define BLOCK_WORD_COUNT  256u    /* 256 16-bit words = 512 bytes per ATA sector */
#define SECTOR_SIZE       512u

/* Blocking receive: spins on INCH until a byte arrives. */
static uint8_t
rx_byte(void)
{
    uint8_t c;
    asm {
        EXTERN INCH
        EXTERN AUX_PORT
        pshs  u
        ldu   #AUX_PORT
loop@:  lbsr  INCH      ; byte in A, carry set if available
        bcc   loop@
        puls  u
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
        EXTERN AUX_PORT
        lda   :c
        pshs  u
        ldu   #AUX_PORT
        lbsr  OUTCH
        puls  u
    }
}

#ifdef USE_PACKBITS
/*
 * PackBits RLE decoder.  Control byte semantics:
 *   0..127   -> copy next (n+1) literal bytes   (1-128)
 *   129..255 -> repeat next byte (257-n) times  (2-128)
 *   128      -> no-op / end marker
 *
 * Returns number of bytes written to dst.
 */
static uint16_t
packbits_decode(uint8_t *dst, uint16_t dst_cap)
{
    uint8_t *out = dst;
    uint8_t *end = dst + dst_cap;
    uint8_t n, count, val;

    for (;;) {
        n = rx_byte();
        if (n == 128u)
            break;                    /* end-of-block marker */
        if (n < 128u) {               /* literal run: n+1 bytes */
            count = n + 1u;
            while (count-- && out < end)
                *out++ = rx_byte();
        } else {                      /* repeat run: 257-n copies */
            count = 257u - n;
            val = rx_byte();
            while (count-- && out < end)
                *out++ = val;
        }
    }
    return (uint16_t)(out - dst);
}
#endif /* USE_PACKBITS */

int
main(void)
{
    uint16_t buf[BLOCK_WORD_COUNT];
    uint8_t *p;
    uint8_t b, chk, got, c;
    uint32_t lba;
    unsigned i;

    initialiseFastSerial();
    setConsoleOutHook(newOutputRoutine);

    printf("Disk imaging facility starting\n");

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

        /* Receive 512-byte sector payload */
        p   = (uint8_t *)buf;
        chk = 0;
#ifdef USE_PACKBITS
        {
            uint16_t decoded = packbits_decode(p, SECTOR_SIZE);
            if (decoded != SECTOR_SIZE) {
                tx_byte(NAK);
                continue;
            }
            for (i = 0; i < SECTOR_SIZE; i++)
                chk ^= p[i];
        }
#else
        for (i = 0; i < SECTOR_SIZE; i++) {
            b    = rx_byte();
            *p++ = b;
            chk ^= b;
        }
#endif

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
