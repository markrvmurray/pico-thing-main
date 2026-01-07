#include <cmoc.h>
#include "pico.h"
#include "ide_lib.h"

/* Memory-mapped base addresses for IDE channel */
#define IDE_BASE			(uint8_t *)0xFF00

/* Task file registers */
#define IDE_DATA			(*(uint16_t *)IDE_BASE)
#define IDE_ERROR			(*(IDE_BASE + 2))
#define IDE_FEATURES			(*(IDE_BASE + 2))
#define IDE_SECCNT			(*(IDE_BASE + 3))
#define IDE_LBA0			(*(IDE_BASE + 4))
#define IDE_LBA1			(*(IDE_BASE + 5))
#define IDE_LBA2			(*(IDE_BASE + 6))
#define IDE_DEVHEAD			(*(IDE_BASE + 7))
#define IDE_STATUS			(*(IDE_BASE + 8))
#define IDE_COMMAND			(*(IDE_BASE + 8))

/* Control register */
/* #define IDE_CTRL			IDE_CTRL_BASE */
#define IDE_CTRL			(*(IDE_BASE + 9))

/* Status bits */
#define IDE_SR_BSY			(uint8_t)0x80
#define IDE_SR_DRDY			(uint8_t)0x40
#define IDE_SR_DRQ			(uint8_t)0x08
#define IDE_SR_ERR			(uint8_t)0x01

/* Control bits */
#define IDE_CTRL_SRST			(uint8_t)0x04
#define IDE_CTRL_nIEN			(uint8_t)0x02

/* Commands */
#define IDE_CMD_IDENTIFY		(uint8_t)0xEC
#define IDE_CMD_READ_SECTORS		(uint8_t)0x20
#define IDE_CMD_WRITE_SECTORS		(uint8_t)0x30

/* External delay function, implemented elsewhere */
extern void sleep_ms(unsigned int ms);

static void
ide_wait_not_busy(void)
{
	while (IDE_STATUS & IDE_SR_BSY)
		;
}

static void
ide_reset(void)
{
	IDE_CTRL = IDE_CTRL_SRST;
	timer_msleep(5);  /* delay instead of busy loop */
	IDE_CTRL = (uint8_t)0x00;
	ide_wait_not_busy();
}

int
ide_identify(uint16_t *buf)
{
	ide_reset();

	IDE_DEVHEAD = (uint8_t)0xA0;
	ide_wait_not_busy();

	IDE_SECCNT = (uint8_t)0;
	IDE_LBA0 = (uint8_t)0;
	IDE_LBA1 = (uint8_t)0;
	IDE_LBA2 = (uint8_t)0;

	IDE_COMMAND = IDE_CMD_IDENTIFY;

	if (IDE_STATUS == (uint8_t)0x00)
		return -1;

	while (!(IDE_STATUS & (IDE_SR_DRQ | IDE_SR_ERR)))
		;

	if (IDE_STATUS & IDE_SR_ERR)
		return -1;

	for (int i = 0; i < 256; i++)
		buf[i] = IDE_DATA;

	return 0;
}

int
ide_read_sector(uint32_t lba, uint16_t *buf)
{
	IDE_DEVHEAD = (uint8_t)(0xE0 | ((lba >> 24) & 0x0F));
	ide_wait_not_busy();

	IDE_SECCNT = 1;
	IDE_LBA0 = (uint8_t)(lba);
	IDE_LBA1 = (uint8_t)(lba >> 8);
	IDE_LBA2 = (uint8_t)(lba >> 16);

	IDE_COMMAND = IDE_CMD_READ_SECTORS;

	while (!(IDE_STATUS & IDE_SR_DRQ))
		;

	for (int i = 0; i < 256; i++)
		buf[i] = IDE_DATA;

	return 0;
}

int
ide_write_sector(uint32_t lba, const uint16_t *buf)
{
	IDE_DEVHEAD = (uint8_t)(0xE0 | ((lba >> 24) & 0x0F));
	ide_wait_not_busy();

	IDE_SECCNT = 1;
	IDE_LBA0 = (uint8_t)(lba);
	IDE_LBA1 = (uint8_t)(lba >> 8);
	IDE_LBA2 = (uint8_t)(lba >> 16);

	IDE_COMMAND = IDE_CMD_WRITE_SECTORS;

	while (!(IDE_STATUS & IDE_SR_DRQ))
		;

	for (int i = 0; i < 256; i++)
		IDE_DATA = buf[i];

	ide_wait_not_busy();

	return 0;
}

