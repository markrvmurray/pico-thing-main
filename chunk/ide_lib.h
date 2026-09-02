/* Every routine returns 0 on success.  The errors are negative, so a caller
 * that only tests for non-zero still sees both. */
#define IDE_ERR_DRIVE	(-1)	/* the drive reported an error */
#define IDE_ERR_ADDRESS	(-2)	/* the task file did not read back; no command issued */

int ide_identify(uint16_t *buf);
int ide_read_sector(uint32_t lba, uint16_t *buf);
int ide_write_sector(uint32_t lba, const uint16_t *buf);
