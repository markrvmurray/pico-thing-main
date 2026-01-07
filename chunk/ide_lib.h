int ide_identify(uint16_t *buf);
int ide_read_sector(uint32_t lba, uint16_t *buf);
int ide_write_sector(uint32_t lba, const uint16_t *buf);
