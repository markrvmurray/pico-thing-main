
extern void ide_init(void);
extern void interrupt interrupt_handler(void);
void intr_register_install(void interrupt (*handler)(void));
