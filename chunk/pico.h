#define false		  (0u)
#define true		  (1u)
typedef unsigned char bool;
typedef unsigned char uint8_t;
typedef char int8_t;
typedef unsigned int uint16_t;
typedef int int16_t;
typedef unsigned long uint32_t;
typedef long int32_t;

extern void initialiseFastSerial(void);
extern void newOutputRoutine(void);

//#define stringify(a) x_stringify(a)
//#define x_stringify(a) #a
#define str(x) #x
#define str1(x) str(x)
#define str2(x) str1(x)

#define NOP do { asm { nop } } while(0)
#define SYNC do { asm { sync } } while(0)

#define assert(x)	do { if (!(x)) { printf("ASSERT(#x) %s %d\n", __FILE__, __LINE__); SYNC; } } while (0)
#define PANIC(...)	do { printf(__VA_ARGS__); SYNC; } while (0)

extern uint8_t inb(uint8_t *addr);
extern void outb(uint8_t *addr, uint8_t val);
extern void insw(uint8_t *addr, void *dest, uint16_t count);
extern void outsw(uint8_t *addr, const void *dest, uint16_t count);

//extern void *malloc(uint16_t size);
//extern void timer_nsleep(uint16_t nanoseconds);
//extern void timer_usleep(uint16_t microseconds);
extern void timer_usleep_10(void);
extern void timer_msleep(uint16_t milliseconds);

struct sysvectors {
	uint8_t jmp0;
	void interrupt (*reserved)(void);
	uint8_t jmp1;
	void interrupt (*swi3)(void);
	uint8_t jmp2;
	void interrupt (*swi2)(void);
	uint8_t jmp3;
	void interrupt (*firq)(void);
	uint8_t jmp4;
	void interrupt (*irq)(void);
	uint8_t jmp5;
	void interrupt (*swi)(void);
	uint8_t jmp6;
	void interrupt (*nmi)(void);
	uint8_t jmp7;
	void interrupt (*reset)(void);
};

extern struct sysvectors *sys_vector;
