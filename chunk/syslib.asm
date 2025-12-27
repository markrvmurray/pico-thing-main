	USE	syslib.inc

DEVICE	EXPORT
SHARED	EXPORT
SNIPPET	EXPORT
VECTORS	EXPORT

UARTC	EXPORT
UARTS	EXPORT
UARTTX	EXPORT
UARTRX	EXPORT

Start	EXTERN

	SECTION	PICO2

DEVICE	RMB	8		; Devices provided by the Pico 2
UARTC	RMB	0		; These two share an address
UARTS	RMB	1
UARTTX	RMB	0		; These tow share an address
UARTRX	RMB	1
	RMB	6
SHARED	RMB	16		; Address of a shared buffer for moving bytes
SNIPPET	RMB	16		; Start address of code snippet
VECTORS	RMB	0		; CPU reset and interrupt vectors
	FDB	0
	FDB	0
	FDB	0
	FDB	0
	FDB	IRQ_ISR
	FDB	0
	FDB	0
	FDB	Start

	ENDSECTION
