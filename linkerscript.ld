/* Enforce emmition of the vector table. This is the vector_table variable in vector.c */
EXTERN(vector_table)

/* Define the firmware entrypoint. This is the reset_handler() function in vector.c */
ENTRY(reset_handler)

/* Define the memory layout */
MEMORY
{
	RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
	FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
}

/* Define the sections */
SECTIONS
{
	.text : {
		*(.vectors)   /* Interrupt vector table */
		*(.text*)     /* Program code */
		. = ALIGN(4);
		*(.rodata*)   /* Read-only data */
		. = ALIGN(4);
		_etext = .;
	} >FLASH

	/* Section of RAM that is not cleared on reset. */
	.noinit (NOLOAD) : {
		*(.noinit*)
	} >RAM
	. = ALIGN(4);

	/* Initialized data. */
	.data : {
		*(.data*)
		*(.ramtext*)
		. = ALIGN(4);
		_edata = .;
	} >RAM AT >FLASH

	/* Zero-initialized data. */
	.bss : {
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} >RAM

	. = ALIGN(4);
	end = .;
}

PROVIDE(_stack = ORIGIN(RAM) + LENGTH(RAM));
