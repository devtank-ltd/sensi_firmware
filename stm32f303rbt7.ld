/* Linker script for the STM32F303RBT7 chip. */

/* Define memory regions. */
MEMORY
{
	rom (rx) :     ORIGIN = 0x08000000, LENGTH = 256K
	ccmram (rwx) : ORIGIN = 0x10000000, LENGTH = 8K
	ram (rwx) :    ORIGIN = 0x20000000, LENGTH = 40K
}

/* Include the common ld script. */
INCLUDE cortex-m-generic.ld
