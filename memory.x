/* Linker script for the STM32F103C8T6 *with* stm32duino bootloader */
MEMORY
{
  FLASH : ORIGIN = 0x08002000, LENGTH = 56K
  RAM : ORIGIN = 0x20000000, LENGTH = 20K
}
