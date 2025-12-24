ENTRY(reset_handler)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00400000, LENGTH = 2048K
  RAM (rwx) : ORIGIN = 0x20400000, LENGTH = 384K
}

SECTIONS
{
  .text :
  {
    *(.vector_table)   /* Vector table */
    *(.text*)          /* Code */
    *(.rodata*)        /* Read-only data */
  } > FLASH

  .data :
  {
    _sdata = .;
    *(.data*)
    _edata = .;
  } > RAM AT > FLASH

  .bss :
  {
    _sbss = .;
    *(.bss*)
    *(COMMON)
    _ebss = .;
  } > RAM

  /* Stack at top of RAM (optional, but common) */
  _stack_start = ORIGIN(RAM) + LENGTH(RAM);
}