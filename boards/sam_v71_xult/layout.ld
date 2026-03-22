MEMORY {
  flash (rx) : ORIGIN = 0x00000000, LENGTH = 2048K  /* Logical address after remap */
  sram (rwx) : ORIGIN = 0x20400000, LENGTH = 384K
}