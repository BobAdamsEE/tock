/* SAMV71Q21B memory layout for use with the Tock bootloader.
 *
 * Bootloader: 0x00000000–0x00007FFF (32 KB)
 * Kernel:     0x00008000–0x0003FFFF (224 KB)
 * Apps:       0x00040000–0x001FFFFF (1792 KB)
 * SRAM:       0x20400000–0x2045FFFF (384 KB)
 *
 * Flash addresses are in the alias region (0x00000000 maps to 0x00400000). */

MEMORY
{
  rom  (rx)  : ORIGIN = 0x00008000, LENGTH = 224K
  prog (rx)  : ORIGIN = 0x00040000, LENGTH = 1792K
  ram  (rwx) : ORIGIN = 0x20400000, LENGTH = 384K
}

MPU_MIN_ALIGN = 8K;
PAGE_SIZE = 512;
