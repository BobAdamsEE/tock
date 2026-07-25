/* SAMV71Q21B memory layout with tock-bootloader.
 *
 * Bootloader: 0x00000000–0x00007FFF (32 KB)   physical 0x00400000–0x00407FFF
 * Kernel:     0x00008000–0x0003FFFF (224 KB)  physical 0x00408000–0x0043FFFF
 * Apps:       0x00040000–0x001FFFFF (1792 KB) physical 0x00440000–0x005FFFFF
 * SRAM:       0x20400000–0x2045FFFF (384 KB)
 *
 * Flash addresses are in the alias region (0x00000000 maps to 0x00400000).
 * The tock-bootloader advertises appaddr="0x40000" to tockloader, which
 * matches prog ORIGIN here. */

MEMORY
{
  rom  (rx)  : ORIGIN = 0x00008000, LENGTH = 224K
  prog (rx)  : ORIGIN = 0x00040000, LENGTH = 1792K
  ram  (rwx) : ORIGIN = 0x20400000, LENGTH = 384K
}

MPU_MIN_ALIGN = 8K;
PAGE_SIZE = 512;
