/* SAMV71Q21B memory layout with tock-bootloader.
 *
 * Bootloader: 0x00000000–0x0000FFFF (64 KB)   physical 0x00400000–0x0040FFFF
 * Kernel:     0x00010000–0x0003FFFF (192 KB)  physical 0x00410000–0x0043FFFF
 * Apps:       0x00040000–0x001FFFFF (1792 KB) physical 0x00440000–0x005FFFFF
 * SRAM:       0x20400000–0x2045FFFF (384 KB)
 *
 * Flash addresses are in the alias region (0x00000000 maps to 0x00400000).
 * The tock-bootloader advertises appaddr="0x40000" to tockloader, which
 * matches prog ORIGIN here.
 *
 * The bootloader region is 64 KB but only ~24 KB of it is used. The rest is
 * reserved deliberately: signature verification costs about 12 KB, and growing
 * the region after the fact would move the kernel start -- the one change that
 * cannot be deployed over CAN or UART, because the bootloader performing the
 * update is the thing being replaced. Reserving it now keeps that upgrade a
 * drop-in bootloader swap. The 32 KB came from kernel headroom, which still
 * leaves ~105 KB spare here, and the application region is deliberately
 * unmoved. See the design document, section 14. */

MEMORY
{
  rom  (rx)  : ORIGIN = 0x00010000, LENGTH = 192K
  prog (rx)  : ORIGIN = 0x00040000, LENGTH = 1792K
  ram  (rwx) : ORIGIN = 0x20400000, LENGTH = 384K
}

MPU_MIN_ALIGN = 8K;
PAGE_SIZE = 512;
