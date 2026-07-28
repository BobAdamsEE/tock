/* SAMV71Q21B memory layout with tock-bootloader.
 *
 * Bootloader: 0x00000000–0x0000FFFF (64 KB)   physical 0x00400000–0x0040FFFF
 * Kernel:     0x00010000–0x0003FFFF (192 KB)  physical 0x00410000–0x0043FFFF
 * Backup:     0x00040000–0x0006FFFF (192 KB)  physical 0x00440000–0x0046FFFF
 * Apps:       0x00070000–0x001FFFFF (1600 KB) physical 0x00470000–0x005FFFFF
 * SRAM:       0x20400000–0x2045FFFF (384 KB)
 *
 * Flash addresses are in the alias region (0x00000000 maps to 0x00400000).
 * The tock-bootloader advertises appaddr="0x70000" to tockloader, which
 * matches prog ORIGIN here.
 *
 * The bootloader region is 64 KB but only ~27 KB of it is used. The rest is
 * reserved deliberately: signature verification costs about 12 KB, and growing
 * the region after the fact would move the kernel start -- the one change that
 * cannot be deployed over CAN or UART, because the bootloader performing the
 * update is the thing being replaced. See the design document, section 14.
 *
 * The backup region is a cold copy of the kernel, never executed in place. It
 * mirrors the active region exactly so that any kernel which fits in one fits
 * in the other, which is why it is 192 KB rather than the ~87 KB a kernel
 * currently needs. The kernel is linked EXEC at 0x00010000 and is not position
 * independent, so a backup cannot simply be booted where it lies -- rollback
 * copies it down over the active slot. See section 19. */

MEMORY
{
  rom  (rx)  : ORIGIN = 0x00010000, LENGTH = 192K
  prog (rx)  : ORIGIN = 0x00070000, LENGTH = 1600K
  ram  (rwx) : ORIGIN = 0x20400000, LENGTH = 384K
}

MPU_MIN_ALIGN = 8K;
PAGE_SIZE = 512;
