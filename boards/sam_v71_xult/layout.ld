MEMORY
{
  FLASH : ORIGIN = 0x00400000, LENGTH = 2048K
  RAM   : ORIGIN = 0x20400000, LENGTH = 384K
}

INCLUDE ../../boards/cargo/link.ld  # This pulls in Tock's shared cortex-m setup