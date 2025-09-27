/* Licensed under the Apache License, Version 2.0 or the MIT License. */
/* SPDX-License-Identifier: Apache-2.0 OR MIT                         */
/* Copyright Tock Contributors 2023.                                  */

/* Memory Spaces Definitions, 2MB flash, 384K ram                     */
/* Keeping first 128 kbytes open for eventual bootloader, and to      */
/* avoid weird sector sizing of Sector 0                              */
MEMORY
{
  rom (rx)  : ORIGIN = 0x00400000, LENGTH = 0x00020000
  prog (rx) : ORIGIN = 0x00420000, LENGTH = 0x001E0000
  ram (rwx) : ORIGIN = 0x20400000, LENGTH = 0x00060000
}

PAGE_SIZE = 512;