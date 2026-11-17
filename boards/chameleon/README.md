Chameleon
=========

Tock board for the Chameleon automotive controller (PCA-0001-01): an
ATSAMV71Q21B with CAN, LIN, high- and low-side outputs and a microSD card.

**This is a draft.** It builds, but it was written from the PCB netlist before
any board existed. Nothing here has run on Chameleon hardware.

## Differences from `sam_v71_xult`

| | Xplained Ultra | Chameleon |
|---|---|---|
| Crystal | 12 MHz | 20 MHz |
| Console | USART1 + RTT | RTT only; USART1's pins are LIN A |
| CAN | MCAN1 | CAN B = MCAN1 (same pins), CAN A = MCAN0 |
| SD card detect | PD18 | PA29 |
| SD power | always on | PA24 enables U14 |
| SD pull-ups | internal | 10k on `+3V3_SD`; internal ones off |
| EEPROM | AT24MAC402 | M24C02 |
| LEDs | PA23, PC9 | none |
| Power | always on | PC1 `MAIN_HOLD` keeps it on |
| External watchdog | none | U17 window watchdog |

## Must be done before this can boot a board

* **Bootloader.** Flash `tock-bootloader/boards/chameleon-bootloader`, not the
  Xplained one, which assumes a 12 MHz crystal (PLLA would be 500 MHz) and
  leaves `MAIN_HOLD` and U17 alone. It runs first, so it asserts `MAIN_HOLD` and
  disables U17 before the crystal start. Its `safe_state` and this board's must
  stay in step.

## Not done yet

* U17 is left disabled. Feeding it needs a pulse paced to its 10.35-92.7 ms
  window, not the kernel's `tickle`, which runs every main-loop pass.
* CAN A: MCAN0 is not instantiated and no second CAN syscall driver exists. The
  transceiver is awake with TXD recessive.
* LIN, LSO/HSO outputs, VEXT, ADC inputs, digital inputs and the EEPROM have no
  drivers here. Their control pins are held off in `safe_state`.
* Going to sleep: nothing releases `MAIN_HOLD` yet.

## To check on the first board

* HSO and LSO outputs through reset. SAMV71 pins reset with a nominal 100k
  pull-up on, which R125-R130 (10k) now hold down on each ENABLE and on HSO0/1-uC.
  Check that no output moves during the reset U17 holds after +3V3 is good.
* The RTT control block is no longer at `0x20402000`. Find it with `llvm-nm` on
  the ELF, or let the J-Link search for it.
