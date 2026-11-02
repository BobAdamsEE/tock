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

* **Bootloader.** `tock-bootloader/boards/samv71xplained-bootloader` runs first
  and has neither of these right: it assumes a 12 MHz crystal (PLLA would be
  500 MHz) and uses USART1. It must also assert `MAIN_HOLD` and disable U17
  before anything slow: the crystal start alone can take 62 ms, and U17 expects
  its first WDI edge within 92.7 ms.

## Not done yet

* U17 is left disabled. Feeding it needs a pulse paced to its 10.35-92.7 ms
  window, not the kernel's `tickle`, which runs every main-loop pass.
* CAN A: MCAN0 is not instantiated and no second CAN syscall driver exists. The
  transceiver is awake with TXD recessive.
* LIN, LSO/HSO outputs, VEXT, ADC inputs, digital inputs and the EEPROM have no
  drivers here. Their control pins are held off in `safe_state`.
* Going to sleep: nothing releases `MAIN_HOLD` yet.

## To check on the first board

* HSO outputs through reset. SAMV71 pins reset with a nominal 100k pull-up on,
  and U13's IN pull-downs are only 100-230k, through R77/R78 (4.7k). That puts
  IN at 1.6-2.3 V, 2.06 V typical, against a VIH of 2 V, so both channels can
  turn on until firmware drives PA11 and PA13 low. A 10k pull-down on the MCU
  side of R77 and R78 would hold IN near 0.3 V. U17 holds reset for 8.5-11.5 ms
  after +3V3 is good, so the bootloader must drive these pins first.
* LSO outputs through reset. BTF3080 IN and ENABLE have 25-100k pull-downs, which
  gives 0.66-1.65 V: below VIH (2 V), and above VIL (0.8 V) for most of the
  range. An output needs both pins high, so this is lower risk than HSO, but
  check it with a load fitted.
* The RTT control block is no longer at `0x20402000`. Find it with `llvm-nm` on
  the ELF, or let the J-Link search for it.
