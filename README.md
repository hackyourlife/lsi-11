LSI-11
======

This is an emulator of an LSI-11 system (PDP-11/03-L).

The emulated system has 5 modules installed:
- KD11-NA (M7270) CPU
- MSV11 (M8044) 32KW RAM
- RXV21 (M8029) RX02 floppy disk controller
- DLV11-J (M8043) 4x Serial line interface
- BDV11 bus terminator, bootstrap, and diagnostic ROM

This emulator passes (at least) the basic instruction test (VKAAC0), EIS
instruction test (VKABB0), traps test (VKADC0), and 4K system exerciser
(VKAHA1). It can boot RT-11SB V05.07 and XXDP V2.6.

Usage
-----

```
lsi-11 [-t trace.trc] [-f floppy.bin] [-h] [program.bin]
```

Options:
- `-t`: saves the complete execution trace of the LSI-11 system in the
  given file. This is useful for offline analysis.
- `-f`: floppy disk image for the first floppy disk drive.
- `-h`: Halt. If this option is given, the system will immediately halt.
- `program.bin`: the last argument is an optional program file in
  standard absolute loader format. If it is specified, the emulator will
  load and execute it. If it is not specified, the emulator will start
  the bootstrap code from the BDV11 modules.

Implementation Details
----------------------

- Microcode ODT is implemented (but incomplete); it is probably the only
  implementation of microcode ODT in a PDP-11 emulator.
- The switch configuration on the BDV11 module can be changed in the file
  `src/bdv11.c`. The macro `BDV11_SWITCH` is the switch configuration.
- IRQs are delayed: if a device asserts an IRQ, some time passes before
  the CPU acknowledges it. This is necessary for some tests to pass.
- The time between asserting the IRQ and the CPU acknowledging it has a
  random jitter. This is required to pass some tests which assume that
  there is always some jitter.
- It is easily possible to develop new (virtual) modules. The
  implementation of the RAM module (MSV11-DE) could serve as an example.
