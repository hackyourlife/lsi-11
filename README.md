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
instruction test (VKABB0), FIS instruction test (VKACC1), traps test (VKADC0),
and 4K system exerciser (VKAHA1). It can boot RT-11SB V05.07, RT-11FB V05.07,
and XXDP V2.6.


Usage
-----

```
lsi-11 [OPTIONS] [-t trace.trc] [-f floppy.rx2] [-d disk.rl2] [program.bin]
```

Options:
- `-t`: saves the complete execution trace of the LSI-11 system in the
  given file. This is useful for offline analysis.
- `-f`: floppy disk image for the first floppy disk drive.
- `-f1`: floppy disk image for the second floppy disk drive.
- `-d`: hard disk image for the first RL02 unit.
- `-d1`: hard disk image for the second RL02 unit.
- `-d2`: hard disk image for the third RL02 unit.
- `-d3`: hard disk image for the fourth RL02 unit.
- `-sd`: save hard disk image of first RL02 unit on exit
- `-sd1`: save hard disk image of second RL02 unit on exit
- `-sd2`: save hard disk image of third RL02 unit on exit
- `-sd3`: save hard disk image of fourth RL02 unit on exit
- `-q`: quiet start, skip BDV11 console test.
- `-n`: skip BDV11 cpu and memory tests to get shorter execution traces.
- `-x`: exit on HALT.
- `-l`: load program in absolute loader format.
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
