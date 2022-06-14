BootLoopMKL05_SWD_Disabled
==========================

The BootLoopMKL05_SWD_Disabled.elf file can be used with a FRDM-KL05 board to enable the
external programming interface without cutting tracks etc.  This is necessary due to a hardware
design fault with the board where the links are in the wrong location.

It achieves this by disabling the SWD interface on the on-board KL05 so that it does not interfere with the
external programming connector signals.
This also disables programming of the target chip!!!
To restore programming the chip must be erased (see below to restore).

To use:
- Program the USBDM firmware to the OpenSDA interface in the usual manner.
- Use the USBDM programmer to program KL05_SWD_Disable.elf to the target KL05.
- You can now use the external programming interface without changing any jumpers or links.

To erase the chip - 
Pull PTB8 (=A0) low and then plug in the board.  The firmware will erase itself.
Note the chip will be mass-erased which leaves the chip secured but re-programmable.

