The KL05_SWD_Disable.elf file can be used with a FRDM-KL05 board to enable the
external programming interface.  This is necessary due to a hardware design fault 
with the board.

It does this by disabling the SWD interface on the on-board KL05 
This disables programming of the target chip!!!
To restore programming the chip must be erased

To erase the chip - 
Pull PTB13 (=A5) low and then plug in the board
