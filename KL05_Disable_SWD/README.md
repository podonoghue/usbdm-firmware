This binary produced by this project (KL05_SWD_Disable.elf) may be used with a 
FRDM-KL05 board to enable the external programming interface without changing any 
links or modifying the board.  

This is necessary due to a hardware design fault with the board.

It does this by disabling the SWD interface on the on-board KL05 including
the RESET and SWD pins. 

This temporarily disables programming of the target chip!!!
To restore programming the chip must be erased

To erase the chip - 
Pull PTB13 (=A5) low and then plug in the board
