==== What is it =====
This is a utility to mass erase the TARGET chip on a FRDM Kinetis board.  It is intended
to be quite aggressive in attempting to do this and may be useful for erasing targets that have
become unusable due to the following reasons:

- Programming of the FOPT location to disable the RESET pin
- Software disabling of the SWD-DATA or SWD-CLK pins 
- Programming of the low-power modes
- Combinations of the above

Doing any of the above may result in the on-board P&E debugger being unable to re-program the target chip.
The current version of the USBDM software also finds this difficult.

It can also be used with external chips if the 20-pin SWD connector is populated on the FRDM board.

NOTE:
This does not allow permanently secured chips to be unsecured.  Once you've done that it's all over :)

===== How it works =====
Once the firmware is programmed to the debugger chip and the board rebooted the debugger will sit in a tight 
loop attempting to use software reset commands to force the target chip into a reset state. This is a bit of 
a complicated process involving connecting to the target, enabling the debugger interface on the target and
then sending various commands to force the chip into reset.
Normally this process won't work if your have one of the problems listed above.  However, there is a small
window of opportunity during power-on of the chip where it may work - providing it's done fast enough.  
Obviously this then requires you to be able to power-on the target without upsetting the debugger interface.
This requires a link on the FRDM board to be cut and replaced by a manual jumper.

After successfully connecting the debugger will then mass erase the target.

The debugger firmware is based on a stripped down version of USBDM.
  
===== PREREQUISITES =====
** A lightly modified FRDM board.
   The reqired modifications are: 
   - Cutting a link on the FRDM board and replacing with a manual jumper.
   - Removing associated resistors in some cases (0R & 10R)
    Board type  | Jumper to cut
    ===========================
     FRDM-KE02Z |   J3 also remove R11,R12
     FRDM-KL02Z |   J4 also remove R27
     FRDM-KL05Z |   J4 also remove R27
     FRDM-KL25Z |   J4
     FRDM-KL46Z |   J17 also remove R1,R2
     
   The manual jumper can be used to power the target in normal use.

** A wire jumper lead for use when erasing the board - for some reason using a jumper is unreliable.
   
=====  METHOD =====
The attached SREC file is intended to be programmed to the MK20 debugger chip on a FRDM board (Not
the target chip!)  This can be done in the following manner:

* While holding the RESET switch pressed, plug in the FRDM board using the OpenSDA USB connector.
* The FRDM board will then appear as a USB drive.
* Drag the .sx file to the drive to program it to the debugger processor (small MKL20DX128 chip)

===== Doing a Mass erase ====
The following sequence should result in the target chip being mass erased:

* Remove the jumper added above so that the target is not powered.
* Plug in the board using the OpenSDA USB connector.
* The green LED on the board should be flashing at a fast rate with roughly equal on and off times.
  The debugger is now trying to connect to the target
* Close the power jumper pins (see above) using a JUMPER LEAD (not a jumper block).  I have no real idea why a
  jumper block does not work realiably.  When testing I have used a lead or an external switch - they both worked 
  about 90% of the time on my test case.  Using a small jumper block was much less often successful.
* A few moments later the green LED should change to a flashing pattern of long-on, short-off.  This indicates 
  that the mass erase has completed successfully.
  If the pattern changes to long-off, short-on then the process failed and should be repeated from the first step.

  