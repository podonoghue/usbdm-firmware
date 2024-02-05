/** \file
    \brief ARM-SWD low-level interface

   \verbatim

   USBDM
   Copyright (C) 2016  Peter O'Donoghue

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
   \endverbatim

   \verbatim
   Change History
   +=========================================================================================================
   | 21 May 2021 | Kinetis version                                                          V4.12.1.270 - pgo
   +=========================================================================================================
   \endverbatim
 */
#include "delay.h"
#include "interface.h"
#include "resetInterface.h"
#include "spi.h"
#include "commands.h"
#include "console.h"
#include "swd.h"

using namespace USBDM;

namespace Swd {

/** Select SPI to use */
using SpiInfo = Spi0Info;

// Make sure pins have been assigned to SPI
USBDM::PcrBase::CheckSignalMapping<SpiInfo, 0> sck_chk;         // SWCLK_O pin
USBDM::PcrBase::CheckSignalMapping<SpiInfo, 1> sin_chk;         // SWD_I pin
USBDM::PcrBase::CheckSignalMapping<SpiInfo, 2> sout_chk;        // SWD_O pin

/** Manual control of SWDIO */
class SwdInterface {

private:
   /** GPIO for SWD_I = SDA_SPI0_SIN / Shared with SPI */
   using swdIn  = GpioTable_T<SpiInfo, SpiInfo::sinPin, ActiveHigh>;

   /** GPIO for SWD_O = SDA_SPI0_SOUT / Shared with SPI */
   class swdOut : private GpioTable_T<SpiInfo, SpiInfo::soutPin, ActiveHigh> {
   public:

      /**
       * Changes pin to being controlled by the GPIO and set low
       */
      static void setLow() {
         static constexpr PcrInit pinInit(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Fast);
         setOutput(pinInit);
      }
      /**
       * Changes pin to being controlled by the GPIO and set active
       */
      static void setHigh() {
         setActive();
         static constexpr PcrInit pinInit(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Fast);
         setPCR(pinInit);
         setOut();
      }
      /**
       * Changes pin to being controlled by the GPIO and set 3-state (input)
       */
      static void triState() {
         static constexpr PcrInit pinInit(PinPull_Up);
         setInput(pinInit);
      }
   };

   /**
    * GPIO for SDA_SWD_OE_B pin
    * This GPIO controls the 3-state output buffer on the SWD_DIO_TGTMCU line
    */
   class swdDioEnable : private GpioB<0, USBDM::ActiveLow> {
   public:
      /*
       * Enable SWD_O buffer
       */
      static void enable() {
         static constexpr PcrInit pinInit(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Fast);
         setOutput(pinInit);
         on();
      }
      /*
       * Disable SWD_O buffer
       * Note there is a potential conflict as when disabled SDA_SPI0_SOUT will be driven
       */
      static void disable() {
         static constexpr PcrInit pinInit(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Fast);
         setOutput(pinInit);
         off();
      }
   };

public:
   /**
    * Set SWDIO as output driving high
    *
    * @note SWDIO control is transferred from SPI to GPIO
    */
   static void driveHigh() {
      swdOut::setHigh();
      swdDioEnable::enable();
   }

   /**
    * Set SWDIO direction as output driving low
    *
    * @note SWDIO control is transferred from SPI to GPIO
    */
   static void driveLow()  {
      swdOut::setLow();
      swdDioEnable::enable();
   }

   /**
    * Set SWDIO direction as output driving low
    *
    * @note SWDIO control is transferred from SPI to GPIO
    */
   static void triState()  {
      swdOut::triState();
      swdDioEnable::disable();
   }

   /**
    * Read SWDIO level
    *
    * @note SWDIO control is not affected
    */
   static bool peek()  {
      // GPIO PDIR always reflects digital value even when not configured as GPIO
      return swdIn::readBit();
   }
};

/** Manual control of SWCLK */
class SwdClkControl{

   /** GPIO for SWD_CLK = SDA_SPI0_SCK pin / Shared with SPI */
   class swdClk : private GpioTable_T<SpiInfo, SpiInfo::sckPin, ActiveHigh> {
   public:
      using GpioTable_T::readState;

      /**
       * Changes pin to being controlled by the GPIO and set low
       */
      static void setLow() {
         static constexpr PcrInit pinInit(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Fast);
         setOutput(pinInit);
      }
      /**
       * Changes pin to being controlled by the GPIO and set active
       */
      static void setHigh() {
         setActive();
         static constexpr PcrInit pinInit(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Fast);
         setPCR(pinInit);
         setOut();
      }
      /**
       * Changes pin to being controlled by the GPIO and set 3-state (input)
       */
      static void triState() {
         static constexpr PcrInit pinInit(PinPull_Up);
         setPCR(pinInit);
      }
   };

   /**
    * GPIO for SDA_SWD_EN pin
    * This GPIO controls the 3-state buffer on the SWD_CLK_TGTMCU line
    */
   class SwdClkEnable : private GpioA<4, USBDM::ActiveLow> {
   public:
      /*
       * Enable SWD_CLK buffer
       */
      static void enableClkOut() {
         static constexpr PcrInit pinInit(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Fast);
         setOutput(pinInit);
         on();
      }
      /*
       * Disable SWD_CLK buffer
       * Note there is a potential conflict as when disabled SDA_SPI0_SCK will be driven
       */
      static void disableClkOut() {
         static constexpr PcrInit pinInit(PinDriveStrength_High, PinDriveMode_PushPull, PinSlewRate_Fast);
         setOutput(pinInit);
         off();
      }
   };

public:
   /**
    * Enable the buffer associated with the SWD CLK signal
    */
   static void enableBuffer() {
      SwdClkEnable::enableClkOut();
   }

   /**
    * Disable the buffer associated with the SWD CLK signal
    */
   static void disableBuffer() {
      SwdClkEnable::disableClkOut();
   }

   /**
    * Set SWCLK as output driving high
    *
    * @note SWCLK control is transferred to GPIO from SPI
    */
   static void driveHigh() {
      swdClk::setHigh();
      SwdClkEnable::enableClkOut();
   }

   /**
    * Set SWCLK direction as output driving low
    *
    * @note SWCLK control is transferred to GPIO from SPI
    */
   static void driveLow()  {
      swdClk::setLow();
      SwdClkEnable::enableClkOut();
   }

   /**
    * Read SWCLK level
    *
    * @note SWCLK control is not affected
    * @note This reads the value being driven on SWCLK_O which may not always reflect the level on SWDCLK
    */
   static bool peek()  {
      // GPIO PDIR always reflects digital value even when not configured as GPIO
      return swdClk::readState();
   }
};

/**
 * GPIO for SDA_SWD_OE_B pin
 * This GPIO controls the 3-state output buffer on the SWD_DIO_TGTMCU line
 */
class SwdDataBuffer : private GpioB<0, USBDM::ActiveLow> {
public:
   using GpioB::setOutput;
   using GpioB::on;
   using GpioB::off;
};

//===========================================================================

// Masks for SWD_DP_ABORT clear sticky
static constexpr uint32_t  SWD_DP_ABORT_CLEAR_STICKY_ERRORS = 0x0000001E;

// Masks for SWD_DP_ABORT abort AP
static constexpr uint32_t  SWD_DP_ABORT_ABORT_AP            = 0x00000001;

// Masks for SWD_DP_STATUS
//static constexpr uint32_t  SwdRead_DP_STATUS_ANYERROR        = 0x000000B2;

// Masks for SWD_DP_CONTROL
static constexpr uint32_t  SWD_DP_CONTROL_POWER_REQ = (1<<30)|(1<<28);
static constexpr uint32_t  SWD_DP_CONTROL_POWER_ACK = (1<<31)|(1<<29);

// AP number for AHB-AP (MEM-AP implementation)
static constexpr uint32_t  AHB_AP_NUM        = (0x0);

// DP_SELECT register value to access AHB_AP Bank #0 for memory read/write
static constexpr uint32_t ARM_AHB_AP_BANK0 = AHB_AP_NUM;

//   static constexpr uint32_t  AHB_CSW_REGNUM    = (0x0);  // CSW register bank+register number
//   static constexpr uint32_t  AHB_TAR_REGNUM    = (0x4);  // TAR register bank+register number
//   static constexpr uint32_t  AHB_DRW_REGNUM    = (0xC);  // DRW register bank+register number


// AHB-AP (MEM-AP) CSW Register masks
static constexpr uint32_t  AHB_AP_CSW_INC_SINGLE    = (1<<4);
//static constexpr uint32_t  AHB_AP_CSW_INC_PACKED    = (2<<4);
//static constexpr uint32_t  AHB_AP_CSW_INC_MASK      = (3<<4);
static constexpr uint32_t  AHB_AP_CSW_SIZE_BYTE     = (0<<0);
static constexpr uint32_t  AHB_AP_CSW_SIZE_HALFWORD = (1<<0);
static constexpr uint32_t  AHB_AP_CSW_SIZE_WORD     = (2<<0);
//static constexpr uint32_t  AHB_AP_CSW_SIZE_MASK     = (7<<0);

// CTAR values used for communication
uint32_t PreambleCtar;  //< Transmit  8 bits = Write:start,APnDP,RnW,ADDR(2:3),parity,stop,park
uint32_t AckCtar;       //< Receive   5 bits = Read:T,ACK,Data(0) or Write:T,ACK,T
uint32_t RxCtar;        //< Receive  16 bits = Read:Data(1-16) or Read:Data(17-31),parity
uint32_t TxCtar;        //< Transmit 11 bits = Write:Data(0-10) or Write:Data(11-21) or Write:Data(22-31),parity
uint32_t Tx16Ctar;      //< Transmit 16 bits = Write:Data(15-0)

static constexpr uint32_t cswValues[5] = {
      0,
      AHB_AP_CSW_SIZE_BYTE    |AHB_AP_CSW_INC_SINGLE,
      AHB_AP_CSW_SIZE_HALFWORD|AHB_AP_CSW_INC_SINGLE,
      0,
      AHB_AP_CSW_SIZE_WORD    |AHB_AP_CSW_INC_SINGLE,
};

/**
 * Get AHB.CSW value based on size
 *
 * @param size Transfer size in bytes (one of 1,2 or 4)
 *
 * @return AHB.CSW mask.  This will include size and increment.
 */
static constexpr uint32_t getcswValue(int size) {
   return cswValues[size];
};

   /**
    * SWD ACK values padded to 5 bits e.g. 0 <ACK value> 0
    */
enum SwdAck {
   /** OK ACK value. Indicates SWD transfer was successful */
   SWD_ACK_OK    = 0b01000,/**< SWD_ACK_OK */
   /** WAIT ACK value. Indicates SWD transfer cannot complete yet. Should be retried */
   SWD_ACK_WAIT  = 0b00100,/**< SWD_ACK_WAIT */
   /** FAULT ACK value. Indicates SWD transfer failed. */
   SWD_ACK_FAULT = 0b00010,/**< SWD_ACK_FAULT */
   /** Mask to indicate valid ACK bits in return value */
   SW_ACK_MASK   = 0b01110, /**< SW_ACK_MASK */
};

/** SPI Object */
static const HardwarePtr<SPI_Type> spi = SpiInfo::baseAddress;

/** Initial value of AHB_SP_CSW register read from target */
static uint32_t ahb_ap_csw_defaultValue;

/**
 * Calculate parity of a 32-bit value
 *
 * @param  data Data value
 *
 * @return parity value (0/1)
 */
__attribute__((naked))
static uint8_t calcParity(const uint32_t data) {
   (void)data;
   __asm__ volatile (
         "eor.w r0, r0, r0, lsr #16  \n\t"
         "eor.w r0, r0, r0, lsr #8   \n\t"
         "eor.w r0, r0, r0, lsr #4   \n\t"
         "eor.w r0, r0, r0, lsr #2   \n\t"
         "eor.w r0, r0, r0, lsr #1   \n\t"
         "and.w r0, #1               \n\t"
         "bx lr                      \n\t"
   );
   return 0; // prevent warning
}

/**
 *  Transmit [32-bit word]
 *
 *  @param data Data to send
 */
static void tx32(const uint32_t data) {
   spi->CTAR[0] = Tx16Ctar;

//   spi->MCR &= ~SPI_MCR_HALT_MASK;

   spi->SR = SPI_SR_TCF_MASK|SPI_SR_EOQF_MASK;

   SwdDataBuffer::on();
   spi->PUSHR = ((uint16_t)(data))    |SPI_PUSHR_CTAS(0)|SPI_PUSHR_CONT(1)|SPI_PUSHR_EOQ(0);
   spi->PUSHR = ((uint16_t)(data>>16))|SPI_PUSHR_CTAS(0)|SPI_PUSHR_CONT(0)|SPI_PUSHR_EOQ(1);

   // Wait until End of Transmission
   while ((spi->SR & SPI_SR_EOQF_MASK)==0) {
   }
   SwdDataBuffer::off();

   (void)(spi->POPR);
   (void)(spi->POPR);

   spi->SR = SPI_SR_TCF_MASK|SPI_SR_EOQF_MASK;
}

/**
 * Sets Communication speed for SPI
 *
 * @param frequency Frequency in Hz
 *
 * @return BDM_RC_OK success
 *
 * Note: Chooses the highest speed that is not greater than frequency.
 */
USBDM_ErrorCode setSpeed(uint32_t frequency) {
   const uint32_t ctarBase = Spi::calculateCtarTiming(SpiInfo::getClockFrequency(), frequency);

   PreambleCtar = ctarBase|SpiMode_3|SpiFrameSize_8_bits |SpiBitOrder_MsbFirst;
   AckCtar      = ctarBase|SpiMode_2|SpiFrameSize_5_bits |SpiBitOrder_MsbFirst;
   RxCtar       = ctarBase|SpiMode_2|SpiFrameSize_16_bits|SpiBitOrder_LsbFirst;
   TxCtar       = ctarBase|SpiMode_3|SpiFrameSize_11_bits|SpiBitOrder_LsbFirst;
   Tx16Ctar     = ctarBase|SpiMode_3|SpiFrameSize_16_bits|SpiBitOrder_LsbFirst;

//   console.setPadding(Padding_LeadingZeroes).setWidth(32);
//   console.writeln("PreambleCtar 0b", PreambleCtar, Radix_2);
//   console.writeln("AckCtar      0b", AckCtar, Radix_2);
//   console.writeln("RxCtar       0b", RxCtar, Radix_2);
//   console.writeln("TxCtar       0b", TxCtar, Radix_2);
//   console.writeln("Tx16Ctar     0b", Tx16Ctar, Radix_2);

   return BDM_RC_OK;
}

/**
 * Gets Communication speed of SWD
 *
 * @return frequency Frequency in Hz
 *
 * @note This may differ from set speed due to limited range of speeds available
 */
uint32_t getSpeed() {
   return Spi::calculateSpeed(SpiInfo::getClockFrequency(), TxCtar);
}

/**
 * Obtain default AHB_AP.csw register default value from target
 *
 * @return BDM_RC_OK ahb_ap_csw_defaultValue already valid or successfully updated, error otherwise
 */
static USBDM_ErrorCode update_ahb_ap_csw_defaultValue() {

   if (ahb_ap_csw_defaultValue != 0) {
      return BDM_RC_OK;
   }

   // Read initial AHB-AP.csw register value as device dependent
   // Do posted read - dummy data returned
   uint32_t ahb_ap_cswValue = 0;
   USBDM_ErrorCode rc = readReg(SwdRead_AHB_CSW, ahb_ap_cswValue);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Get actual data
   rc = readReg(SwdRead_DP_RDBUFF, ahb_ap_cswValue);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Modify value - preserve some bits
   ahb_ap_csw_defaultValue = (ahb_ap_cswValue & 0xFF000000) | 0x00000040;
   return BDM_RC_OK;
}

/**
 * Set pin state
 *
 * @param pins Pin control mask
 *
 * @note Only handles SWD, SWCLK functions as others (such as reset) are assumed handled in common code
 * @note Transfers SWD and SWDCLK control to GPIO from SPI. Use @ref interfaceIdle() to return control to SPI.
 */
void setPinState(PinLevelMasks_t control) {
   switch (control & PIN_SWD_MASK) {
      case PIN_SWD_3STATE :
         SwdInterface::triState();
         break;
      case PIN_SWD_LOW :
         SwdInterface::driveLow();
         break;
      case PIN_SWD_HIGH :
         SwdInterface::driveHigh();
         break;
   }
   switch (control & PIN_SWCLK_MASK) {
      case PIN_SWCLK_3STATE :
         // Not supported as fixed buffer between pin and T_SWCLK - approximate as high
         // No break
      case PIN_SWCLK_HIGH :
         SwdClkControl::driveHigh();      // Drive high
         break;
      case PIN_SWCLK_LOW :
         SwdClkControl::driveLow();      // Drive low
         break;
   }
}

/**
 * Get pin status
 *
 * @return Pin status from this interface
 */
PinLevelMasks_t getPinState() {
   return (PinLevelMasks_t) ((SwdInterface::peek()?PIN_SWD_LOW:PIN_SWD_LOW) | (SwdClkControl::peek()?PIN_SWD_LOW:PIN_SWD_LOW));
}

/**
 * Initialise interface                \n
 * Does not communicate with target    \n
 * Reset is initially 3-state
 */
void initialiseInterface() {

   console.WRITELN("initialiseInterface()");

   SpiInfo::enableClock();

   setSpeed(12000000);

   // Configure reset signal
   ResetInterface::initialise();

   // GPIO controlling some interface signals (SWD, UART-TX)
   InterfaceEnable::initialise();
   InterfaceEnable::on();

   interfaceIdle();

   // Set mode
   spi->MCR =
         SPI_MCR_FRZ(1)|      // Freeze in debug mode
         SPI_MCR_CLR_RXF(1)|  // Clear Receive FIFO
         SPI_MCR_CLR_TXF(1)|  // Clear Transmit FIFO
         SPI_MCR_ROOE(0)|     // Ignore Receive data on overflow
         SPI_MCR_MSTR(1)|     // Master mode
         SPI_MCR_DCONF(0)|    // SPI (must be zero)
         SPI_MCR_MTFE(0)|     // Don't use modified sample point
         SPI_MCR_SMPL_PT(0)|  // Modified sample point (N/A)
         SPI_MCR_PCSIS(0);    // All PCSes active-high
}

/**
 * Initialise interface                   \n
 * Does not communicate with target       \n
 * Reset is 3-state                       \n
 * SWD signals are controlled by SPI
 */
void interfaceIdle() {

//   console.WRITELN("interfaceIdle()");

   // Configure RESET pins
   ResetInterface::highZ();

   // Configure SPI pins SIN,SOUT,SCK,CS (SWDIO, SWDCLK)
   SpiInfo::initPCRs();

   SwdClkControl::enableBuffer();
   SwdDataBuffer::setOutput();
   SwdDataBuffer::on();
}

/**
 * Disables interface         \n
 * SPI is disconnected from SWD signals
 *
 * Note: Reset is not affected
 */
void disableInterface() {
   SpiInfo::clearPCRs();
   SwdClkControl::disableBuffer();
   SwdDataBuffer::off();
}


/**
 *  Switches interface to SWD and confirm connection to target
 *
 *  Reference ARM Debug Interface v5 Architecture Specification
 *            ADIv5.1 Supplement - 6.2.1 JTAG to Serial Wire switching
 *
 *  Sequence as follows:
 *   - >=50-bit sequence of 1's
 *   - 16-bit magic number 0xE79E
 *   - >=50-bit sequence of 1's
 *   - 8-bit idle
 *   - Read IDCODE
 *
 *  @return BDM_RC_OK => Success
 */
USBDM_ErrorCode connect(void) {
   ahb_ap_csw_defaultValue = 0;

   tx32(0xFFFFFFFF);  // 32 1's
   tx32(0x79EFFFFF);  // 20 1's + 0x79E
   tx32(0xFFFFFFFE);  // 0xE + 28 1's
   tx32(0x00FFFFFF);  // 24 1's + 8 0's

   // Target must respond to read IDCODE immediately
   uint32_t buff;
   return readReg(SwdRead_DP_IDCODE, buff);
}

/**
 * Power up debug interface and system\n
 * Sets CSYSPWRUPREQ and CDBGPWRUPREQ\n
 * Confirms CSYSPWRUPACK and CDBGPWRUPACK
 *
 *  @return \n
 *  @return BDM_RC_OK => Success
 */
USBDM_ErrorCode powerUp() {
   USBDM_ErrorCode rc;
   rc = writeReg(SwdWrite_DP_CONTROL, SWD_DP_CONTROL_POWER_REQ);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   uint32_t status;
   rc = readReg(SwdRead_DP_STATUS, status);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   return ((status&SWD_DP_CONTROL_POWER_ACK) == SWD_DP_CONTROL_POWER_ACK)?BDM_RC_OK:BDM_RC_ARM_PWR_UP_FAIL;
}

/**
 * Resets SWD interface
 *
 *  Sequence as follows:
 *   - >=50-bit sequence of 1's (55 0's)
 *   - >=8-bit sequence of 0's  (9 1's)
 *   - Read IDCODE
 *
 *  @return BDM_RC_OK => Success, error otherwise
 */
USBDM_ErrorCode lineReset(void) {
   tx32(0xFFFFFFFF);  // 32 1's
   tx32(0x007FFFFF);  // 23 1's, 9 0's

   // Target must respond to read IDCODE immediately
   uint32_t buff;
   return readReg(SwdRead_DP_IDCODE, buff);
}

/**
 *  Read ARM-SWD DP & AP register
 *
 *  @param swdRead - SWD command byte to select register etc.
 *  @param data    - 32-bit value read
 *
 *  @return BDM_RC_OK               => Success        \n
 *  @return BDM_RC_ARM_FAULT_ERROR  => FAULT response from target \n
 *  @return BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
 *  @return BDM_RC_NO_CONNECTION    => Unexpected/no response from target \n
 *  @return BDM_RC_ARM_PARITY_ERROR => Parity error on data read
 *
 *  @note Action and Data returned depends on register (some responses are pipelined)\n
 *    SwdRead_DP_IDCODE - Value from IDCODE reg \n
 *    SwdRead_DP_STATUS - Value from STATUS reg \n
 *    SwdRead_DP_RESEND - LAST value read (AP read or DP-RDBUFF), FAULT on sticky error    \n
 *    SwdRead_DP_RDBUFF - Value from last AP read and clear READOK flag in STRL/STAT, FAULT on sticky error \n
 *    SwdRead_AP_REGx   - Value from last AP read, clear READOK flag in STRL/STAT and INITIATE next AP read, FAULT on sticky error
 */
USBDM_ErrorCode readReg(const SwdRead swdRead, uint32_t &data) {

   spi->CTAR[0] = PreambleCtar;
   spi->CTAR[1] = AckCtar;

//   spi->MCR &= ~SPI_MCR_HALT_MASK;

   spi->SR = SPI_SR_TCF_MASK|SPI_SR_EOQF_MASK;

   unsigned        retry = 2000;
   USBDM_ErrorCode rc    = BDM_RC_OK;

   do {
      // Write SWD command
      SwdDataBuffer::on();
      spi->PUSHR = swdRead|SPI_PUSHR_CTAS(0)|SPI_PUSHR_CONT(0)|SPI_PUSHR_EOQ(1);

      // Wait until End of Transmission
      while ((spi->SR & SPI_SR_EOQF_MASK)==0) {
      }
      spi->SR = SPI_SR_TCF_MASK|SPI_SR_EOQF_MASK;
      SwdDataBuffer::off();

      // Read ACK
      spi->PUSHR = 0b00000|SPI_PUSHR_CTAS(1)|SPI_PUSHR_CONT(0)|SPI_PUSHR_EOQ(1);

      // Wait until End of Transmission
      while ((spi->SR & SPI_SR_EOQF_MASK)==0) {
      }
      spi->SR = SPI_SR_TCF_MASK|SPI_SR_EOQF_MASK;

      (void)(spi->POPR);
      uint8_t value = (spi->POPR);
      SwdAck ack = (SwdAck)(value & SW_ACK_MASK);

      data = value & 0b00001; // Data[0] is captured as part of the ACK read

      if (ack == SWD_ACK_OK) {

         // OK receive the rest of the data
         spi->CTAR[1] = RxCtar;
         spi->PUSHR = 0xFFFF|SPI_PUSHR_CTAS(1)|SPI_PUSHR_CONT(0)|SPI_PUSHR_EOQ(0);
         spi->PUSHR = 0xFFFF|SPI_PUSHR_CTAS(1)|SPI_PUSHR_CONT(0)|SPI_PUSHR_EOQ(1);

         // Wait until End of Transmission
         while ((spi->SR & SPI_SR_EOQF_MASK)==0) {
         }
         spi->SR = SPI_SR_TCF_MASK|SPI_SR_EOQF_MASK;

         // Transmit 8 bits idle
         SwdDataBuffer::on();
         spi->PUSHR = 0b00000000|SPI_PUSHR_CTAS(0)|SPI_PUSHR_CONT(0)|SPI_PUSHR_EOQ(1);

         // This can overlap with idle transmission
         data |= (spi->POPR)<<1;       // Data[1-16]
         uint32_t temp = (spi->POPR);  // Data[17-31],Parity
         data |= temp<<17;
         uint8_t receivedParity = temp>>15;
         uint8_t calculatedparity = calcParity(data);
         if (receivedParity != calculatedparity) {
            rc = BDM_RC_ARM_PARITY_ERROR;
         }
         // Wait until End of Idle Transmission
         while ((spi->SR & SPI_SR_EOQF_MASK)==0) {
         }
         SwdDataBuffer::off();
         spi->SR = SPI_SR_TCF_MASK|SPI_SR_EOQF_MASK;

         // Discard idle Rx
         (void)(spi->POPR);
      }
      else if (ack == SWD_ACK_WAIT) {
         if (retry-->0) {
            continue;
         }
         rc = BDM_RC_ACK_TIMEOUT;
      }
      else if (ack == SWD_ACK_FAULT) {
         rc = BDM_RC_ARM_FAULT_ERROR;
      }
      else {
         rc = BDM_RC_NO_CONNECTION;
      }
      break;
   } while (true);

//   spi->MCR |= SPI_MCR_HALT_MASK;

   return rc;
}

/**
 *  Write ARM-SWD DP & AP register
 *
 *  @param swdWrite - SWD command byte to select register etc.
 *  @param data     - 32-bit value to write
 *
 *  @return BDM_RC_OK               => Success        \n
 *  @return BDM_RC_ARM_FAULT_ERROR  => FAULT response from target \n
 *  @return BDM_RC_ACK_TIMEOUT      => Excessive number of WAIT responses from target \n
 *  @return BDM_RC_NO_CONNECTION    => Unexpected/no response from target
 *
 *  @note Action depends on register (some responses are pipelined)\n
 *    SwdWrite_DP_ABORT   - Write value to ABORT register (accepted) \n
 *    SwdWrite_DP_CONTROL - Write value to CONTROL register (may be pending), FAULT on sticky error. \n
 *    SwdWrite_DP_SELECT  - Write value to SELECT register (may be pending), FAULT on sticky error. \n
 *    SwdWrite_AP_REGx    - Write to AP register.  May initiate action e.g. memory access.  Result is pending, FAULT on sticky error.
 */
USBDM_ErrorCode writeReg(const SwdWrite swdWrite, const uint32_t data) {

   spi->CTAR[0] = PreambleCtar;
   spi->CTAR[1] = AckCtar;

//   spi->MCR &= ~SPI_MCR_HALT_MASK;

   unsigned        retry = 2000;
   USBDM_ErrorCode rc    = BDM_RC_OK;

   spi->SR = SPI_SR_TCF_MASK|SPI_SR_EOQF_MASK;

   do {
      SwdDataBuffer::on();
      spi->PUSHR = swdWrite  |SPI_PUSHR_CTAS(0)|SPI_PUSHR_CONT(0)|SPI_PUSHR_EOQ(1);
      // Wait until End of Transmission
      while ((spi->SR & SPI_SR_EOQF_MASK)==0) {
      }
      SwdDataBuffer::off();
      spi->SR = SPI_SR_TCF_MASK|SPI_SR_EOQF_MASK;

      spi->PUSHR = SWD_ACK_OK|SPI_PUSHR_CTAS(1)|SPI_PUSHR_CONT(0)|SPI_PUSHR_EOQ(1);

      // Wait until End of Transmission
      while ((spi->SR & SPI_SR_EOQF_MASK)==0) {
      }
      spi->SR = SPI_SR_TCF_MASK|SPI_SR_EOQF_MASK;

      (void)(spi->POPR);
      SwdAck ack = (SwdAck)(spi->POPR & SW_ACK_MASK);

      if (ack == SWD_ACK_OK) {
         // OK send the data
         uint32_t parity = calcParity(data)?(1<<10):0;

         spi->CTAR[1] = TxCtar;

         // Transmit 3x11 bits = Write:Data(0-10) or Write:Data(11-21) or Write:Data(22-31),parity
         SwdDataBuffer::on();
         spi->PUSHR = ((uint16_t)(data>>0))|SPI_PUSHR_CTAS(1)|SPI_PUSHR_CONT(1)|SPI_PUSHR_EOQ(0);
         spi->PUSHR = ((uint16_t)(data>>11))|SPI_PUSHR_CTAS(1)|SPI_PUSHR_CONT(1)|SPI_PUSHR_EOQ(0);
         spi->PUSHR = ((uint16_t)(data>>22))|parity|SPI_PUSHR_CTAS(1)|SPI_PUSHR_CONT(0)|SPI_PUSHR_EOQ(0);
         // Transmit 8 bits idle
         spi->PUSHR = 0b00000000|SPI_PUSHR_CTAS(0)|SPI_PUSHR_CONT(0)|SPI_PUSHR_EOQ(1);

         // Wait until End of Transmission
         while ((spi->SR & SPI_SR_EOQF_MASK)==0) {
         }
         spi->SR = SPI_SR_TCF_MASK|SPI_SR_EOQF_MASK;
         SwdDataBuffer::off();

         (void)(spi->POPR);
         (void)(spi->POPR);
         (void)(spi->POPR);
         (void)(spi->POPR);
      }
      else if (ack == SWD_ACK_WAIT) {
         if (retry-->0) {
            continue;
         }
         rc = BDM_RC_ACK_TIMEOUT;
      }
      else if (ack == SWD_ACK_FAULT) {
         rc = BDM_RC_ARM_FAULT_ERROR;
      }
      else {
         rc = BDM_RC_NO_CONNECTION;
      }
      break;
   } while (true);

   SwdDataBuffer::off();

//   spi->MCR |= SPI_MCR_HALT_MASK;

   return rc;
}

/**
 *  Read register of Access Port
 *
 *  @param address 32-bit address \n
 *     A[31:24] => DP-AP-SELECT[31:24] (AP # Select) \n
 *     A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
 *     A[3:2]   => APACC[3:2]          (Register select within bank)
 *  @param buff 32-bit register value
 *
 *  @return BDM_RC_OK => success
 *
 *  @note - Access is completed before return
 */
USBDM_ErrorCode readAPReg(const uint32_t address, uint32_t &buff) {
   static const uint8_t readAP[]  = {SwdRead_AP_REG0,   SwdRead_AP_REG1,    SwdRead_AP_REG2,   SwdRead_AP_REG3};
   USBDM_ErrorCode rc;
   SwdRead  regNo      = (SwdRead)readAP[(address>>2)&0x3];
   uint32_t selectData = address&0xFF0000F0;

   // Set up SELECT register for AP access
   rc = writeReg(SwdWrite_DP_SELECT, selectData);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Initiate read from AP register (dummy data)
   rc = readReg(regNo, buff);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Read from READBUFF register
   return readReg(SwdRead_DP_RDBUFF, buff);
}

/**
 *  Write Access Port register
 *
 *  @param address 16-bit address \n
 *     A[15:8]  => DP-AP-SELECT[31:24] (AP # Select) \n
 *     A[7:4]   => DP-AP-SELECT[7:4]   (Bank select within AP) \n
 *     A[3:2]   => APACC[3:2]          (Register select within bank)
 *  @param data 32-bit register value
 *
 *  @return BDM_RC_OK => success
 *
 *  @note - Access is completed before return
 */
USBDM_ErrorCode writeAPReg(const uint32_t address, const uint32_t data) {
   static const uint8_t writeAP[] = {SwdWrite_AP_REG0,   SwdWrite_AP_REG1,    SwdWrite_AP_REG2,   SwdWrite_AP_REG3};
   USBDM_ErrorCode rc;
   SwdWrite regNo      = (SwdWrite)writeAP[(address>>2)&0x3];
   uint32_t selectData = address&0xFF0000F0;

   // Set up SELECT register for AP access
   rc = writeReg(SwdWrite_DP_SELECT, selectData);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Initiate write to AP register
   rc = writeReg(regNo, data);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Read from READBUFF register to allow stall/status response
   return readReg(SwdRead_DP_RDBUFF, selectData);
}

/**
 *  Clear all sticky bits in status register
 *
 *  @return error code
 */
USBDM_ErrorCode clearStickyBits(void) {
   return writeReg(SwdWrite_DP_ABORT, SWD_DP_ABORT_CLEAR_STICKY_ERRORS);
}

/**
 * Clear sticky bits and abort AP transactions
 *
 *  @return error code
 */
USBDM_ErrorCode abortAP(void) {
   return writeReg(SwdWrite_DP_ABORT, SWD_DP_ABORT_CLEAR_STICKY_ERRORS|SWD_DP_ABORT_ABORT_AP);
}

static constexpr uint32_t  MDM_AP_STATUS                     = 0x01000000;
static constexpr uint32_t  MDM_AP_CONTROL                    = 0x01000004;
//static constexpr uint32_t  MDM_AP_IDR                        = 0x010000FC;

static constexpr uint32_t  MDM_AP_CONTROL_MASS_ERASE_REQUEST = (1<<0);
//static constexpr uint32_t  MDM_AP_CONTROL_DEBUG_REQUEST      = (1<<2);
static constexpr uint32_t  MDM_AP_CONTROL_RESET_REQUEST      = (1<<3);
//static constexpr uint32_t  MDM_AP_CONTROL_VLLDBGREQ          = (1<<5);
//static constexpr uint32_t  MDM_AP_CONTROL_VLLDBGACK          = (1<<6);
//static constexpr uint32_t  MDM_AP_CONTROL_LLS_VLLSx_ACK      = (1<<7);

//static constexpr uint32_t  MDM_AP_STATUS_FLASH_READY         = (1<<1);
static constexpr uint32_t  MDM_AP_STATUS_SECURE              = (1<<2);
static constexpr uint32_t  MDM_AP_STATUS_MASS_ERASE_ENABLE   = (1<<5);

static constexpr uint32_t  ATTEMPT_MULTIPLE                  = 100;  // How many times to attempt mass erase
static constexpr uint32_t  ERASE_MULTIPLE                    = 2;    // How many times to mass erase

/**
 * Mass erase target
 *
 * @return BDM_RC_OK if successful
 */
USBDM_ErrorCode kinetisMassErase(void) {
   unsigned successCount = 0;
   unsigned attemptCount = 0;
   uint8_t rc;
   uint32_t valueRead;

   ResetInterface::low();
   for (;;) {
      UsbLed::on();
      if (attemptCount++>ATTEMPT_MULTIPLE) {
         break;
      }
      // Do connect sequence
      rc = connect();
      if (rc != BDM_RC_OK) {
         continue;
      }
      rc = clearStickyBits();
      if (rc != BDM_RC_OK) {
         continue;
      }
      // Power up Debug interface
      rc = powerUp();
      if (rc != BDM_RC_OK) {
         continue;
      }
      // Do mass erase
      rc = writeAPReg(MDM_AP_CONTROL, MDM_AP_CONTROL_RESET_REQUEST|MDM_AP_CONTROL_MASS_ERASE_REQUEST);
      if (rc != BDM_RC_OK) {
         continue;
      }
      // Check if mass erase commenced
      rc = readAPReg(MDM_AP_CONTROL, valueRead);
      if (rc != BDM_RC_OK) {
         continue;
      }
      if ((valueRead&MDM_AP_CONTROL_MASS_ERASE_REQUEST) == 0) {
         continue;
      }
      UsbLed::off();
      // Wait until complete
      for (int eraseWait=0; eraseWait<20; eraseWait++) {
         waitMS(100);
         rc = readAPReg(MDM_AP_CONTROL, valueRead);
         if (rc != BDM_RC_OK) {
            continue;
         }
         if ((valueRead&MDM_AP_CONTROL_MASS_ERASE_REQUEST) == 0) {
            break;
         }
      }
      rc = readAPReg(MDM_AP_STATUS, valueRead);
      if (rc != BDM_RC_OK) {
         continue;
      }
      // Check if mass erase is disabled
      if ((valueRead&MDM_AP_STATUS_MASS_ERASE_ENABLE) == 0) {
         return BDM_RC_MASS_ERASE_DISABLED;
      }
      rc = ((valueRead&MDM_AP_STATUS_SECURE) == 0)?BDM_RC_OK:BDM_RC_FAIL;
      if (rc == BDM_RC_OK) {
         successCount++;
      }
      // Only consider successful if done ERASE_MULTIPLE times in a row
      // This prevents mass-erase attempts during power-on bounces etc.
      if (successCount>=ERASE_MULTIPLE) {
         break;
      }
   }
   return (successCount>=ERASE_MULTIPLE)?BDM_RC_OK:BDM_RC_FAIL;
}

/** Write 32-bit value to ARM-SWD Memory
 *
 *  @param address 32-bit memory address
 *  @param data    32-bit data value
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode writeMemoryWord(const uint32_t address, const uint32_t data) {
   USBDM_ErrorCode  rc;
   /* Steps
    *  - Set up to access AHB-AP register bank 0 (CSW,TAR,DRW)
    *  - Write AP-CSW value (auto-increment etc)
    *  - Write AP-TAR value (target memory address)
    *  - Write value to DRW (data value to target memory)
    */
   // Select AHB-AP memory bank - subsequent AHB-AP register accesses are all in the same bank
   rc = writeReg(SwdWrite_DP_SELECT, ARM_AHB_AP_BANK0);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   rc = update_ahb_ap_csw_defaultValue();
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write CSW (word access etc)
   rc = writeReg(SwdWrite_AHB_CSW, ahb_ap_csw_defaultValue|AHB_AP_CSW_SIZE_WORD);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write TAR (target address)
   rc = writeReg(SwdWrite_AHB_TAR, address);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write data value
   rc = writeReg(SwdWrite_AHB_DRW, data);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Dummy read to get status
   uint32_t tt;
   return readReg(SwdRead_DP_RDBUFF, tt);
}

/**  Write ARM-SWD Memory
 *
 *  @param elementSize  Size of the data elements
 *  @param count        Number of data bytes
 *  @param addr         LSB of Address in target memory
 *  @param data_ptr     Where in buffer to write the data
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode writeMemory(
      uint32_t  elementSize,
      uint32_t  count,
      uint32_t  addr,
      uint8_t   *data_ptr
) {
   USBDM_ErrorCode  rc;
   uint8_t  temp[4] ={0};

   /* Steps
    *  - Set up to access AHB-AP register bank 0 (CSW,TAR,DRW)
    *  - Write AP-CSW value (auto-increment etc)
    *  - Write AP-TAR value (target memory address)
    *  - Loop
    *    - Pack data
    *    - Write value to DRW (data value to target memory)
    */
   // Select AHB-AP memory bank - subsequent AHB-AP register accesses are all in the same bank
   rc = writeReg(SwdWrite_DP_SELECT, ARM_AHB_AP_BANK0);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   rc = update_ahb_ap_csw_defaultValue();
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write CSW (auto-increment etc)
   rc = writeReg(SwdWrite_AHB_CSW, ahb_ap_csw_defaultValue|getcswValue(elementSize));
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write TAR (target address)
   rc = writeReg(SwdWrite_AHB_TAR, addr);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   switch (elementSize) {
   case MS_Byte:
      while (count > 0) {
         switch (addr&0x3) {
         case 0: temp[3] = *data_ptr++; break;
         case 1: temp[2] = *data_ptr++; break;
         case 2: temp[1] = *data_ptr++; break;
         case 3: temp[0] = *data_ptr++; break;
         }
         rc = writeReg(SwdWrite_AHB_DRW, temp);
         if (rc != BDM_RC_OK) {
            return rc;
         }
         addr++;
         count--;
      }
      break;
   case MS_Word:
      count >>= 1;
      while (count > 0) {
         switch (addr&0x2) {
         case 0:  temp[3] = *data_ptr++;  temp[2] = *data_ptr++; break;
         case 2:  temp[1] = *data_ptr++;  temp[0] = *data_ptr++; break;
         }
         rc = writeReg(SwdWrite_AHB_DRW, temp);
         if (rc != BDM_RC_OK) {
            return rc;
         }
         addr  += 2;
         count--;
      }
      break;
   case MS_Long:
      count >>= 2;
      while (count-- > 0) {
         temp[3] = *data_ptr++;
         temp[2] = *data_ptr++;
         temp[1] = *data_ptr++;
         temp[0] = *data_ptr++;
         rc = writeReg(SwdWrite_AHB_DRW, temp);
         if (rc != BDM_RC_OK) {
            return rc;
         }
      }
      break;
   }
   // Dummy read to obtain status from last write
   return readReg(SwdRead_DP_RDBUFF, temp);
}

/** Read 32-bit value from ARM-SWD Memory
 *
 *  @param address 32-bit memory address
 *  @param data    32-bit data value from _last_ read in BIG-ENDIAN order!
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode readMemoryWord(const uint32_t address, uint32_t &data) {
   USBDM_ErrorCode  rc;

   /* Steps
    *  - Set up to DP_SELECT to access AHB-AP register bank 0 (CSW,TAR,DRW)
    *  - Write AP-CSW value (auto-increment etc)
    *  - Write AP-TAR value (starting target memory address)
    *  - Initiate read by reading from DRW (dummy value)
    *  - Read data value from DP-READBUFF
    */
   // Select AHB-AP memory bank - subsequent AHB-AP register accesses are all in the same bank
   rc = writeReg(SwdWrite_DP_SELECT, ARM_AHB_AP_BANK0);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   rc = update_ahb_ap_csw_defaultValue();
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write memory access control to CSW
   rc = writeReg(SwdWrite_AHB_CSW, ahb_ap_csw_defaultValue|AHB_AP_CSW_SIZE_WORD);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write TAR (target address)
   rc = writeReg(SwdWrite_AHB_TAR, address);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   uint32_t tt;
   // Initial read of DRW (dummy data)
   rc = readReg(SwdRead_AHB_DRW, tt);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Read memory data
   return readReg(SwdRead_DP_RDBUFF, data);
}

/**  Read ARM-SWD Memory
 *
 *  @param elementSize  Size of the data elements
 *  @param count        Number of data bytes
 *  @param addr         LSB of Address in target memory
 *  @param data_ptr     Where in buffer to write the data
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode readMemory(uint32_t elementSize, int count, uint32_t addr, uint8_t *data_ptr) {
   USBDM_ErrorCode  rc;
   uint8_t  temp[4];

   /* Steps
    *  - Set up to DP_SELECT to access AHB-AP register bank 0 (CSW,TAR,DRW)
    *  - Write AP-CSW value (auto-increment etc)
    *  - Write AP-TAR value (starting target memory address)
    *  - Loop
    *    - Read value from DRW (data value from target memory)
    *      Note: 1st value read from DRW is discarded
    *      Note: Last value is read from DP-READBUFF
    *    - Copy to buffer adjusting byte order
    */
   if (count>MAX_COMMAND_SIZE-1) {
      return BDM_RC_ILLEGAL_PARAMS;  // requested block+status is too long to fit into the buffer
   }
#ifdef HACK
   {
      uint32_t address = (commandBuffer[4]<<24)+(commandBuffer[5]<<16)+(commandBuffer[6]<<8)+commandBuffer[7];
      memcpy(data_ptr, (void*)address, count);
      returnSize = count+1;
      return BDM_RC_OK;
   }
#else
   // Select AHB-AP memory bank - subsequent AHB-AP register accesses are all in the same bank
   rc = writeReg(SwdWrite_DP_SELECT, ARM_AHB_AP_BANK0);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   rc = update_ahb_ap_csw_defaultValue();
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write CSW (auto-increment etc)
   rc = writeReg(SwdWrite_AHB_CSW, ahb_ap_csw_defaultValue|getcswValue(elementSize));
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Write TAR (target address)
   rc = writeReg(SwdWrite_AHB_TAR, addr);
   if (rc != BDM_RC_OK) {
      return rc;
   }

   // Initial read of DRW (dummy data)
   rc = readReg(SwdRead_AHB_DRW, temp);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   switch (elementSize) {
   case MS_Byte:
      do {
         count--;
         if (count == 0) {
            // Read data from RDBUFF for final read
            rc = readReg(SwdRead_DP_RDBUFF, temp);
         }
         else {
            // Start next read and collect data from last read
            rc = readReg(SwdRead_AHB_DRW, temp);
         }
         if (rc != BDM_RC_OK) {
            return rc;
         }
         // Save data
         switch (addr&0x3) {
         case 0: *data_ptr++ = temp[3];  break;
         case 1: *data_ptr++ = temp[2];  break;
         case 2: *data_ptr++ = temp[1];  break;
         case 3: *data_ptr++ = temp[0];  break;
         }
         addr++;
      } while (count > 0);
      break;
   case MS_Word:
      count >>= 1;
      do {
         count--;
         if (count == 0) {
            // Read data from RDBUFF for final read
            rc = readReg(SwdRead_DP_RDBUFF, temp);
         }
         else {
            // Start next read and collect data from last read
            rc = readReg(SwdRead_AHB_DRW, temp);
         }
         if (rc != BDM_RC_OK) {
            return rc;
         }
         // Save data
         switch (addr&0x2) {
         case 0:
            *data_ptr++ = temp[3];
            *data_ptr++ = temp[2];  break;
         case 2:
            *data_ptr++ = temp[1];
            *data_ptr++ = temp[0];  break;
         }
         addr+=2;
      } while (count > 0);
      break;
   case MS_Long:
      count >>= 2;
      do {
         count--;
         if (count == 0) {
            // Read data from RDBUFF for final read
            rc = readReg(SwdRead_DP_RDBUFF, temp);
         }
         else {
            // Start next read and collect data from last read
            rc = readReg(SwdRead_AHB_DRW, temp);
         }
         if (rc != BDM_RC_OK) {
            return rc;
         }
         // Save data
         *data_ptr++ = temp[3];
         *data_ptr++ = temp[2];
         *data_ptr++ = temp[1];
         *data_ptr++ = temp[0];
         //      addrLSB+=4;
      } while (count > 0);
      break;
   }
   return rc;
#endif
}

/**
 *  Initiates core register operation (read/write) and waits for completion
 *
 *  @param dcrsrValue - value to write to DCSRD register to control operation
 *
 *  @return BDM_RC_OK               Success
 *  @return BDM_RC_TARGET_BUSY      Register is inaccessible as processor is not in debug mode
 *  @return BDM_RC_ARM_ACCESS_ERROR Failed access
 */
static USBDM_ErrorCode coreRegisterOperation(uint32_t dcrsrValue) {
   int retryCount = 40;
   USBDM_ErrorCode rc;

   // Write operation+regNo
   rc = writeMemoryWord(DCRSR_ADDR, dcrsrValue);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Wait for transfer complete
   uint32_t dhcsrValue;
   do {
      if (retryCount-- == 0) {
         // Assume target busy
         return BDM_RC_ARM_ACCESS_ERROR;
      }
      // Check complete (use dcrsrValue as scratch)
      rc = readMemoryWord(DHCSR_ADDR, dhcsrValue);
      if (rc != BDM_RC_OK) {
         return rc;
      }
      if ((dhcsrValue & DHCSR_C_HALT) == 0) {
         // Target must be in DEBUG mode
         return BDM_RC_TARGET_BUSY;
      }
   } while ((dhcsrValue & DHCSR_S_REGRDY) == 0);
   return BDM_RC_OK;
}

/**
 *  Read target register
 *
 *  @param regNo    Number of register to read
 *  @param data     Register value as 32-bit data value in BIG-ENDIAN order
 *
 *  @return BDM_RC_OK               Success
 *  @return BDM_RC_TARGET_BUSY      Register is inaccessible as processor is not in debug mode
 *  @return BDM_RC_ARM_ACCESS_ERROR Failed access
 */
USBDM_ErrorCode readCoreRegister(uint8_t regNo, uint8_t data[4]) {
   USBDM_ErrorCode rc;
   // Execute register transfer command
   rc = coreRegisterOperation(DCRSR_READ|regNo);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Read register value from DCRDR holding register (Big-endian)
   return readMemoryWord(DCRDR_ADDR, data);
}

/**
 *  Write ARM-SWD core register
 *
 *  @param regNo  Register number
 *  @param data   Register value as 32-bit data value in BIG-ENDIAN order
 *
 *  @return BDM_RC_OK               Success
 *  @return BDM_RC_TARGET_BUSY      Register is inaccessible as processor is not in debug mode
 *  @return BDM_RC_ARM_ACCESS_ERROR Failed access
 */
USBDM_ErrorCode writeCoreReg(uint32_t regNo, uint8_t data[4]) {
   USBDM_ErrorCode rc;

   // Write data value to DCRDR holding register
   rc = writeMemoryWord(DCRDR_ADDR, data);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   // Execute register transfer
   return coreRegisterOperation(DCRSR_WRITE|regNo);
}

/**
 *  Modifies value in DHCSR
 *  DHCSR.lsb = (DHCSR&preserveBits)|setBits
 *
 *  @param preserveBits Bits to preserve (done first)
 *  @param setBits      Bits to set (done last)
 *
 *  @return BDM_RC_OK => success, error otherwise
 */
USBDM_ErrorCode modifyDHCSR(uint8_t preserveBits, uint8_t setBits) {
   uint32_t debugStepValue;
   USBDM_ErrorCode rc = readMemoryWord(DHCSR_ADDR, debugStepValue);
   if (rc != BDM_RC_OK) {
      return rc;
   }
   debugStepValue = (debugStepValue&preserveBits) | DHCSR_DBGKEY | setBits;
   return writeMemoryWord(DHCSR_ADDR, debugStepValue);
}

}; // End namespace Swd
