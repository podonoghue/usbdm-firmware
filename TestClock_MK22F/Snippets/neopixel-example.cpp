/**
 * ============================================================================
 * @file    neopixel-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Demo using Ftm class to implement a basic driver for a neopixel
 *
 * The number of lines of neopixels that can be driven is limited by the
 * number of port pins available on a single port.
 * The code assumes consecutive pins within a byte as it uses a GpioXField
 * and byte size data but this limitation can be removed.
 *
 * This code is loosely based on the the technique described here:
 * https://mcuoneclipse.com/2015/08/01/tutorial-adafruit-ws2812b-neopixels-with-the-freescale-frdm-k64f-board-part-1-hardware/
 *
 *  Created on: 3/10/2018
 *      Author: podonoghue
 * ============================================================================
 */
#include <string.h>
#include "hardware.h"
#include "dma.h"
#include "smc.h"

using namespace USBDM;

/**
 * This example uses DMA interrupts.
 *
 * It is necessary to enable these in Configure.usbdmProject
 * under the "Peripheral Parameters"->DMA0 tab.
 * Select irqHandlingMethod option (Class Method - Software ...)
 */

/** GPIO driving pixel data */
using Pixel = GpioDField<3,2>;

/**
 * FTM Timer being used - change as required
 */
using Timer = Ftm0;

// Fixed Timer channels.
// These are used to trigger DMA transfers to the PORT registers.
using TimerSetChannel   = Timer::Channel<1>; //!< PSOR - Sets pin high
using TimerDataChannel  = Timer::Channel<2>; //!< PCOR - Sets pin low/unchanged according to data
using TimerClearChannel = Timer::Channel<3>; //!< PCOR - Sets pin low

// These must match the above channels
constexpr DmaSlot TimerSetChannel_dmaSlot   = Dma0Slot_FTM0_Ch1;
constexpr DmaSlot TimerDataChannel_dmaSlot  = Dma0Slot_FTM0_Ch2;
constexpr DmaSlot TimerClearChannel_dmaSlot = Dma0Slot_FTM0_Ch3;

/** Used to indicate transfer has completed */
static volatile bool complete;

/**
 * DMA complete callback.
 *
 * Sets flag to indicate sequence complete.\n
 * Disables timer channels.
 */
static void dmaCallback(DmaChannelNum channel) {
   Dma0::clearInterruptRequest(channel);
   TimerSetChannel::configure(FtmChMode_Disabled);
   TimerDataChannel::configure(FtmChMode_Disabled);
   TimerClearChannel::configure(FtmChMode_Disabled);
   complete = true;
}

/** Number of pixels in a string. */
static constexpr unsigned PIXEL_LENGTH = 12;

/** Number of RGB bits required per pixel. */
static constexpr unsigned PIXEL_BIT_LENGTH = 24;

/**
 * Transmit Data (used with TimerData DMA channel)
 * This holds the unpacked pixel RGB information.\n
 * Each bit in an element represents a different pixel string.
 * For consistent DMA latency this must be allocated in same region as pixelBitmask.
 */
static uint8_t pixelBuffer[PIXEL_LENGTH*PIXEL_BIT_LENGTH] = {0};

/**
 * Mask for GPIO pins (used with TimerData DMA channel)
 * For consistent DMA latency this must be allocated in same region as pixelBuffer.
 */
static uint8_t pixelBitmask = Pixel::MASK;

/**
 * Unpack RGB pixel data into format for transmission.
 * This routine would be called once for each pixel string being driven by
 * the GPIO pins.
 *
 * @param[in] pixelMask  Bit mask for the GPIO pin driving the pixel string.
 * @param[in] pixelData  Array of RGB data for pixels in string.
 */
void unpackPixels(uint8_t pixelMask, const uint32_t pixelData[PIXEL_LENGTH]) {
   uint8_t *pixelBufferPtr = pixelBuffer;
   for (unsigned pixel=0; pixel<PIXEL_LENGTH; pixel++) {
      uint32_t data = pixelData[pixel];

      // Shuffle data RGB => GRB
      data = ((data<<8)&0xFF0000)|((data>>8)&0x00FF00)|(data&0xFF);

      // Merge into transmission format - 1 bit per byte
      // GGGGGGGGRRRRRRRRBBBBBBBB => bit order in bytes
      for (unsigned bitMask=(1<<(PIXEL_BIT_LENGTH-1)); bitMask != 0; bitMask>>=1) {
         if (data & bitMask) {
            *pixelBufferPtr++ &= ~pixelMask;
         }
         else {
            *pixelBufferPtr++ |= pixelMask;
         }
      }
   }
}

/**
 * Configure DMA from pixelBuffer-->GPIO.
 *
 * This sets up three DMA channels:
 *  - Fixed Bit-mask -> GPIO.PSOR Set all used bits
 *  - pixelBuffer    -> GPIO.PCOR Selectively clear bits
 *  - Fixed Bit-mask -> GPIO.PCOR Clear all used bits
 */
static void initialiseDma(DmaChannelNum dmaSetChannel, DmaChannelNum dmaDataChannel, DmaChannelNum dmaClearChannel) {

   // This example assumes the Pixels lie within the 1st byte of the port
   static_assert((Pixel::MASK&~0xFF) == 0);


   /**
    * @verbatim
    * +------------------------------+            Simple DMA mode (MLNO = Minor Loop Mapping Disabled)
    * | Major Loop =                 |            ==================================================
    * |    CITER x Minor Loop        |
    * |                              |            Each DMA request triggers a minor-loop transfer sequence.
    * | +--------------------------+ |<-DMA Req.  The minor loops are counted in the major-loop.
    * | | Minor Loop               | |
    * | | Each transfer            | |            The following are used during a minor loop:
    * | |   SADDR->DADDR           | |             - SADDR Source address
    * | |   SADDR += SOFF          | |             - SOFF  Adjustment applied to SADDR after each transfer
    * | |   DADDR += DOFF          | |             - DADDR Destination address
    * | | Total transfer is NBYTES | |             - DOFF  Adjustment applied to DADDR after each transfer
    * | +--------------------------+ |             - NBYTES Number of bytes to transfer
    * | +--------------------------+ |<-DMA Req.   - Attributes
    * | | Minor Loop               | |               - ATTR_SSIZE, ATTR_DSIZE Source and destination transfer sizes
    * |..............................|               - ATTR_SMOD, ATTR_DMOD Modulo of the transfer
    * | |                          | |
    * | +--------------------------+ |             The number of reads and writes done will depend on NBYTES, SSIZE and DSIZE
    * | +--------------------------+ |<-DMA Req.   For example: NBYTES=12, SSIZE=16-bits, DSIZE=32-bits => 6 reads, 3 writes
    * | | Minor Loop               | |             NBYTES must be an even multiple of SSIZE and DSIZE in bytes.
    * | | Each transfer            | |
    * | |   SADDR->DADDR           | |            The following are used by the major loop
    * | |   SADDR += SOFF          | |             - SLAST Adjustment applied to SADDR after major loop
    * | |   DADDR += DOFF          | |             - DLAST Adjustment applied to DADDR after major loop
    * | | Total transfer is NBYTES | |             - CITER Major loop counter - counts how many minor loops
    * | +--------------------------+ |
    * |                              |            SLAST and DLAST may be used to reset the addresses to the initial value or
    * | At end of Major Loop         |            link to the next transfer.
    * |    SADDR += SLAST            |            The total transferred for the entire sequence is CITER x NBYTES.
    * |    DADDR += DLAST            |
    * |                              |            Important options in the CSR:
    * | Total transfer =             |              - DMA_CSR_INTMAJOR = Generate interrupt at end of Major-loop
    * |    CITER*NBYTES              |              - DMA_CSR_DREQ     = Clear hardware request at end of Major-loop
    * +------------------------------+              - DMA_CSR_START    = Start transfer. Used for software transfers. Automatically cleared.
    * @endverbatim
    */

   /**
    * Structure to define the Pin setting DMA transfer.
    * Fixed bit-mask => PSOR.
    * Sets all pins driving strings of pixels.
    */
   static constexpr DmaTcd setTCD (
      /* Source address              */ (uint32_t)(&pixelBitmask),         // Source = Fixed bit-mask
      /* Source address offset       */ 0,                                 // Source address doesn't advance
      /* Source size                 */ dmaSize(pixelBitmask),             // 8-bit read from Destination address
      /* Source address modulo       */ DmaModulo_Disabled,                // No modulo
      /* Source last adjustment      */ 0,                                 // Source address doesn't change

      /* Destination address         */ Pixel::gpioPSOR(),                 // Destination is GPIO.PSOR register
      /* Destination address offset  */ 0,                                 // Destination address doesn't change
      /* Destination size            */ dmaSize(pixelBitmask),             // 8-bit write to Source address
      /* Destination address modulo  */ DmaModulo_Disabled,                // No modulo
      /* Destination last adjustment */ 0,                                 // Destination address doesn't change

      /* Minor loop byte count       */ sizeof(pixelBitmask),              // Total transfer in one minor-loop

      /* Major loop count            */ dmaCiter(sizeof(pixelBuffer))/
      /*                             */          sizeof(pixelBuffer[0]),   // Transfer count matches entire pixelBuffer

      /* Channel Start               */ false,                             // Don't start (triggered by hardware)
      /* Disable Request             */ true,                              // Clear hardware request when complete major loop
      /* Interrupt on major complete */ false,                             // Disabled
      /* Interrupt on half complete  */ false,                             // Disabled
      /* Bandwidth (speed) Control   */ DmaSpeed_NoStalls                  // Full speed
   );

   /**
    * Structure to define the Pin data DMA transfer
    * pixelBuffer[n] => PCOR
    * This selectively clears pins driving strings of pixels.
    */
   static constexpr DmaTcd dataTCD (
      /* Source address              */ (uint32_t)(pixelBuffer),           // Source = Pixel data
      /* Source address offset       */ sizeof(pixelBuffer[0]),            // Source address advances
      /* Source size                 */ dmaSize(pixelBuffer[0]),           // 8-bit read from Destination address
      /* Source address modulo       */ DmaModulo_Disabled,                // No modulo
      /* Source last adjustment      */ -(int)sizeof(pixelBuffer),         // Reset Source address back to start of buffer

      /* Destination address         */ Pixel::gpioPCOR(),                 // Destination is GPIO.PCOR register
      /* Destination address offset  */ 0,                                 // Destination address doesn't change
      /* Destination size            */ dmaSize(pixelBuffer[0]),           // 8-bit write to Source address
      /* Destination address modulo  */ DmaModulo_Disabled,                // No modulo
      /* Destination last adjustment */ 0,                                 // Destination address doesn't change

      /* Minor loop byte count       */ sizeof(pixelBuffer[0]),            // Total transfer in one minor-loop

      /* Major loop count            */ dmaCiter(sizeof(pixelBuffer))/
      /*                             */          sizeof(pixelBuffer[0]),   // Transfer count matches entire pixelBuffer

      /* Channel Start               */ false,                             // Don't start (triggered by hardware)
      /* Disable Request             */ true,                              // Clear hardware request when complete major loop
      /* Interrupt on major complete */ false,                             // Disabled
      /* Interrupt on half complete  */ false,                             // Disabled
      /* Bandwidth (speed) Control   */ DmaSpeed_NoStalls                  // Full speed
   );

   /**
    * Structure to define the Pin clearing DMA transfer
    * Fixed bit-mask => PCOR.
    * Clears all pins driving strings of pixels.
    */
   static constexpr DmaTcd clearTCD (
      /* Source address              */ (uint32_t)(&pixelBitmask),          // Source = Fixed bit-mask
      /* Source address offset       */ 0,                                  // Source address doesn't advance
      /* Source size                 */ dmaSize(pixelBitmask),              // 8-bit read from Destination address
      /* Source address modulo       */ DmaModulo_Disabled,                 // No modulo
      /* Source last adjustment      */ 0,                                  // Source address doesn't change

      /* Destination address         */ Pixel::gpioPCOR(),                  // Destination is GPIO.PCOR register
      /* Destination address offset  */ 0,                                  // Destination address doesn't change
      /* Destination size            */ dmaSize(pixelBitmask),                   // 8-bit write to Source address
      /* Destination address modulo  */ DmaModulo_Disabled,                 // No modulo
      /* Destination last adjustment */ 0,                                  // Destination address doesn't change

      /* Minor loop byte count       */ sizeof(pixelBitmask),               // Total transfer in one minor-loop

      /* Major loop count            */ dmaCiter(sizeof(pixelBuffer))/
      /*                             */          sizeof(pixelBuffer[0]),    // Transfer count matches entire pixelBuffer

      /* Channel Start               */ false,                              // Don't start (triggered by hardware)
      /* Disable Request             */ true,                               // Clear hardware request when complete major loop
      /* Interrupt on major complete */ true,                               // Generate interrupt on completion of Major-loop
      /* Interrupt on half complete  */ false,                              // Disabled
      /* Bandwidth (speed) Control   */ DmaSpeed_NoStalls                   // Full speed
   );

   // Enable DMAC with default settings
   Dma0::configure(
         DmaOnError_Halt,
         DmaMinorLoopMapping_Disabled,
         DmaContinuousLink_Disabled,
         DmaArbitration_RoundRobin);

   // Set callback (Interrupts are enabled in one TCD above)
   Dma0::setCallback(dmaClearChannel, dmaCallback);
   Dma0::enableNvicInterrupts(dmaClearChannel, NvicPriority_Normal);

   // Connect DMA channels to FTM for triggering
   DmaMux0::configure(dmaSetChannel,   TimerSetChannel_dmaSlot,   DmaMuxEnable_Continuous);
   DmaMux0::configure(dmaDataChannel,  TimerDataChannel_dmaSlot,  DmaMuxEnable_Continuous);
   DmaMux0::configure(dmaClearChannel, TimerClearChannel_dmaSlot, DmaMuxEnable_Continuous);

   // Configure the DMA transfers
   Dma0::configureTransfer(dmaSetChannel,   setTCD);
   Dma0::configureTransfer(dmaDataChannel,  dataTCD);
   Dma0::configureTransfer(dmaClearChannel, clearTCD);

   // Check if configuration failed
   checkError();
}

/*
 * FTM/DMA   Set       Clear/None     Clear
 * Channels  (1)          (2)          (3)
 *            |            |            |
 *            V            V            V
 *            .            .            .
 *            +------------+            .            +--
 *    0 =>    |  T0_HIGH   |            .  T0_LOW    |
 *          --+            +-------------------------+
 *            .                         .
 *            +-------------------------+            +--
 *    1 =>    |       T1_HIGH           |  T1_LOW    |
 *          --+                         +------------+
 *            |<-------------- Period -------------->|
 */
//#define WS2812
//#define WS2812B
#define SK6812
#if defined WS2812
static constexpr float T0_HIGH = 400*ns;   // 350 +/- 150 ns
static constexpr float T1_HIGH = 600*ns;   // 700 +/- 150 ns
static constexpr float PERIOD  = 1200*ns;
//static constexpr float T0_LOW  = 800*ns; // 800 +/- 150 ns
//static constexpr float T1_LOW  = 600*ns; // 600 +/- 150 ns
#elif defined WS2812B
static constexpr float T0_HIGH = 500*ns;   // 400 +/- 150 ns
static constexpr float T1_HIGH = 800*ns;   // 800 +/- 150 ns
static constexpr float PERIOD  = 1350*ns;
//static constexpr float T0_LOW  = 850*ns; // 850 +/- 150 ns
//static constexpr float T1_LOW  = 550*ns; // 450 +/- 150 ns
#elif defined SK6812
static constexpr float T0_HIGH = 300*ns+20*ns; // 300 +/- 150 ns (20ns tweak for latency)
static constexpr float T1_HIGH = 600*ns;       // 600 +/- 150 ns
static constexpr float PERIOD  = 1200*ns;
//static constexpr float T0_LOW  = 900*ns;     // 900 +/- 150 ns
//static constexpr float T1_LOW  = 600*ns;     // 600 +/- 150 ns
#else
#error "Unknown Neopixel type"
#endif

/**
 * Start DMA transfer of pixel data to LEDs.
 * Assumes initialiseFtm() and initialiseDma() have been called previously.
 */
static void startTransfer(DmaChannelNum dmaSetChannel, DmaChannelNum dmaDataChannel, DmaChannelNum dmaClearChannel) {
   // Sequence not complete yet
   complete = false;

   const uint32_t dmaChannelMask = (1<<dmaSetChannel)|(1<<dmaDataChannel)|(1<<dmaClearChannel);
   const uint32_t ftmChannelMask = TimerSetChannel::CHANNEL_MASK|TimerDataChannel::CHANNEL_MASK|TimerClearChannel::CHANNEL_MASK;

   /**
    * It is important that the FTM and DMA start in the correct sequence.
    * Due to the short timing involved it is easier to disable the timer
    * counter clock before making changes to FTM or DMA
    */
   Timer::setClockSource(FtmClockSource_Disabled);

   // Clear any old interrupts (DMA requests)
   // This only works because the FTM channels have DMA disabled at the moment!
   Timer::clearSelectedInterruptFlags(ftmChannelMask);

   // Configure the three channels for DMA (timing is already set)
#if 1
   TimerSetChannel::configure(FtmChMode_OutputCompare, FtmChannelAction_Dma);
   TimerDataChannel::configure(FtmChMode_OutputCompare, FtmChannelAction_Dma);
   TimerClearChannel::configure(FtmChMode_OutputCompare, FtmChannelAction_Dma);
#else
   // For debug - output FTM channels to pins
   TimerSetChannel::configure(FtmChMode_OutputCompareToggle, FtmChannelAction_Dma);
   TimerDataChannel::configure(FtmChMode_OutputCompareToggle, FtmChannelAction_Dma);
   TimerClearChannel::configure(FtmChMode_OutputCompareToggle, FtmChannelAction_Dma);

   Timer::setChanelOutputs(0);

   TimerSetChannel::setOutput();
   TimerDataChannel::setOutput();
   TimerClearChannel::setOutput();
#endif
   // Enable timer->DMA hardware requests
   Dma0::enableMultipleRequests(dmaChannelMask);

   // Restart the counter from zero so sequence starts in correct order
   Timer::resetTime();

   // Restore clock
   Timer::setClockSource(FtmClockSource_System);

   // Check if configuration failed
   checkError();
}

/**
 * Base initialisation of FTM
 */
static void initialiseFtm() {
   /**
    * FTM set up as required for Output Compare
    */
   // Configure base FTM (affects all channels)
   Timer::configure(
         FtmMode_LeftAlign,      // Left-aligned is required for OC/IC
         FtmClockSource_System); // System clock

   // Set required Timer period.
   // Note that this requires the counter clock to have been already set
   Timer::setPeriod(PERIOD);

   // Configure the channels for O.C so event times can be set - no DMA initially
   TimerSetChannel::configure(FtmChMode_OutputCompare, FtmChannelAction_None);
   TimerDataChannel::configure(FtmChMode_OutputCompare, FtmChannelAction_None);
   TimerClearChannel::configure(FtmChMode_OutputCompare, FtmChannelAction_None);

   // Set channel event times
   TimerSetChannel::setEventTime(0);
   TimerDataChannel::setEventTime(Timer::convertSecondsToTicks(T0_HIGH));
   TimerClearChannel::setEventTime(Timer::convertSecondsToTicks(T1_HIGH));

   // Check if configuration failed
   checkError();
}

/**
 * Initialise the GPIO pins driving the Neopixel
 */
void initialisePins() {
   Pixel::setOutput(PinDriveStrength_High);
}

/**
 * Write pixel buffer to LEDs
 *
 * @param pixel0Data
 * @param pixel1Data
 */
void writeLEDs(const uint32_t *pixel0Data, const uint32_t *pixel1Data) {
   //   console.writeln("Starting Transfer");

   unpackPixels(Pixel::mask(0), pixel0Data);
   unpackPixels(Pixel::mask(1), pixel1Data);

   // Start transfer
   startTransfer(DmaChannelNum_0, DmaChannelNum_1, DmaChannelNum_2);

   // Wait for completion of 1 Major-loop = 1 pixelBuffer transfer
   while (!complete) {
      Smc::enterWaitMode();
   }
   //   console.writeln("Transfer complete");
}

/**
 * Construct RGB value from individual RGB bytes
 *
 * @param[in] red
 * @param[in] green
 * @param[in] blue
 *
 * @return 24-bit RG value
 */
constexpr uint32_t rgb(uint8_t red, uint8_t green, uint8_t blue) {
   return (red<<16)|(green<<8)|blue;
}

/**
 * Extract red component from RGB colour
 *
 * @param[in] rgb
 *
 * @return Extracted component
 */
constexpr uint8_t red(uint32_t rgb) {
   return (rgb>>16)&0xFF;
}

/**
 * Extract green component from RGB colour
 *
 * @param[in] rgb
 *
 * @return Extracted component
 */
constexpr uint8_t green(uint32_t rgb) {
   return (rgb>>8)&0xFF;
}

/**
 * Extract blue component from RGB colour
 *
 * @param[in] rgb
 *
 * @return Extracted component
 */
constexpr uint8_t blue(uint32_t rgb) {
   return rgb&0xFF;
}

/**
 * Calculate an intermediate colour
 *
 * @param[in] from          1st colour
 * @param[in] to            2nd colour
 * @param[in] percentage    Controls the percentage of each colour included in the result
 * @param[in] brightness    Weighting for overall brightness (percent)
 *
 * @return  Intermediate colour = (((100-percentage)*from) + (percentage*to))/100;
 */
uint32_t mix(uint32_t from, uint32_t to, int percentage, int brightness=100) {

   uint8_t r = brightness*(((100-percentage)*red(from))   + (percentage*red(to)))/10000;
   uint8_t g = brightness*(((100-percentage)*green(from)) + (percentage*green(to)))/10000;
   uint8_t b = brightness*(((100-percentage)*blue(from))  + (percentage*blue(to)))/10000;

   return rgb(r,g,b);
}

/**
 *  Pan LEDs between colours
 *
 * @param[in]    from       Colour to pan from (current colour)
 * @param[in]    to         Colour to pan to
 * @param[in]    delay      Delay between changes (milliseconds)
 * @param[inout] pixelData  Buffer for persistent pixel data
 */
void panLeds(uint32_t from, uint32_t to, int delay, uint32_t pixelData[PIXEL_LENGTH]) {
   for(unsigned percentage=0; percentage<=100; percentage+=10) {
      memmove(pixelData, pixelData+1, (PIXEL_LENGTH-1)*sizeof(*pixelData));
      pixelData[PIXEL_LENGTH-1] = mix(from, to, percentage, 100);
      writeLEDs(pixelData, pixelData);
      waitMS(delay);
   }
}

/**
 * Pan between random colours
 */
void animate1() {
   // Delay between changes
   constexpr int DELAY_VALUE = 100;

   // Buffer for pixel data
   uint32_t pixelData[PIXEL_LENGTH];

   // Clear buffer initially
   memset(pixelData, 0, sizeof(pixelData));

   // Pan from off
   uint32_t to, from = rgb(0,0,0);

   // Pan between random colours
   for(;;) {
      to = rgb(rand()%256,rand()%256,rand()%256);
      panLeds(from, to, DELAY_VALUE, pixelData);
      from = to;
   }
}

/**
 * Pan between random colours from a table
 */
void animate2() {
   static const uint32_t colours[] = {
         rgb(  0,   0, 255),
         rgb(  0, 255,   0),
         rgb(255,   0,   0),
         rgb(  0, 255, 255),
         rgb(255, 255,   0),
         rgb(255,   0, 255),
         rgb(255, 255, 255),
   };

   // Delay between changes
   constexpr int DELAY_VALUE = 100;

   // Buffer for pixel data
   uint32_t pixelData[PIXEL_LENGTH];

   // Clear buffer initially
   memset(pixelData, 0, sizeof(pixelData));

   // Pan between random colours from array
   uint32_t to, from = rgb(0,0,0);
   for(;;) {
      to   = colours[rand()%(sizeof(colours)/sizeof(colours[0]))];
      panLeds(from, to, DELAY_VALUE, pixelData);
      from = to;
   }
}

/**
 * Change to colours from a table
 */
void animate3() {

   // Delay between changes
   constexpr int DELAY_VALUE = 100;

#if 0
   uint32_t pixels[PIXEL_LENGTH];
   for (unsigned i=0;;i+=100) {
      for (unsigned j=0; j<(PIXEL_LENGTH);j++) {
         pixels[j] = i;
         i += 0x010101;
      }
      writeLEDs(pixels, pixels);
      waitMS(100);
   }
#else
   //                                       RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB
   //   uint32_t pixel0Data[PIXEL_LENGTH]  = { 0x341256, 0x221133, 0x221133, 0x221133, 0x221133, 0x221133, 0x221133, 0x221133, 0x221133, 0x221133, 0x221133, 0x221133, };
   static const uint32_t pixel1DataA[PIXEL_LENGTH] = { 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, };
   static const uint32_t pixel1DataB[PIXEL_LENGTH] = { 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, };
   static const uint32_t pixel1DataC[PIXEL_LENGTH] = { 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, };
   static const uint32_t pixel1DataD[PIXEL_LENGTH] = { 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, };
   static const uint32_t pixel1DataE[PIXEL_LENGTH] = { 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, };
   static const uint32_t pixel1DataF[PIXEL_LENGTH] = { 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, };
   static const uint32_t pixel1DataG[PIXEL_LENGTH] = { 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, };
   static const uint32_t pixel1DataH[PIXEL_LENGTH] = { 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, };
   static const uint32_t pixel1DataI[PIXEL_LENGTH] = { 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, };
   static const uint32_t pixel1DataJ[PIXEL_LENGTH] = { 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, };
   static const uint32_t pixel1DataK[PIXEL_LENGTH] = { 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, };
   static const uint32_t pixel1DataL[PIXEL_LENGTH] = { 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, };

   for(;;) {
      writeLEDs(pixel1DataB, pixel1DataL);
      waitMS(DELAY_VALUE);
      writeLEDs(pixel1DataA, pixel1DataK);
      waitMS(DELAY_VALUE);
      writeLEDs(pixel1DataC, pixel1DataJ);
      waitMS(DELAY_VALUE);
      writeLEDs(pixel1DataD, pixel1DataI);
      waitMS(DELAY_VALUE);
      writeLEDs(pixel1DataE, pixel1DataH);
      waitMS(DELAY_VALUE);
      writeLEDs(pixel1DataF, pixel1DataG);
      waitMS(DELAY_VALUE);
      writeLEDs(pixel1DataG, pixel1DataF);
      waitMS(DELAY_VALUE);
      writeLEDs(pixel1DataH, pixel1DataE);
      waitMS(DELAY_VALUE);
      writeLEDs(pixel1DataI, pixel1DataD);
      waitMS(DELAY_VALUE);
      writeLEDs(pixel1DataJ, pixel1DataC);
      waitMS(DELAY_VALUE);
      writeLEDs(pixel1DataK, pixel1DataB);
      waitMS(DELAY_VALUE);
      writeLEDs(pixel1DataL, pixel1DataA);
      waitMS(DELAY_VALUE);
   }
#endif
}

/**
 * Sends fixed pattern for testing timing
 */
void test() {

   // Delay between changes
   constexpr int DELAY_VALUE = 100;

   //                                                    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB
   static const uint32_t pixel1DataA[PIXEL_LENGTH] = { 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, 0x0000FF, 0x00FF00, 0xFF0000, 0xFFFF00, 0xFF00FF, 0x00FFFF, };
   for(;;) {
      writeLEDs(pixel1DataA, pixel1DataA);
      waitMS(DELAY_VALUE);
   }
}

/**
 * Sends fixed pattern for testing timing
 */
void testRandom() {

   // Delay between changes
   constexpr int DELAY_VALUE = 100;

   //                                 RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB    RRGGBB
   static uint32_t pixel1DataA[] = {0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, 0xFFFFFF, };
   for(;;) {
      uint32_t value = rand();
      for (unsigned index=0; index<PIXEL_LENGTH; index++) {
         pixel1DataA[index] = value;
      }
      writeLEDs(pixel1DataA, pixel1DataA);
      waitMS(DELAY_VALUE);
   }
}

/**
 * Demonstration main-line
 *
 * @return Not used.
 */
int main() {

   initialisePins();

   initialiseFtm();

   initialiseDma(DmaChannelNum_0, DmaChannelNum_1, DmaChannelNum_2);

//   testRandom();

   animate2();

   return 0;
}
