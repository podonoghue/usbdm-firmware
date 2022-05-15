/*
 ============================================================================
 * @file    console-example.cpp (180.ARM_Peripherals/Snippets)
 * @brief   Basic C++ demo using Console class
 *
 *  Created on: 10/1/2016
 *      Author: podonoghue
 ============================================================================
 */
#include <limits.h>
#include "hardware.h"
#include "stringFormatter.h"

// Allow access to USBDM methods without USBDM:: prefix
using namespace USBDM;

int main() {
   char           buff[100];
   int            integer;
   long           longInteger;
   unsigned       unsignedInteger;
   unsigned long  unsignedLong;

   // Writing stuff to console
   console.writeln("\nStarting");

   console.writeln("0.999_us  = ", 0.999_us);
   console.writeln("1_us      = ", 1_us);
   console.writeln("1.0_us    = ", 1.0_us);
   console.writeln("1.001_us  = ", 1.001_us);

   console.writeln("9.99_us   = ", 9.99_us);
   console.writeln("10_us     = ", 10_us);
   console.writeln("10.0_us   = ", 10.0_us);
   console.writeln("10.01_us  = ", 10.01_us);

   console.writeln("99.9_us   = ", 99.9_us);
   console.writeln("100_us    = ", 100_us);
   console.writeln("100.0_us  = ", 100.0_us);
   console.writeln("100.1_us  = ", 100.1_us);

   console.writeln("0.999_ms  = ", 0.999_ms);
   console.writeln("1_ms      = ", 1_ms);
   console.writeln("1.0_ms    = ", 1.0_ms);
   console.writeln("1.001_ms  = ", 1.001_ms);

   console.writeln("9.99_ms   = ", 9.99_ms);
   console.writeln("10_ms     = ", 10_ms);
   console.writeln("10.0_ms   = ", 10.0_ms);
   console.writeln("10.01_ms  = ", 10.01_ms);

   console.writeln("99.9_ms   = ", 99.9_ms);
   console.writeln("100_ms    = ", 100_ms);
   console.writeln("100.0_ms  = ", 100.0_ms);
   console.writeln("100.1_ms  = ", 100.1_ms);

   console.writeln("0.999_s   = ", 0.999_s);
   console.writeln("1_s       = ", 1_s);
   console.writeln("1.0_s     = ", 1.0_s);
   console.writeln("1.001_s   = ", 1.001_s);

   console.writeln("9.99_s    = ", 9.99_s);
   console.writeln("10_s      = ", 10_s);
   console.writeln("10.0_s    = ", 10.0_s);
   console.writeln("10.01_s   = ", 10.01_s);

   console.writeln("99.9_s    = ", 99.9_s);
   console.writeln("100_s     = ", 100_s);
   console.writeln("100.0_s   = ", 100.0_s);
   console.writeln("100.1_s   = ", 100.1_s);


   console.writeln("0.999_Hz  = ", 0.999_Hz);
   console.writeln("1_Hz      = ", 1_Hz);
   console.writeln("1.0_Hz    = ", 1.0_Hz);
   console.writeln("1.001_Hz  = ", 1.001_Hz);

   console.writeln("9.99_Hz   = ", 9.99_Hz);
   console.writeln("10_Hz     = ", 10_Hz);
   console.writeln("10.0_Hz   = ", 10.0_Hz);
   console.writeln("10.01_Hz  = ", 10.01_Hz);

   console.writeln("99.9_Hz   = ", 99.9_Hz);
   console.writeln("100_Hz    = ", 100_Hz);
   console.writeln("100.0_Hz  = ", 100.0_Hz);
   console.writeln("100.1_Hz  = ", 100.1_Hz);

   console.writeln("0.999_kHz = ", 0.999_kHz);
   console.writeln("1_kHz     = ", 1_kHz);
   console.writeln("1.0_kHz   = ", 1.0_kHz);
   console.writeln("1.001_kHz = ", 1.001_kHz);

   console.writeln("9.99_kHz  = ", 9.99_kHz);
   console.writeln("10_kHz    = ", 10_kHz);
   console.writeln("10.0_kHz  = ", 10.0_kHz);
   console.writeln("10.01_kHz = ", 10.01_kHz);

   console.writeln("99.9_kHz  = ", 99.9_kHz);
   console.writeln("100_kHz   = ", 100_kHz);
   console.writeln("100.0_kHz = ", 100.0_kHz);
   console.writeln("100.1_kHz = ", 100.1_kHz);

   console.writeln("0.999_MHz = ", 0.999_MHz);
   console.writeln("1_MHz     = ", 1_MHz);
   console.writeln("1.0_MHz   = ", 1.0_MHz);
   console.writeln("1.001_MHz = ", 1.001_MHz);

   console.writeln("9.99_MHz  = ", 9.99_MHz);
   console.writeln("10_MHz    = ", 10_MHz);
   console.writeln("10.0_MHz  = ", 10.0_MHz);
   console.writeln("10.01_MHz = ", 10.01_MHz);

   console.writeln("99.9_MHz  = ", 99.9_MHz);
   console.writeln("100_MHz   = ", 100_MHz);
   console.writeln("100.0_MHz = ", 100.0_MHz);
   console.writeln("100.1_MHz = ", 100.1_MHz);

   console.writeln("99.9_ticks = ", 99.9_ticks);

   console.writeln(123456.4567f);
   console.writeln(3.4567f);
   console.writeln(3.0067f);
   console.writeln(0.0067f);
   console.writeln(0.12f);
   console.writeln(0.0f);
   console.writeln(-3.4567f);
   console.writeln(-3.0067f);
   console.writeln(-0.0067f);
   console.writeln(-0.12f);
   console.writeln(-3.4567);
   console.writeln(-3.0067);
   console.writeln(-0.0067);
   console.writeln(-0.12);

   console<<123456.4567f<<EndOfLine;
   console<<3.4567f<<EndOfLine;
   console<<3.0067f<<EndOfLine;
   console<<123456.4567<<EndOfLine;
   console<<3.4567<<EndOfLine;
   console<<3.0067<<EndOfLine;

   // Console read/write
   if (console.write("Number: ").readln(integer).isError()) {
      console.writeln("Opps");
   }
   else {
      console.writeln(integer);
   }

   console.writeln("3.1f      = ", 3.1f);
   console.writeln("3.1       = ", 3.1);

   // Test writing basic types using various methods
   console.writeln("True   = ", true);
   console.writeln("False  = ", false);
   console.writeln("peek() = ", console.peek());

   console.writeln("x         = ", 'x');
   console.writeln("0UL       = ", 0UL);
   console.writeln("ULONG_MAX = ", ULONG_MAX);
   console.writeln("LONG_MIN  = ", LONG_MIN);
   console.writeln("LONG_MAX  = ", LONG_MAX);
   console.writeln("0U        = ", 0U);
   console.writeln("UINT_MAX  = ", UINT_MAX);
   console.writeln("INT_MIN   = ", INT_MIN);
   console.writeln("INT_MAX   = ", INT_MAX);

   console.writeln("0UL,Radix_2        = ", 0UL,       Radix_2).flushOutput();
   console.writeln("ULONG_MAX,Radix_2  = ", ULONG_MAX, Radix_2);
   console.writeln("ULONG_MAX,Radix_8  = ", ULONG_MAX, Radix_8);
   console.writeln("ULONG_MAX,Radix_10 = ", ULONG_MAX, Radix_10);
   console.writeln("ULONG_MAX,Radix_16 = ", ULONG_MAX, Radix_16);
   console.writeln("UINT_MAX, Radix_2  = ", UINT_MAX,  Radix_2);
   console.writeln("UINT_MAX, Radix_8  = ", UINT_MAX,  Radix_8);
   console.writeln("UINT_MAX, Radix_10 = ", UINT_MAX,  Radix_10);
   console.writeln("UINT_MAX, Radix_16 = ", UINT_MAX,  Radix_16);

   /*
    * Arrays
    */
   static const int dataArray[] = {1,2,3,4,5,6,};
   const int * const dataPointer = dataArray;
   console.writelnArray(dataArray, Radix_16);               // print array of integers
   console.writelnArray(dataPointer, 6, Radix_16);          // print array of integers
   console.writelnArray(dataArray, 6, Radix_16);            // print array of integers
   console.writeln(dataPointer);                            // print pointer value
   console.writeln(dataPointer, Radix_16);                  // print pointer value
   console.write(dataPointer, Radix_16).writeln();          // print pointer value
   console.write(dataPointer).writeln();                    // print pointer value

   static const float fDataArray[] = {1,2,3,4,5,6,};
   const float *fDataPointer = fDataArray;

   console.writelnArray(fDataArray);                         // print array of floats
   console.writelnArray(fDataPointer, 6);                    // print array of floats
   console.writelnArray(fDataArray, 6);                      // print array of floats
   console.writeln(fDataPointer);                            // print pointer value
   console.writeln(fDataPointer, Radix_10);                  // print pointer value

   static const double dDataArray[] = {1,2,3,4,5,6,};
   const double *dDataPointer = dDataArray;

   console.writelnArray(dDataArray);               // print array of doubles
   console.writelnArray(dDataPointer, 6);          // print array of doubles
   console.writelnArray(dDataArray, 6);            // print array of doubles
   console.writeln(dDataPointer);                  // print pointer value
   console.writeln(dDataPointer, Radix_10);        // print pointer value

   console.writeln("hello"); // print a string

   char aString[] = "hello";

   console.writeln(aString);           // print  string
   console.writelnArray(aString, Radix_16); // print  array of char as integers

   unsigned char uString[] = "hello";

   console.writeln(uString);            // print array of char as integers
   console.writelnArray(uString, Radix_16);  // print array of char as integers

   signed char sString[] = "hello";

   console.writelnArray(sString);            // print array of signed char as integers
   console.writelnArray(sString, Radix_16);  // print array of signed char as integers

   uint8_t uiString[] = "hello this is a long string to split over multiple lines xx";

   console.writelnArray(uiString);           // print array of uint8_t as integers
   console.writelnArray(uiString, Radix_16); // print array of uint8_t as integers

   char cString[] = "hello this is a string in an array";

   console.writeln("Array as string          : ", cString);              // Print array as string
   console.writeln("Array as pointer to array: ", cString, Radix_16);    // Print array as pointer to array
   console.writeln("Array as pointer to array: ", (uint8_t *)cString);   // Print array as pointer to array

   const char *pString[] = {"one", "two", "three"};

   console.write("Array of string pointers as strings:  ").writelnArray(pString);            // print array of strings
   console.write("Array of string pointers as integers: ").writelnArray(pString, Radix_16);  // print array of pointers as integers

   console<<"true               = "<<true<<EndOfLine;
   console<<"false              = "<<false<<EndOfLine;
   console<<"peek()             = "<<console.peek()<<EndOfLine;
   console<<"1>2                = "<<(1>2)<<EndOfLine;
   console<<"1<2                = "<<(1<2)<<EndOfLine;
   console<<"0UL,Radix_2        = "<<Radix_2<<0UL<<EndOfLine;
   console<<"ULONG_MAX,Radix_2  = "<<Radix_2<<ULONG_MAX<<EndOfLine;
   console<<"ULONG_MAX,Radix_8  = "<<Radix_8<<ULONG_MAX<<EndOfLine;
   console<<"ULONG_MAX,Radix_10 = "<<Radix_10<<ULONG_MAX<<EndOfLine;
   console<<"ULONG_MAX,Radix_16 = "<<Radix_16<<ULONG_MAX<<EndOfLine;
   console<<"UINT_MAX,Radix_2   = "<<Radix_2<<UINT_MAX<<EndOfLine;
   console<<"3,Radix_2          = "<<Radix_2<<Padding_LeadingZeroes<<3<<EndOfLine;
   console<<"3,Radix_2,w=10,p=0 = "<<console.width(10)<<Radix_2<<Padding_LeadingZeroes<<3<<EndOfLine;
   console.resetFormat();
   console<<"UINT_MAX,Radix_8   = "<<Radix_8<<UINT_MAX<<EndOfLine;
   console<<"UINT_MAX,Radix_10  = "<<Radix_10<<UINT_MAX<<EndOfLine;
   console<<"UINT_MAX,Radix_16  = "<<Radix_16<<UINT_MAX<<EndOfLine;
   console<<"UINT_MAX,radix(16) = "<<console.radix(16)<<UINT_MAX<<EndOfLine;
   console<<"UINT_MAX,radix(10) = "<<console.radix(10)<<UINT_MAX<<EndOfLine;
   console<<"UINT_MAX,radix(8)  = "<<console.radix(8)<<UINT_MAX<<EndOfLine;
   console<<"UINT_MAX,radix(2)  = "<<console.radix(2)<<UINT_MAX<<EndOfLine;
   console<<Radix_10;

   //  Basic integer formatting to strings
   FormattedIO::ultoa(buff, 100, Radix_10, Padding_LeadingZeroes, 12);
   console.writeln("ultoa(100, buff, Radix_10, Padding_LeadingZeroes, 12)  = ", buff);
   FormattedIO::ltoa(buff, -100, Radix_16, Padding_LeadingSpaces, 12);
   console.writeln("ltoa(-100, buff, Radix_16, Padding_LeadingSpaces, 12)  = ", buff);

   // Test input
   console.write("Value (in radix 2): ").readln(integer,Radix_2);
   console.writeln(integer, Radix_10, ", 0x", (unsigned long)integer, Radix_16, ", 0b", (unsigned long)integer, Radix_2);

   console<<"Value (in radix 16): ">>Radix_16>>integer>>EndOfLine;
   console<<Radix_10<<integer<<", 0x"<<Radix_16<<(unsigned long)integer<<", 0b"<<Radix_2<<(unsigned long)integer<<EndOfLine;

   // Read and echo an input line
   int length = console.write("text : ").gets(buff, sizeof(buff));
   console.writeln("[", length, " chars] = '", buff, "'").flushOutput();

   // Read and write numbers
   console<<Radix_10;
   console<<"value :">>integer>>EndOfLine;
   console<<integer<<EndOfLine;
   console<<"value :">>longInteger>>EndOfLine;
   console<<longInteger<<EndOfLine;
   console<<"value :">>unsignedInteger>>EndOfLine;
   console<<unsignedInteger<<EndOfLine;
   console<<"value :">>unsignedLong>>EndOfLine;
   console<<unsignedLong<<EndOfLine;

   console.write("Two hex integers : ").read(integer,Radix_16).readln(longInteger,Radix_16);
   console.writeln(integer, Radix_16, ", ", longInteger, Radix_16);
   console.write("An integer   : ").readln(longInteger);
   console.writeln(longInteger);
   console.write("An integer   : ").readln(unsignedInteger);
   console.writeln(unsignedInteger);
   console.write("An integer   : ").readln(unsignedLong);
   console.writeln(unsignedLong);

   // Writing formatted data to character buffer
   StringFormatter sf(buff,sizeof(buff));
   sf.write("This is being written to a string ", 1, ",", 2, ",", 3.0, " - done");
   console.writeln(sf.toString());

   console.writeln("Type any character - ignored").readChar();

   console.writeln("\nFinished");

   // If direct interrupt handling is needed.
   //   console.setRxTxCallback(nullptr);

//   Above functions may be applied directly to any Uart as well.
   Uart0 uart;
   uart<<"Hello World\n";

   for(;;) {
      __asm__("nop");
   }
   return 0;
}
