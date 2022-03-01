//------------Copyright (C) 2008 Maxim Integrated Products --------------
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY,  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL MAXIM INTEGRATED PRODCUTS BE LIABLE FOR ANY CLAIM, DAMAGES
// OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// Except as contained in this notice, the name of Maxim Integrated Products
// shall not be used except as stated in the Maxim Integrated Products
// Branding Policy.
// ---------------------------------------------------------------------------
//
//  an3684.C - Application Note 3684 example implementation using CMAXQUSB.
//

#include <stdio.h>
#include <conio.h>
#include <windows.h>

// CMOD function prototypes
void  UnLoadCMOD(void);
int LoadCMOD(void);

// 1-Wire API for DS2482 function prototypes
int OWReset(void);
void OWWriteByte(unsigned char sendbyte);
unsigned char OWReadByte(void);
unsigned char OWTouchByte(unsigned char sendbyte);
unsigned char OWTouchBit(unsigned char sendbit);
void OWWriteBit(unsigned char sendbit);
unsigned char OWReadBit(void);
void OWBlock(unsigned char *tran_buf, int tran_len);
int OWFirst();
int OWNext();
int  OWVerify();
void OWTargetSetup(unsigned char family_code);
void OWFamilySkipSetup();
int OWSearch();

// Extended 1-Wire functions
int OWSpeed(int new_speed);
int OWLevel(int level);
int OWWriteBytePower(int sendbyte);
int OWReadBitPower(int applyPowerResponse);

// Helper functions
int DS2482_detect(unsigned char addr);
unsigned char DS2482_search_triplet(int search_direction);
int DS2482_write_config(unsigned char config);
int DS2482_reset();
int DS2482_channel_select(int channel);
unsigned char calc_crc8(unsigned char data);
void msDelay(int delay);

// I2C primitives using CMOD
void I2C_stop(void);
int I2C_start();
int I2C_rep_start();
int I2C_write(unsigned char data, int expect_ack);
unsigned char I2C_read(int ack);

// defines the data direction (reading from I2C device) in I2C_start(),I2C_rep_start() 
#define I2C_READ    1

// defines the data direction (writing to I2C device) in I2C_start(),I2C_rep_start() 
#define I2C_WRITE   0

// Flags on I2C_read 
#define ACK    1
#define NACK   0

#define EXPECT_ACK    1
#define EXPECT_NACK   0

// misc constants
#define POLL_LIMIT  200
#define TRUE    1
#define FALSE   0

// DS2482 commands
#define CMD_DRST   0xF0
#define CMD_WCFG   0xD2
#define CMD_CHSL   0xC3
#define CMD_SRP    0xE1
#define CMD_1WRS   0xB4
#define CMD_1WWB   0xA5
#define CMD_1WRB   0x96
#define CMD_1WSB   0x87
#define CMD_1WT    0x78

// DS2482 config bits
#define CONFIG_APU  0x01
#define CONFIG_PPM  0x02
#define CONFIG_SPU  0x04
#define CONFIG_1WS  0x08

// DS2482 status bits 
#define STATUS_1WB  0x01
#define STATUS_PPD  0x02
#define STATUS_SD   0x04
#define STATUS_LL   0x08
#define STATUS_RST  0x10
#define STATUS_SBR  0x20
#define STATUS_TSB  0x40
#define STATUS_DIR  0x80

// API mode bit flags
#define MODE_STANDARD                  0x00
#define MODE_OVERDRIVE                 0x01
#define MODE_STRONG                    0x02

// globals for CMOD
static FARPROC CmodBoardConnect, CmodBoardConnected, CmodBoardDisconnect, CmodComm2ByteWrite, CmodCommResultChar;
static HINSTANCE hInst;

// Search state
unsigned char ROM_NO[8];
int LastDiscrepancy;
int LastFamilyDiscrepancy;
int LastDeviceFlag;
unsigned char crc8;

// DS2482 state
unsigned char I2C_address;
int short_detected;
int c1WS, cSPU, cPPM, cAPU;

//--------------------------------------------------------------------------
// AN3684 Code example using CMAXQUSB 
//
void main(int argc, char **argv)
{
   unsigned char buf[200];
   int i,rslt;
   int cnt=0;
   unsigned char sendpacket[10],crc_verify;
   int sendlen=0;

   // CMOD API SETUP ----------------------------------------
   // load the CMOD driver and get pointers to functions 
   if (!LoadCMOD())
   {
      printf("ERROR, could not load CMODCOMM.DLL   %d\n",hInst);
      exit(0);
   }
   printf("CMODCOMM.DLL loaded\n");

   // Connect to CMOD board
   CmodBoardConnect();
   if (!CmodBoardConnected())
   {
      printf("ERROR, could not connect to CMOD board\n");
      UnLoadCMOD();
      exit(0);
   }
   printf("CMOD board connected\n");
   // END CMOD API SETUP ------------------------------------

   // verify DS2482 is present on default address (0x30)
   if (!DS2482_detect(0x30))
   {
      printf("Failed to find and setup DS2482\n");
      exit(0);
   }

   // The following code performs various operations using the API

   // find ALL devices
   printf("\nFIND ALL\n");
   cnt = 0;
   rslt = OWFirst();
   while (rslt)
   {
      // print device found
      for (i = 7; i >= 0; i--)
         printf("%02X", ROM_NO[i]);
      printf("  %d\n",++cnt);

      rslt = OWNext();
   }

   // find only 0x1A
   printf("\nFIND ONLY 0x1A\n");
   cnt = 0;
   OWTargetSetup(0x1A);
   while (OWNext())
   {
      // check for incorrect type
      if (ROM_NO[0] != 0x1A)
         break;
      
      // print device found
      for (i = 7; i >= 0; i--)
         printf("%02X", ROM_NO[i]);
      printf("  %d\n",++cnt);
   }

   // find all but 0x04, 0x1A, 0x23, and 0x01
   printf("\nFIND ALL EXCEPT 0x10, 0x04, 0x0A, 0x1A, 0x23, 0x01\n");
   cnt = 0;
   rslt = OWFirst();
   while (rslt)
   {
      // check for incorrect type
      if ((ROM_NO[0] == 0x04) || (ROM_NO[0] == 0x1A) || 
          (ROM_NO[0] == 0x01) || (ROM_NO[0] == 0x23) ||
          (ROM_NO[0] == 0x0A) || (ROM_NO[0] == 0x10))
         OWFamilySkipSetup();
      else
      {
         // print device found
         for (i = 7; i >= 0; i--)
            printf("%02X", ROM_NO[i]);
         printf("  %d\n",++cnt);
      }

      rslt = OWNext();
   }

   // find a DS1920 
   printf("\nFind DS1920/DS1820 and do a conversion\n");
   OWTargetSetup(0x10);
   if (OWNext())
   {
      // verify correct type
      if (ROM_NO[0] == 0x10)
      {
         // print device found
         for (i = 7; i >= 0; i--)
            printf("%02X", ROM_NO[i]);
         printf("\n");

         // device already selected from search
         // send the convert command
         if (!OWWriteBytePower(0x44))
            printf("Fail convert command\n");

         // delay for 1 second
         msDelay(1000);

         // turn off the 1-Wire Net strong pull-up
         OWLevel(MODE_STANDARD); 

         // verify complete
         if (OWReadByte() != 0xFF)
            printf("ERROR, temperature conversion was not complete\n");

         // select the device
         sendpacket[0] = 0x55; // match command
         for (i = 0; i < 8; i++)
            sendpacket[i+1] = ROM_NO[i];

         // Reset 1-Wire 
         if (OWReset())
         {
            // MATCH ROM sequence
            OWBlock(sendpacket,9);

            // Read Scratch pad
            sendlen = 0;
            sendpacket[sendlen++] = 0xBE;
            for (i = 0; i < 9; i++)
               sendpacket[sendlen++] = 0xFF;

            OWBlock(sendpacket,sendlen);

            printf("Scatchpad result = ");
            for (i = 0; i < sendlen; i++)
               printf("%02X",sendpacket[i]);
            printf("\n");
         }
         else
            printf("NO RESET\n");
      }
   }

   // Find a DS2502 and read the entire contents
   printf("\nFind DS2502 and read contents\n");
   OWTargetSetup(0x09);
   if (OWNext())
   {
      // verify correct type
      if (ROM_NO[0] == 0x09)
      {
         // print device found
         for (i = 7; i >= 0; i--)
            printf("%02X", ROM_NO[i]);
         printf("\n");
      }

      // device already selected from search
      // send read memory command
      crc8 = 0;
      OWWriteBytePower(0xF0);
      calc_crc8(0xF0);

      // send address and verify CRC8
      OWWriteBytePower(0x00);
      calc_crc8(0x00);
      OWWriteBytePower(0x00);
      calc_crc8(0x00);
      crc_verify = OWReadByte();

      // check CRC on command sequence
      if (crc_verify != crc8)
         printf("ERROR, CRC8 on command sequence was incorrect\n");
      else 
         printf("CRC8 verified on command sequence\n");

      // read the memory
      crc8 = 0;
      // construct block to read device
      for (i = 0; i < 128; i++)
         buf[i] = 0xFF;
      OWBlock(buf,128);

      // print the block and calculate CRC8
      printf("DS2502 Contents:\n");
      for (i = 0; i < 128; i++)
      {
         printf("%02X ", buf[i]);
         calc_crc8(buf[i]);
      }

      // read CRC8 from device
      crc_verify = OWReadByte();

      // check CRC on command sequence
      if (crc_verify != crc8)
         printf("ERROR, CRC8 on command sequence was incorrect\n");
      else 
         printf("CRC8 verified on data\n");
   }

   // CMOD API CLEANUP --------------------------------------
   // disconnect from board
   CmodBoardDisconnect();
   printf("CMOD board disconnected\n");

   // Unload the CMOD driver
   UnLoadCMOD();
   printf("CMODCOMM.DLL unloaded\n");
   // END CMOD API CLEANUP ----------------------------------

   printf("AN3684 end\n");
}

//--------------------------------------------------------------------------
// DS2428 Detect routine that sets the I2C address and then performs a 
// device reset followed by writing the configuration byte to default values:
//   1-Wire speed (c1WS) = standard (0)
//   Strong pull-up (cSPU) = off (0)
//   Presence pulse masking (cPPM) = off (0)
//   Active pull-up (cAPU) = on (CONFIG_APU = 0x01)
//
// Returns: TRUE if device was detected and written
//          FALSE device not detected or failure to write configuration byte
//
int DS2482_detect(unsigned char addr)
{
   // set global address
   I2C_address = addr;

   // reset the DS2482 ON selected address
   if (!DS2482_reset())
      return FALSE;

   // default configuration
   c1WS = FALSE;
   cSPU = FALSE;
   cPPM = FALSE;
   cAPU = CONFIG_APU;

   // write the default configuration setup
   if (!DS2482_write_config(c1WS | cSPU | cPPM | cAPU))
      return FALSE;

   return TRUE;
}

//--------------------------------------------------------------------------
// Perform a device reset on the DS2482
//
// Returns: TRUE if device was reset
//          FALSE device not detected or failure to perform reset
//
int DS2482_reset()
{
   unsigned char status;

   // Device Reset
   //   S AD,0 [A] DRST [A] Sr AD,1 [A] [SS] A\ P
   //  [] indicates from slave
   //  SS status byte to read to verify state

   I2C_start();
   I2C_write(I2C_address | I2C_WRITE, EXPECT_ACK);
   I2C_write(CMD_DRST, EXPECT_ACK);
   I2C_rep_start();
   I2C_write(I2C_address | I2C_READ, EXPECT_ACK);
   status = I2C_read(NACK);
   I2C_stop();

   // check for failure due to incorrect read back of status
   return ((status & 0xF7) == 0x10);
}

//--------------------------------------------------------------------------
// Write the configuration register in the DS2482. The configuration 
// options are provided in the lower nibble of the provided config byte. 
// The uppper nibble in bitwise inverted when written to the DS2482.
//  
// Returns:  TRUE: config written and response correct
//           FALSE: response incorrect
//
int DS2482_write_config(unsigned char config)
{
   unsigned char read_config;

   // Write configuration (Case A)
   //   S AD,0 [A] WCFG [A] CF [A] Sr AD,1 [A] [CF] A\ P
   //  [] indicates from slave
   //  CF configuration byte to write

   I2C_start();
   I2C_write(I2C_address | I2C_WRITE, EXPECT_ACK);
   I2C_write(CMD_WCFG, EXPECT_ACK);
   I2C_write(config | (~config << 4), EXPECT_ACK);
   I2C_rep_start();
   I2C_write(I2C_address | I2C_READ, EXPECT_ACK);
   read_config = I2C_read(NACK);
   I2C_stop();

   // check for failure due to incorrect read back
   if (config != read_config)
   {
      // handle error
      // ...
      DS2482_reset();

      return FALSE;
   }

   return TRUE;
}

//--------------------------------------------------------------------------
// Select the 1-Wire channel on a DS2482-800. 
//
// Returns: TRUE if channel selected
//          FALSE device not detected or failure to perform select
//
int DS2482_channel_select(int channel)
{
   unsigned char ch, ch_read, check;

   // Channel Select (Case A)
   //   S AD,0 [A] CHSL [A] CC [A] Sr AD,1 [A] [RR] A\ P
   //  [] indicates from slave
   //  CC channel value
   //  RR channel read back

   I2C_start();
   I2C_write(I2C_address | I2C_WRITE, EXPECT_ACK);
   I2C_write(CMD_CHSL, EXPECT_ACK);

   switch (channel)
   {
      default: case 0: ch = 0xF0; ch_read = 0xB8; break;
      case 1: ch = 0xE1; ch_read = 0xB1; break;
      case 2: ch = 0xD2; ch_read = 0xAA; break;
      case 3: ch = 0xC3; ch_read = 0xA3; break;
      case 4: ch = 0xB4; ch_read = 0x9C; break;
      case 5: ch = 0xA5; ch_read = 0x95; break;
      case 6: ch = 0x96; ch_read = 0x8E; break;
      case 7: ch = 0x87; ch_read = 0x87; break;
   };

   I2C_write(ch, EXPECT_ACK);
   I2C_rep_start();
   I2C_write(I2C_address | I2C_READ, EXPECT_ACK);
   check = I2C_read(NACK);
   I2C_stop();

   // check for failure due to incorrect read back of channel
   return (check == ch_read);
}

//---------------------------------------------------------------------------
//-------- Basic 1-Wire functions
//---------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Reset all of the devices on the 1-Wire Net and return the result.
//
// Returns: TRUE(1):  presence pulse(s) detected, device(s) reset
//          FALSE(0): no presence pulses detected
//
int OWReset(void)
{
   unsigned char status;
   int poll_count = 0;

   // 1-Wire reset (Case B)
   //   S AD,0 [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
   //                                   \--------/        
   //                       Repeat until 1WB bit has changed to 0
   //  [] indicates from slave

   I2C_start();
   I2C_write(I2C_address | I2C_WRITE, EXPECT_ACK);
   I2C_write(CMD_1WRS, EXPECT_ACK);
   I2C_rep_start();
   I2C_write(I2C_address | I2C_READ, EXPECT_ACK);

   // loop checking 1WB bit for completion of 1-Wire operation 
   // abort if poll limit reached
   status = I2C_read(ACK);
   do
   {
      status = I2C_read(status & STATUS_1WB);
   }
   while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

   I2C_stop();

   // check for failure due to poll limit reached
   if (poll_count >= POLL_LIMIT)
   {
      // handle error
      // ...
      DS2482_reset();
      return FALSE;
   }

   // check for short condition
   if (status & STATUS_SD)
      short_detected = TRUE;
   else
      short_detected = FALSE;

   // check for presence detect
   if (status & STATUS_PPD)
      return TRUE;
   else
      return FALSE;
}

//--------------------------------------------------------------------------
// Send 1 bit of communication to the 1-Wire Net.
// The parameter 'sendbit' least significant bit is used.
//
// 'sendbit' - 1 bit to send (least significant byte)
//
void OWWriteBit(unsigned char sendbit)
{
   OWTouchBit(sendbit);
}

//--------------------------------------------------------------------------
// Reads 1 bit of communication from the 1-Wire Net and returns the
// result
//
// Returns:  1 bit read from 1-Wire Net
//
unsigned char OWReadBit(void)
{
   return OWTouchBit(0x01);
}

//--------------------------------------------------------------------------
// Send 1 bit of communication to the 1-Wire Net and return the
// result 1 bit read from the 1-Wire Net.  The parameter 'sendbit'
// least significant bit is used and the least significant bit
// of the result is the return bit.
//
// 'sendbit' - the least significant bit is the bit to send
//
// Returns: 0:   0 bit read from sendbit
//          1:   1 bit read from sendbit
//
unsigned char OWTouchBit(unsigned char sendbit)
{
   unsigned char status;
   int poll_count = 0;

   // 1-Wire bit (Case B)
   //   S AD,0 [A] 1WSB [A] BB [A] Sr AD,1 [A] [Status] A [Status] A\ P
   //                                          \--------/        
   //                           Repeat until 1WB bit has changed to 0
   //  [] indicates from slave
   //  BB indicates byte containing bit value in msbit
   
   I2C_start();
   I2C_write(I2C_address | I2C_WRITE, EXPECT_ACK);
   I2C_write(CMD_1WSB, EXPECT_ACK);
   I2C_write(sendbit ? 0x80 : 0x00, EXPECT_ACK);
   I2C_rep_start();
   I2C_write(I2C_address | I2C_READ, EXPECT_ACK);

   // loop checking 1WB bit for completion of 1-Wire operation 
   // abort if poll limit reached
   status = I2C_read(ACK);
   do
   {
      status = I2C_read(status & STATUS_1WB);
   }
   while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

   I2C_stop();

   // check for failure due to poll limit reached
   if (poll_count >= POLL_LIMIT)
   {
      // handle error
      // ...
      DS2482_reset();
      return 0;
   }

   // return bit state
   if (status & STATUS_SBR)
      return 1;
   else
      return 0;
}

//--------------------------------------------------------------------------
// Send 8 bits of communication to the 1-Wire Net and verify that the
// 8 bits read from the 1-Wire Net is the same (write operation).
// The parameter 'sendbyte' least significant 8 bits are used.
//
// 'sendbyte' - 8 bits to send (least significant byte)
//
// Returns:  TRUE: bytes written and echo was the same
//           FALSE: echo was not the same
//
void OWWriteByte(unsigned char sendbyte)
{
   unsigned char status;
   int poll_count = 0;

   // 1-Wire Write Byte (Case B)
   //   S AD,0 [A] 1WWB [A] DD [A] Sr AD,1 [A] [Status] A [Status] A\ P
   //                                          \--------/        
   //                             Repeat until 1WB bit has changed to 0
   //  [] indicates from slave
   //  DD data to write

   I2C_start();
   I2C_write(I2C_address | I2C_WRITE, EXPECT_ACK);
   I2C_write(CMD_1WWB, EXPECT_ACK);
   I2C_write(sendbyte, EXPECT_ACK);
   I2C_rep_start();
   I2C_write(I2C_address | I2C_READ, EXPECT_ACK);

   // loop checking 1WB bit for completion of 1-Wire operation 
   // abort if poll limit reached
   status = I2C_read(ACK);
   do
   {
      status = I2C_read(status & STATUS_1WB);
   }
   while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

   I2C_stop();

   // check for failure due to poll limit reached
   if (poll_count >= POLL_LIMIT)
   {
      // handle error
      // ...
      DS2482_reset();
   }
}

//--------------------------------------------------------------------------
// Send 8 bits of read communication to the 1-Wire Net and return the
// result 8 bits read from the 1-Wire Net.
//
// Returns:  8 bits read from 1-Wire Net
//
unsigned char OWReadByte(void)
{
   unsigned char data, status;
   int poll_count = 0;

   // 1-Wire Read Bytes (Case C)
   //   S AD,0 [A] 1WRB [A] Sr AD,1 [A] [Status] A [Status] A\ 
   //                                   \--------/        
   //                     Repeat until 1WB bit has changed to 0
   //   Sr AD,0 [A] SRP [A] E1 [A] Sr AD,1 [A] DD A\ P
   //                                  
   //  [] indicates from slave
   //  DD data read

   I2C_start();
   I2C_write(I2C_address | I2C_WRITE, EXPECT_ACK);
   I2C_write(CMD_1WRB, EXPECT_ACK);
   I2C_rep_start();
   I2C_write(I2C_address | I2C_READ, EXPECT_ACK);

   // loop checking 1WB bit for completion of 1-Wire operation 
   // abort if poll limit reached
   status = I2C_read(ACK);
   do
   {
      status = I2C_read(status & STATUS_1WB);
   }
   while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

   // check for failure due to poll limit reached
   if (poll_count >= POLL_LIMIT)
   {
      // handle error
      // ...
      DS2482_reset();
      return 0;
   }

   I2C_rep_start();
   I2C_write(I2C_address | I2C_WRITE, EXPECT_ACK);
   I2C_write(CMD_SRP, EXPECT_ACK);
   I2C_write(0xE1, EXPECT_ACK);
   I2C_rep_start();
   I2C_write(I2C_address | I2C_READ, EXPECT_ACK);
   data =  I2C_read(NACK);
   I2C_stop();

   return data;
}

//--------------------------------------------------------------------------
// Send 8 bits of communication to the 1-Wire Net and return the
// result 8 bits read from the 1-Wire Net.  The parameter 'sendbyte'
// least significant 8 bits are used and the least significant 8 bits
// of the result is the return byte.
//
// 'sendbyte' - 8 bits to send (least significant byte)
//
// Returns:  8 bits read from sendbyte
//
unsigned char OWTouchByte(unsigned char sendbyte)
{
   if (sendbyte == 0xFF)
      return OWReadByte();
   else
   {  
      OWWriteByte(sendbyte);
      return sendbyte;
   }
}

//--------------------------------------------------------------------------
// The 'OWBlock' transfers a block of data to and from the
// 1-Wire Net. The result is returned in the same buffer.
//
// 'tran_buf' - pointer to a block of unsigned
//              chars of length 'tran_len' that will be sent
//              to the 1-Wire Net
// 'tran_len' - length in bytes to transfer
//
void OWBlock(unsigned char *tran_buf, int tran_len)
{
   int i;

   for (i = 0; i < tran_len; i++)
      tran_buf[i] = OWTouchByte(tran_buf[i]);
}

//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire network
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : no device present
//
int OWFirst()
{
   // reset the search state
   LastDiscrepancy = 0;
   LastDeviceFlag = FALSE;
   LastFamilyDiscrepancy = 0;

   return OWSearch();
}

//--------------------------------------------------------------------------
// Find the 'next' devices on the 1-Wire network
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
int OWNext()
{
   // leave the search state alone
   return OWSearch();
}

//--------------------------------------------------------------------------
// Verify the device with the ROM number in ROM_NO buffer is present.
// Return TRUE  : device verified present
//        FALSE : device not present
//
int OWVerify()
{
   unsigned char rom_backup[8];
   int i,rslt,ld_backup,ldf_backup,lfd_backup;

   // keep a backup copy of the current state
   for (i = 0; i < 8; i++)
      rom_backup[i] = ROM_NO[i];
   ld_backup = LastDiscrepancy;
   ldf_backup = LastDeviceFlag;
   lfd_backup = LastFamilyDiscrepancy;

   // set search to find the same device
   LastDiscrepancy = 64;
   LastDeviceFlag = FALSE;

   if (OWSearch())
   {
      // check if same device found
      rslt = TRUE;
      for (i = 0; i < 8; i++)
      {
         if (rom_backup[i] != ROM_NO[i])
         {
            rslt = FALSE;
            break;
         }
      }
   }
   else
     rslt = FALSE;

   // restore the search state 
   for (i = 0; i < 8; i++)
      ROM_NO[i] = rom_backup[i];
   LastDiscrepancy = ld_backup;
   LastDeviceFlag = ldf_backup;
   LastFamilyDiscrepancy = lfd_backup;

   // return the result of the verify
   return rslt;
}

//--------------------------------------------------------------------------
// Setup the search to find the device type 'family_code' on the next call
// to OWNext() if it is present.
//
void OWTargetSetup(unsigned char family_code)
{
   int i;

   // set the search state to find SearchFamily type devices
   ROM_NO[0] = family_code;
   for (i = 1; i < 8; i++)
      ROM_NO[i] = 0;
   LastDiscrepancy = 64;
   LastFamilyDiscrepancy = 0;
   LastDeviceFlag = FALSE;
}

//--------------------------------------------------------------------------
// Setup the search to skip the current device type on the next call
// to OWNext().
//
void OWFamilySkipSetup()
{
   // set the Last discrepancy to last family discrepancy
   LastDiscrepancy = LastFamilyDiscrepancy;

   // clear the last family discrpepancy
   LastFamilyDiscrepancy = 0;

   // check for end of list
   if (LastDiscrepancy == 0) 
      LastDeviceFlag = TRUE;
}

//--------------------------------------------------------------------------
// The 'OWSearch' function does a general search.  This function
// continues from the previous search state. The search state
// can be reset by using the 'OWFirst' function.
// This function contains one parameter 'alarm_only'.
// When 'alarm_only' is TRUE (1) the find alarm command
// 0xEC is sent instead of the normal search command 0xF0.
// Using the find alarm command 0xEC will limit the search to only
// 1-Wire devices that are in an 'alarm' state.
//
// Returns:   TRUE (1) : when a 1-Wire device was found and its
//                       Serial Number placed in the global ROM 
//            FALSE (0): when no new device was found.  Either the
//                       last search was the last device or there
//                       are no devices on the 1-Wire Net.
//
int OWSearch()
{
   int id_bit_number;
   int last_zero, rom_byte_number, search_result;
   int id_bit, cmp_id_bit;
   unsigned char rom_byte_mask, search_direction, status;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = FALSE;
   crc8 = 0;

   // if the last call was not the last one
   if (!LastDeviceFlag)
   {       
      // 1-Wire reset
      if (!OWReset())
      {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = FALSE;
         LastFamilyDiscrepancy = 0;
         return FALSE;
      }

      // issue the search command 
      OWWriteByte(0xF0);  

      // loop to do the search
      do
      {
         // if this discrepancy if before the Last Discrepancy
         // on a previous next then pick the same as last time
         if (id_bit_number < LastDiscrepancy)
         {
            if ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0)
               search_direction = 1;
            else
               search_direction = 0;
         }
         else
         {
            // if equal to last pick 1, if not then pick 0
            if (id_bit_number == LastDiscrepancy)
               search_direction = 1;
            else
               search_direction = 0;
         }

         // Perform a triple operation on the DS2482 which will perform 2 read bits and 1 write bit
         status = DS2482_search_triplet(search_direction);

         // check bit results in status byte
         id_bit = ((status & STATUS_SBR) == STATUS_SBR);
         cmp_id_bit = ((status & STATUS_TSB) == STATUS_TSB);
         search_direction = ((status & STATUS_DIR) == STATUS_DIR) ? (byte)1 : (byte)0;

         // check for no devices on 1-Wire
         if ((id_bit) && (cmp_id_bit))
            break;
         else
         {
            if ((!id_bit) && (!cmp_id_bit) && (search_direction == 0))
            {
               last_zero = id_bit_number;

               // check for Last discrepancy in family
               if (last_zero < 9)
                  LastFamilyDiscrepancy = last_zero;
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
               ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
               ROM_NO[rom_byte_number] &= (byte)~rom_byte_mask;

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0)
            {
               calc_crc8(ROM_NO[rom_byte_number]);  // accumulate the CRC
               rom_byte_number++;
               rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!((id_bit_number < 65) || (crc8 != 0)))
      {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0)
            LastDeviceFlag = TRUE;

         search_result = TRUE;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || (ROM_NO[0] == 0))
   {
      LastDiscrepancy = 0;
      LastDeviceFlag = FALSE;
      LastFamilyDiscrepancy = 0;
      search_result = FALSE;
   }

   return search_result;
}

//--------------------------------------------------------------------------
// Use the DS2482 help command '1-Wire triplet' to perform one bit of a 1-Wire
// search. This command does two read bits and one write bit. The write bit
// is either the default direction (all device have same bit) or in case of 
// a discrepancy, the 'search_direction' parameter is used. 
//
// Returns � The DS2482 status byte result from the triplet command
//
unsigned char DS2482_search_triplet(int search_direction)
{
   unsigned char status;
   int poll_count = 0;

   // 1-Wire Triplet (Case B)
   //   S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
   //                                         \--------/        
   //                           Repeat until 1WB bit has changed to 0
   //  [] indicates from slave
   //  SS indicates byte containing search direction bit value in msbit
   
   I2C_start();
   I2C_write(I2C_address | I2C_WRITE, EXPECT_ACK);
   I2C_write(CMD_1WT, EXPECT_ACK);
   I2C_write(search_direction ? 0x80 : 0x00, EXPECT_ACK);
   I2C_rep_start();
   I2C_write(I2C_address | I2C_READ, EXPECT_ACK);

   // loop checking 1WB bit for completion of 1-Wire operation 
   // abort if poll limit reached
   status = I2C_read(ACK);
   do
   {
      status = I2C_read(status & STATUS_1WB);
   }
   while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

   I2C_stop();

   // check for failure due to poll limit reached
   if (poll_count >= POLL_LIMIT)
   {
      // handle error
      // ...
      DS2482_reset();
      return 0;
   }

   // return status byte
   return status;
}

//--------------------------------------------------------------------------
// Calculate the CRC8 of the byte value provided with the current 
// global 'crc8' value. 
// Returns current global crc8 value
//
unsigned char calc_crc8(unsigned char data)
{
   int i; 

   // See Application Note 27
   crc8 = crc8 ^ data;
   for (i = 0; i < 8; ++i)
   {
      if (crc8 & 1)
         crc8 = (crc8 >> 1) ^ 0x8c;
      else
         crc8 = (crc8 >> 1);
   }

   return crc8;
}

//---------------------------------------------------------------------------
//-------- Extended 1-Wire functions
//---------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Set the 1-Wire Net communication speed.
//
// 'new_speed' - new speed defined as
//                MODE_STANDARD   0x00
//                MODE_OVERDRIVE  0x01
//
// Returns:  current 1-Wire Net speed
//
int OWSpeed(int new_speed)
{
   // set the speed 
   if (new_speed == MODE_OVERDRIVE)
      c1WS = CONFIG_1WS;
   else
      c1WS = FALSE;

   // write the new config
   DS2482_write_config(c1WS | cSPU | cPPM | cAPU);

   return new_speed;
}

//--------------------------------------------------------------------------
// Set the 1-Wire Net line level pull-up to normal. The DS2482 does only
// allows enabling strong pull-up on a bit or byte event. Consequently this
// function only allows the MODE_STANDARD argument. To enable strong pull-up
// use OWWriteBytePower or OWReadBitPower.  
//
// 'new_level' - new level defined as
//                MODE_STANDARD     0x00
//
// Returns:  current 1-Wire Net level
//
int OWLevel(int new_level)
{
   // function only will turn back to non-strong pull-up
   if (new_level != MODE_STANDARD)
      return MODE_STRONG;

   // clear the strong pull-up bit in the global config state
   cSPU = FALSE;

   // write the new config
   DS2482_write_config(c1WS | cSPU | cPPM | cAPU);

   return MODE_STANDARD;
}

//--------------------------------------------------------------------------
// Send 8 bits of communication to the 1-Wire Net and verify that the
// 8 bits read from the 1-Wire Net is the same (write operation).  
// The parameter 'sendbyte' least significant 8 bits are used.  After the
// 8 bits are sent change the level of the 1-Wire net.
//
// 'sendbyte' - 8 bits to send (least significant bit)
//
// Returns:  TRUE: bytes written and echo was the same, strong pullup now on
//           FALSE: echo was not the same 
//
int OWWriteBytePower(int sendbyte)
{
   // set strong pull-up enable
   cSPU = CONFIG_SPU;

   // write the new config
   if (!DS2482_write_config(c1WS | cSPU | cPPM | cAPU))
      return FALSE;

   // perform write byte
   OWWriteByte(sendbyte);

   return TRUE;
}

//--------------------------------------------------------------------------
// Send 1 bit of communication to the 1-Wire Net and verify that the
// response matches the 'applyPowerResponse' bit and apply power delivery
// to the 1-Wire net.  Note that some implementations may apply the power
// first and then turn it off if the response is incorrect.
//
// 'applyPowerResponse' - 1 bit response to check, if correct then start
//                        power delivery 
//
// Returns:  TRUE: bit written and response correct, strong pullup now on
//           FALSE: response incorrect
//
int OWReadBitPower(int applyPowerResponse)
{
   unsigned char rdbit;

   // set strong pull-up enable
   cSPU = CONFIG_SPU;

   // write the new config
   if (!DS2482_write_config(c1WS | cSPU | cPPM | cAPU))
      return FALSE;

   // perform read bit
   rdbit = OWReadBit();

   // check if response was correct, if not then turn off strong pull-up
   if (rdbit != applyPowerResponse)
   {
      OWLevel(MODE_STANDARD);
      return FALSE;
   }

   return TRUE;
}

//---------------------------------------------------------------------------
//-------- I2C Low-level functions using CMOD
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// I2C Stop - Terminates the data transfer and releases the I2C bus 
//
void I2C_stop(void)
{
   printf("P\n");
   CmodComm2ByteWrite(0xA3, 0, 1);
}

//---------------------------------------------------------------------------
// I2C Start - Issues a start condition and sends address and transfer 
// direction 
// 
// Returns:  TRUE (1): start successful
//           FALSE (0): start failed
//
int I2C_start()
{
   printf("S ");
   CmodComm2ByteWrite(0xA0, 0, 1);
   if (CmodCommResultChar(0) != 0xB0)
   {
      printf("<Failed to create a start condition> ");
      return 0; // Failed to create a start condition
   }
   else
      return 1;
}

//---------------------------------------------------------------------------
// I2C Repeated start - Issues a repeated start condition and sends address 
// and transfer direction 
//
// Returns:  TRUE (1): repeated start successful
//           FALSE (0): repeated start failed
//
int I2C_rep_start()
{
   printf("Sr ");
   CmodComm2ByteWrite(0xA0, 1, 1);
   if (CmodCommResultChar(0) != 0xB0)
   {
      printf("<Failed to create a repeated start condition> ");
      return 0; // Failed to create a start condition
   }
   else
      return 1;
}
 
//---------------------------------------------------------------------------
// I2C write byte - Send one byte to I2C device
// Parameters:  data -  byte to be transfered
//              expect_ack - (1) expect an ACK from the slave
//                           (0) expect a NACK from the slave   
//
// Returns:  TRUE (1): write successful
//           FALSE (0): write failed or ack not what expected
//
int I2C_write(unsigned char data, int expect_ack)
{
   printf("%02X ",data);
   CmodComm2ByteWrite(0xA1, data, 1);
   if (CmodCommResultChar(0) != 0xB1)
   {
      // failed to get acknowledge
      if (expect_ack)
      {
         printf("<Expecting ack from write and did not get it>\n");
         return 0;
      }
   }
 
   return 1;
}

//---------------------------------------------------------------------------
// I2C Read - read one byte from the I2C device
// 
// Parameters:  ack - (1) send ACK, request more data from device
//                    (0) send NAK, read is followed by a stop condition 
//
// Returns:  byte read from I2C device
//
unsigned char I2C_read(int ack)
{
   int data;

   CmodComm2ByteWrite(0xA2, ack, 2);
   if (CmodCommResultChar(1) != 0xB2)
   {
      printf("<Failure during write> ");
      return 0;
   }
   data = CmodCommResultChar(0);

   if (ack)
      printf("[%02X] ",data);
   else
      printf("[%02X*] ",data);

   return (unsigned char)data;
}

//---------------------------------------------------------------------------
// msDelay - delay a set number of milliseconds
//
void msDelay(int delay)
{
   Sleep(delay); 
}

//---------------------------------------------------------------------------
//-------- CMOD code
//---------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Load the CMOD driver and get a pointers to the functions
//
int LoadCMOD(void)
{
   // attempt to get a SHandle to the TMEX driver
   hInst = LoadLibrary(L"CMODCOMM.DLL");

   // get a pointer to the function needed by loopit32
   if (hInst != NULL)
   {
      CmodBoardConnect = GetProcAddress(hInst,"CmodBoardConnect");
      if (CmodBoardConnect == NULL) printf("could not find CmodBoardConnect\n"); //????
      CmodBoardConnected = GetProcAddress(hInst,"CmodBoardConnected");
      if (CmodBoardConnected == NULL) printf("could not find CmodBoardConnected\n"); //????
      CmodBoardDisconnect = GetProcAddress(hInst,"CmodBoardDisconnect");
      if (CmodBoardDisconnect == NULL) printf("could not find CmodBoardDisconnect\n"); //????
      CmodComm2ByteWrite = GetProcAddress(hInst,"CmodComm2ByteWrite");
      if (CmodComm2ByteWrite == NULL) printf("could not find CmodComm2ByteWrite\n"); //????
      CmodCommResultChar = GetProcAddress(hInst,"CmodCommResultChar");
      if (CmodCommResultChar == NULL) printf("could not find CmodCommResultChar\n"); //????
   
      // check to make sure got ALL of the functions needed
      if ((CmodBoardConnect == NULL) || (CmodBoardConnected == NULL) ||
         (CmodBoardDisconnect == NULL) || (CmodComm2ByteWrite == NULL) ||
         (CmodCommResultChar == NULL))
      {
         printf("ERROR, could not get a pointer to all"
                " of the CMOD functions needed\n");
         return 0;
      }
      return 1;
   }
   else
      return 0;
}

//--------------------------------------------------------------------------
// UnLoad the CMOD driver
//
void UnLoadCMOD(void)
{
   // release the TMEX driver
   FreeLibrary(hInst);
}

