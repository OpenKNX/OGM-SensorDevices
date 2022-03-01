/*
   OneWire_Multisensor.h

    Created on: 09.03.2018
        Author: Jens
*/

#pragma once

#include <inttypes.h>
#include "OneWire.h"
#ifdef COUNT_1WIRE_CHANNEL
#include "OneWireDS2482.h"

// byte 0: temperature LSB
// byte 1: temperature MSB
// byte 2: high alarm temp
// byte 3: low alarm temp
// byte 4: DS18S20: store for crc
//         DS18B20 & DS1822: configuration register
// byte 5: internal use & crc
// byte 6: DS18S20: COUNT_REMAIN
//         DS18B20 & DS1822: store for crc
// byte 7: DS18S20: COUNT_PER_C
//         DS18B20 & DS1822: store for crc
// byte 8: SCRATCHPAD_CRC

// Model commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy EEPROM
#define READSCRATCH     0xBE  // Read EEPROM
#define WRITESCRATCH    0x4E  // Write to EEPROM
#define RECALLSCRATCH   0xB8  // Reload from last known
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition

// Scratchpad locations
#define DS1820_TEMP_LSB 0
#define DS1820_TEMP_MSB 1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

class OneWireDS18B20 : public OneWire
{
    enum StateSensorTemp
    {
        Startup,
        StartMeasurement,
        GetMeasure,
        Idle,
        Error
    };

  public:
    OneWireDS18B20(tIdRef iId);

    void init( bool iIsActive );

    float getTemp();
    bool getValue(float &eValue, uint8_t iModelFunction) override;

    bool setParameter(OneWire::ModelParameter iModelParameter, uint8_t iValue, uint8_t iModelFunction);

    bool lResult = false;

    // attempt to determine if the device at the given address is connected to the bus
    bool isConnected();

    // get global resolution
    uint8_t resolution();

    // set resolution of a device to 9, 10, 11, or 12 bits
    bool resolution(uint8_t iResolution);

    // returns true if the bus requires parasite power
    bool isParasitePowerMode();

  private:
    ScratchPad mScratchPad;

    StateSensorTemp mState = Startup;

    bool mIsActive = false;
    uint8_t mBitResolution = 255;
    bool mParasite;
    float mTemp;

    void loop() override;
    bool startConversionTemp();
    bool updateTemp();
    // read device's scratchpad
    void readScratchPad();
    // write device's scratchpad
    void writeScratchPad();
    // read device's power requirements
    bool readPowerSupply();
};
#endif
