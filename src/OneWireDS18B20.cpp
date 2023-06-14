#include <Wire.h>
#include <Arduino.h>
#include <math.h>

#include "OneWireDS2482.h"
#include "OneWireDS18B20.h"
#ifdef COUNT_1WIRE_CHANNEL

// #define DebugInfoTemp

OneWireDS18B20::OneWireDS18B20(tIdRef iId)
    : OneWire(iId)
{}

void OneWireDS18B20::loop() {

    switch (mState)
    {
    case Startup:
        init(true);
        mState = Idle;
        break;
    case StartMeasurement:
        mState = startConversionTemp() ? GetMeasure : Error;
        pDelay = millis();
        break;
    case GetMeasure:
        if (delayCheck(pDelay, 750)) {
            mState = updateTemp() ? Idle : Error;
            pDelay = millis();
        }
        break;
    case Idle:
        if (delayCheck(pDelay, 2000)) {
            mState = StartMeasurement;
            pDelay = millis();
        }
        break;
    default:
        // error case, we stay here as long as there is a short on 1W-Line
        mState = pBM->readStatusShortDet() ? Error : Idle;
        break;
    }
}

void OneWireDS18B20::init(bool iIsactive)
{
    mIsActive = iIsactive;

    if (!mParasite && readPowerSupply())
        mParasite = true;
    // mBitResolution = max(mBitResolution, resolution());
    resolution(mBitResolution);
}

float OneWireDS18B20::getTemp()
{
    return mTemp;
}

bool OneWireDS18B20::getValue(float& eValue, uint8_t iModelFunction)
{
    // there is just one model function, so we ignore the parameter
    eValue = mTemp;
    return pValid && pMode == Connected;
}

bool OneWireDS18B20::setParameter(OneWire::ModelParameter iModelParameter, uint8_t iValue, uint8_t iModelFunction)
{
    bool lResult = true;
    switch (iModelParameter)
    {
        case MeasureResolution:
            mBitResolution = iModelFunction + 9;
            break;
        default:
            break;
    }
    return lResult;
}

bool OneWireDS18B20::startConversionTemp()
{
    wireSelectThisDevice();
    pBM->wireWriteByte(STARTCONVO, 1); // Wandlung mit aktivierter parasitärer Versorgung starten
    return true;
}

// void test()
// {
//   // Convert the data to actual temperature
//   // because the result is a 16 bit signed integer, it should
//   // be stored to an "int16_t" type, which is always 16 bits
//   // even when compiled on a 32 bit processor.
//   int16_t raw = (data[1] << 8) | data[0];
//   if (type_s) {
//     raw = raw << 3; // 9 bit resolution default
//     if (data[7] == 0x10) {
//       // "count remain" gives full 12 bit resolution
//       raw = (raw & 0xFFF0) + 12 - data[6];
//     }
//   } else {
//     byte cfg = (data[4] & 0x60);
//     // at lower res, the low bits are undefined, so let's zero them
//     if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
//     else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
//     else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
//     //// default is 12 bit resolution, 750 ms conversion time
//   }
//   celsius = (float)raw / 16.0;

// } 

bool OneWireDS18B20::updateTemp()
{
    uint8_t lResolution = resolution();
    int16_t lTempRaw = (mScratchPad[1] << 8) | mScratchPad[0];
    bool lResult = false;
    if (lResolution) {
        if (Family() == MODEL_DS18S20) {
            mTemp = (float)lTempRaw / 2.0;
        } else {
            uint16_t lShift = 0xFFFF << abs(lResolution - 12);
            lTempRaw &= lShift;
            mTemp = (float)lTempRaw / 16.0;
        }
    #ifdef DebugInfoTemp
        logDebugP("Temp = %0.1f°C --- ", mTemp);
        printHEX("Id: ", Id(), 7);
    #endif
        lResult = true;
    }
    pValid = lResult;
    return lResult;
}

// reads the device's power requirements
bool OneWireDS18B20::readPowerSupply()
{
    bool lResult = false;
    wireSelectThisDevice();
    pBM->wireWriteByte(READPOWERSUPPLY);
    if (pBM->wireReadBit() == 0)
        lResult = true;
    pBM->wireReset();
    return lResult;
}

// returns true if the bus requires parasite power
bool OneWireDS18B20::isParasitePowerMode()
{
    return mParasite;
}

// attempt to determine if the device at the given address is connected to the bus
bool OneWireDS18B20::isConnected()
{
    readScratchPad();
    return (pBM->crc8(mScratchPad, 8) == mScratchPad[SCRATCHPAD_CRC]);
}

// returns the global resolution
uint8_t OneWireDS18B20::resolution()
{
    // DS18S20 has a fixed resolution of 9 bits but getTemp calculates
    // a full 12 bits resolution and we need 750ms convert time
    if (isConnected())
    {
        // returned values are 9-12
        return (pId[0] == MODEL_DS18S20) ? 9 : ((mScratchPad[CONFIGURATION] >> 5) & 3) + 9;
    }
    return 0;
}

// set resolution of all devices to 9, 10, 11, or 12 bits
// if new resolution is out of range, it is constrained.
bool OneWireDS18B20::resolution(uint8_t iResolution)
{
    mBitResolution = constrain(iResolution, 9, 12);

    if (isConnected())
    {
        // DS18S20 has a fixed 9-bit resolution
        if (pId[0] != MODEL_DS18S20)
        {
            mScratchPad[CONFIGURATION] = ((mBitResolution - 9) << 5) | 0x0F;
            writeScratchPad();
        }
        return true; // new value set
    }
    return false;
}

// writes device's scratch pad
void OneWireDS18B20::writeScratchPad()
{
    wireSelectThisDevice();
    pBM->wireWriteByte(WRITESCRATCH);
    pBM->wireWriteByte(mScratchPad[HIGH_ALARM_TEMP]); // high alarm temp
    pBM->wireWriteByte(mScratchPad[LOW_ALARM_TEMP]);  // low alarm temp
    // DS18S20 does not use the configuration register
    if (pId[0] != MODEL_DS18S20)
        pBM->wireWriteByte(mScratchPad[CONFIGURATION]); // configuration
    pBM->wireReset();
    // save the newly written values to eeprom
    pBM->wireWriteByte(COPYSCRATCH, mParasite);
    if (mParasite)
        delay(10); // 10ms delay
    pBM->wireReset();
}

// read device's scratch pad
void OneWireDS18B20::readScratchPad()
{
    // send the command
    wireSelectThisDevice();
    pBM->wireWriteByte(READSCRATCH); // 0xBE  // Read EEPROM

    // TODO => collect all comments &  use simple loop
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
    //
    for(uint8_t i=0; i<9; i++)
    {
        mScratchPad[i] = pBM->wireReadByte();
    }
    pBM->wireReset();
}
#endif
