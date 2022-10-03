// #include "IncludeManager.h"
#ifdef SENSORMODULE
#include <Wire.h>
#include "SensorSHT3x.h"

SensorSHT3x::SensorSHT3x(uint16_t iMeasureTypes)
    : Sensor(iMeasureTypes, SHT3X_ADDR){};

SensorSHT3x::SensorSHT3x(uint16_t iMeasureTypes, uint8_t iAddress)
    : Sensor(iMeasureTypes, iAddress){};

uint8_t SensorSHT3x::getSensorClass()
{
    return SENS_SHT3X;
}

void SensorSHT3x::sensorLoopInternal()
{
    static uint32_t sCommandSent = 0;
    switch (gSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
        case Calibrate:
            writeCommand(SHT3X_MEAS_HIGHREP);
            Sensor::sensorLoopInternal();
            break;
        case Finalize:
            if (delayCheck(pSensorStateDelay, 100)) {
                // start first measurement
                getTempHum();
                sCommandSent = 0;
                gSensorState = Running;
                pSensorStateDelay = millis();
            }
            break;
        case Running:
            // quick hack: We use a static member to toggle between aquirering values and fetching them
            if (sCommandSent && delayCheck(sCommandSent, 50)) {
                getTempHum();
                sCommandSent = 0;
            } 
            else if (delayCheck(pSensorStateDelay, 2000)) {
                writeCommand(SHT3X_MEAS_HIGHREP);
                sCommandSent = millis();
                pSensorStateDelay = millis();
            }
            break;
        default:
            pSensorStateDelay = millis();
            break;
    }
}

float SensorSHT3x::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
    case Temperature:
        // hardware calibration
        return mTemp;
        break;
    case Humidity:
        return mHumidity;
        break;
    default:
        break;
    }
    return NO_NUM;
}

bool SensorSHT3x::begin()
{
    bool lResult = false;
    printDebug("Starting sensor SHT3x... ");
    lResult = Sensor::begin();
    printResult(lResult);
    return lResult;
}

uint8_t SensorSHT3x::getI2cSpeed()
{
    return 4; // n * 100kHz
}

float SensorSHT3x::readTemperature(void)
{
    if (!getTempHum())
        return NO_NUM;
    return mTemp;
}

float SensorSHT3x::readHumidity(void)
{
    if (!getTempHum())
        return NO_NUM;
    return mHumidity;
}

void SensorSHT3x::reset(void)
{
    writeCommand(SHT3X_SOFTRESET);
    delay(10);
}

void SensorSHT3x::heater(bool iOn)
{
    if (iOn)
        writeCommand(SHT3X_HEATER_ENABLE);
    else
        writeCommand(SHT3X_HEATER_DISABLE);
}

uint8_t SensorSHT3x::crc8(const uint8_t *iData, uint8_t iLen)
{
    const uint8_t cPolynomial(0x31);
    uint8_t lCrc(0xFF);

    for (uint8_t j = iLen; j; --j)
    {
        lCrc ^= *iData++;
        for (uint8_t i = 8; i; --i)
            lCrc = (lCrc & 0x80) ? (lCrc << 1) ^ cPolynomial : (lCrc << 1);
    }
    return lCrc;
}

bool SensorSHT3x::getTempHum(void)
{
    uint8_t readbuffer[6];

    // writeCommand(SHT3X_MEAS_HIGHREP);

    // delay(50);
    gWire.requestFrom(gAddress, 6);
    if (gWire.available() != 6)
        return false;
    for (uint8_t i = 0; i < 6; i++)
    {
        readbuffer[i] = gWire.read();
    }
    if (readbuffer[2] != crc8(readbuffer, 2) ||
        readbuffer[5] != crc8(readbuffer + 3, 2))
        return false;

    int32_t lTemp = (int32_t)(((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
    // simplified (65536 instead of 65535) integer version of:
    // temp = (stemp * 175.0f) / 65535.0f - 45.0f;
    lTemp = ((4375 * lTemp) >> 14) - 4500;
    mTemp = (float)lTemp / 100.0f;

    uint32_t lHum = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
    // simplified (65536 instead of 65535) integer version of:
    // humidity = (shum * 100.0f) / 65535.0f;
    lHum = (625 * lHum) >> 12;
    mHumidity = (float)lHum / 100.0f;

    return true;
}

void SensorSHT3x::writeCommand(uint16_t iCmd)
{
    gWire.beginTransmission(gAddress);
    gWire.write(iCmd >> 8);
    gWire.write(iCmd & 0xFF);
    gWire.endTransmission();
}
#endif
