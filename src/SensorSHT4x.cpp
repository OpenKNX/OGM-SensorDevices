// #include "IncludeManager.h"
#ifdef SENSORMODULE
    #include "SensorSHT4x.h"
    #include <Wire.h>
    #include <math.h>

SensorSHT4x::SensorSHT4x(uint16_t iMeasureTypes, TwoWire* iWire)
    : Sensor(iMeasureTypes, iWire, 0) {}; // 0 triggers auto-detection in begin()

SensorSHT4x::SensorSHT4x(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress)
    : Sensor(iMeasureTypes, iWire, iAddress) {};

uint8_t SensorSHT4x::getSensorClass()
{
    return SENS_SHT4X;
}

std::string SensorSHT4x::logPrefix()
{
    return "Sensor<SHT4x>";
}

bool SensorSHT4x::begin()
{
    bool lResult = false;
    logInfoP("Starting sensor SHT4x... ");
    if (pI2CAddress == 0)
    {
        pI2CAddress = SHT4X_ADDR;
        lResult = Sensor::begin();
        if (!lResult)
        {
            pI2CAddress = SHT4X_ADDR_ALT;
            lResult = Sensor::begin();
        }
    }
    else
    {
        lResult = Sensor::begin();
    }
    logResult(lResult);
    return lResult;
}

uint8_t SensorSHT4x::getI2cSpeed()
{
    return 4; // n * 100kHz
}

bool SensorSHT4x::prepareTemperatureOffset(float iTempOffset)
{
    pTempOffset = iTempOffset;
    return true;
}

void SensorSHT4x::sensorLoopInternal()
{
    static uint32_t sCommandSent = 0;
    switch (pSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
        case Calibrate:
            writeCommand(SHT4X_MEAS_HIGHREP);
            Sensor::sensorLoopInternal();
            break;
        case Finalize:
            if (delayCheck(pSensorStateDelay, 100))
            {
                getTempHum();
                sCommandSent = 0;
                pSensorState = Running;
                pSensorStateDelay = millis();
            }
            break;
        case Running:
            // quick hack: We use a static member to toggle between acquiring values and fetching them
            if (sCommandSent && delayCheck(sCommandSent, 15))
            {
                getTempHum();
                sCommandSent = 0;
            }
            else if (delayCheck(pSensorStateDelay, 2000))
            {
                writeCommand(SHT4X_MEAS_HIGHREP);
                sCommandSent = millis();
                pSensorStateDelay = millis();
            }
            break;
        default:
            pSensorStateDelay = millis();
            break;
    }
}

float SensorSHT4x::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
        case Temperature:
            return mTemp - pTempOffset;
        case Humidity:
            if (pTempOffset == 0.0f || mHumidity == NO_NUM || mTemp == NO_NUM)
                return mHumidity;
            {
                float tMeas = mTemp;
                float tReal = mTemp - pTempOffset;
                float rhCompensated = mHumidity * expf(17.269f * (tMeas / (237.3f + tMeas) - tReal / (237.3f + tReal)));
                if (rhCompensated < 0.0f) rhCompensated = 0.0f;
                if (rhCompensated > 100.0f) rhCompensated = 100.0f;
                return rhCompensated;
            }
        default:
            break;
    }
    return NO_NUM;
}

uint8_t SensorSHT4x::crc8(const uint8_t* iData, uint8_t iLen)
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

bool SensorSHT4x::getTempHum()
{
    uint8_t readbuffer[6];

    pWire->requestFrom(pI2CAddress, 6);
    if (pWire->available() != 6)
        return false;
    for (uint8_t i = 0; i < 6; i++)
    {
        readbuffer[i] = pWire->read();
    }
    if (readbuffer[2] != crc8(readbuffer, 2) ||
        readbuffer[5] != crc8(readbuffer + 3, 2))
        return false;

    int32_t lTemp = (int32_t)(((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
    // T[°C] = -45 + 175 * raw / 65535  (same as SHT3x)
    lTemp = ((4375 * lTemp) >> 14) - 4500;
    mTemp = (float)lTemp / 100.0f;

    uint32_t lHum = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
    // RH[%] = -6 + 125 * raw / 65535  (SHT4x-specific; differs from SHT3x)
    // in hundredths of %: ((raw * 12500) >> 16) - 600
    int32_t lHumInt = (int32_t)((lHum * 12500u) >> 16) - 600;
    if (lHumInt < 0) lHumInt = 0;
    if (lHumInt > 10000) lHumInt = 10000;
    mHumidity = (float)lHumInt / 100.0f;

    return true;
}

void SensorSHT4x::writeCommand(uint8_t iCmd)
{
    pWire->beginTransmission(pI2CAddress);
    pWire->write(iCmd);
    pWire->endTransmission();
}
#endif
