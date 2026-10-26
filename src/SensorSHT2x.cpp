// #include "IncludeManager.h"
#ifdef SENSORMODULE
    #include "SensorSHT2x.h"
    #include <Wire.h>
    #include <math.h>

SensorSHT2x::SensorSHT2x(uint16_t iMeasureTypes, TwoWire* iWire)
    : Sensor(iMeasureTypes, iWire, SHT2X_ADDR) {}

uint8_t SensorSHT2x::getSensorClass()
{
    return SENS_SHT2X;
}

std::string SensorSHT2x::logPrefix()
{
    return "Sensor<SHT2x>";
}

bool SensorSHT2x::begin()
{
    bool lResult = false;
    logInfoP("Starting sensor SHT2x... ");
    lResult = Sensor::begin();
    logResult(lResult);
    return lResult;
}

uint8_t SensorSHT2x::getI2cSpeed()
{
    return 4; // n * 100kHz
}

bool SensorSHT2x::prepareTemperatureOffset(float iTempOffset)
{
    pTempOffset = iTempOffset;
    return true;
}

void SensorSHT2x::sensorLoopInternal()
{
    switch (pSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
        case Calibrate:
            Sensor::sensorLoopInternal();
            break;
        case Finalize:
            if (delayCheck(pSensorStateDelay, 100))
            {
                mPhase = 0;
                pSensorState = Running;
                pSensorStateDelay = millis();
            }
            break;
        case Running:
            // Phase 0: every 2s trigger temperature measurement
            // Phase 1: after 100ms read T, trigger RH measurement
            // Phase 2: after 40ms read RH
            if (mPhase == 0 && delayCheck(pSensorStateDelay, 2000))
            {
                writeCommand(SHT2X_MEAS_T_NOHOLD);
                mCommandSent = millis();
                mPhase = 1;
            }
            else if (mPhase == 1 && delayCheck(mCommandSent, 100))
            {
                readRawValue(mTemp, true);
                writeCommand(SHT2X_MEAS_RH_NOHOLD);
                mCommandSent = millis();
                mPhase = 2;
            }
            else if (mPhase == 2 && delayCheck(mCommandSent, 40))
            {
                readRawValue(mHumidity, false);
                mPhase = 0;
                pSensorStateDelay = millis();
            }
            break;
        default:
            pSensorStateDelay = millis();
            break;
    }
}

float SensorSHT2x::measureValue(MeasureType iMeasureType)
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

uint8_t SensorSHT2x::crc8(const uint8_t* iData, uint8_t iLen)
{
    // CRC-8: polynomial 0x31, init 0x00 (SHT2x differs from SHT3x/SHT4x which use init 0xFF)
    const uint8_t cPolynomial = 0x31;
    uint8_t lCrc = 0x00;
    for (uint8_t j = iLen; j; --j)
    {
        lCrc ^= *iData++;
        for (uint8_t i = 8; i; --i)
            lCrc = (lCrc & 0x80) ? (lCrc << 1) ^ cPolynomial : (lCrc << 1);
    }
    return lCrc;
}

bool SensorSHT2x::readRawValue(float& eValue, bool iIsTemp)
{
    uint8_t buf[3];
    pWire->requestFrom(pI2CAddress, (uint8_t)3);
    if (pWire->available() != 3)
        return false;
    for (uint8_t i = 0; i < 3; i++)
        buf[i] = pWire->read();

    if (buf[2] != crc8(buf, 2))
        return false;

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    raw &= 0xFFFC; // clear 2 status bits

    if (iIsTemp)
    {
        // T[°C] = -46.85 + 175.72 * raw / 65536
        eValue = -46.85f + 175.72f * (float)raw / 65536.0f;
    }
    else
    {
        // RH[%] = -6 + 125 * raw / 65536
        float rh = -6.0f + 125.0f * (float)raw / 65536.0f;
        if (rh < 0.0f) rh = 0.0f;
        if (rh > 100.0f) rh = 100.0f;
        eValue = rh;
    }
    return true;
}

void SensorSHT2x::writeCommand(uint8_t iCmd)
{
    pWire->beginTransmission(pI2CAddress);
    pWire->write(iCmd);
    pWire->endTransmission();
}
#endif
