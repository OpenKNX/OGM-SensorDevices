// #include "IncludeManager.h"
#ifdef SENSORMODULE
    #include "SensorSGP30.h"
    #include <Wire.h>
    #include <math.h>

static const uint8_t sMagicWord[] = {0x53, 0x47, 0x50, 0x33}; // "SGP3"

SensorSGP30::SensorSGP30(uint16_t iMeasureTypes, TwoWire* iWire)
    : Sensor(iMeasureTypes, iWire, SGP30_ADDR), Adafruit_SGP30() {}

uint8_t SensorSGP30::getSensorClass()
{
    return SENS_SGP30;
}

std::string SensorSGP30::logPrefix()
{
    return "Sensor<SGP30>";
}

bool SensorSGP30::begin()
{
    logInfoP("Starting sensor SGP30... ");
    bool lResult = Sensor::begin();
    if (lResult) lResult = Adafruit_SGP30::begin(pWire);
    if (lResult)
        logDebugP("Serial: 0x%04X%04X%04X",
                  Adafruit_SGP30::serialnumber[0],
                  Adafruit_SGP30::serialnumber[1],
                  Adafruit_SGP30::serialnumber[2]);
    if (lResult) lResult = Adafruit_SGP30::IAQinit();
    logResult(lResult);
    return lResult;
}

uint8_t SensorSGP30::getI2cSpeed()
{
    return 4; // n * 100kHz
}

uint32_t SensorSGP30::getAbsoluteHumidity(float iTemp, float iHum)
{
    // AH [mg/m³] = 1000 * RH/100 * 216.7 * 6.112 * exp(17.67*T/(T+243.5)) / (273.15+T)
    float absHum = 1000.0f * (iHum / 100.0f) * 6.112f
                   * expf(17.67f * iTemp / (iTemp + 243.5f))
                   * 216.7f / (273.15f + iTemp);
    return (uint32_t)absHum;
}

void SensorSGP30::processHumidity()
{
#if defined(KoSENS_Temp) && defined(KoSENS_Hum)
    if (!KoSENS_Temp.initialized() || !KoSENS_Hum.initialized())
        return;
    float lTemp = KoSENS_Temp.value(DPT_Value_Temp);
    float lHum  = KoSENS_Hum.value(DPT_Value_Humidity);
    if (lTemp < -40.0f || lTemp > 80.0f || lHum < 0.0f || lHum > 100.0f)
        return;
    mAbsoluteHumidity = getAbsoluteHumidity(lTemp, lHum);
    mHumidityPending = true;
    // logDebugP("Humidity compensation: T=%.1f RH=%.1f", lTemp, lHum);
#endif
}

void SensorSGP30::sensorLoopInternal()
{
    switch (pSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
        case Calibrate:
            if (delayCheck(pSensorStateDelay, 100))
            {
                if (mFlashBuffer != nullptr)
                {
                    uint16_t eco2Base = ((uint16_t)mFlashBuffer[1] << 8) | mFlashBuffer[0];
                    uint16_t tvocBase = ((uint16_t)mFlashBuffer[3] << 8) | mFlashBuffer[2];
                    Adafruit_SGP30::setIAQBaseline(eco2Base, tvocBase);
                    logDebugP("SGP30: Baseline restored from flash");
                }
                Sensor::sensorLoopInternal(); // → Finalize
            }
            break;
        case Finalize:
        {
            static uint32_t sMeasTimer = 0;
            if (mPhase == 0)
            {
                // transition to Running after 15s warmup
                if (delayCheck(pSensorStateDelay, 15000))
                {
                    sMeasTimer = 0;
                    mPhase = 0;
                    pSensorState = Running;
                    pSensorStateDelay = millis();
                    break;
                }
                // trigger measurement every 1s (required by SGP30 internal algorithm)
                if (delayCheck(sMeasTimer, 1000))
                {
                    triggerMeasure();
                    mCommandSent = millis();
                    sMeasTimer = millis();
                    mPhase = 2;
                }
            }
            else if (mPhase == 2 && delayCheck(mCommandSent, 12))
            {
                readMeasureResult(); // result not used during warmup
                mPhase = 0;
            }
            break;
        }
        case Running:
            if (mPhase == 0 && delayCheck(pSensorStateDelay, 1000))
            {
                processHumidity();
                if (mHumidityPending)
                {
                    sendHumidityCompensation();
                    mCommandSent = millis();
                    mPhase = 1;
                }
                else
                {
                    triggerMeasure();
                    mCommandSent = millis();
                    mPhase = 2;
                }
                pSensorStateDelay = millis();
            }
            else if (mPhase == 1 && delayCheck(mCommandSent, 5))
            {
                triggerMeasure();
                mCommandSent = millis();
                mPhase = 2;
            }
            else if (mPhase == 2 && delayCheck(mCommandSent, 12))
            {
                readMeasureResult();
                mPhase = 0;
            }
            break;
        default:
            pSensorStateDelay = millis();
            break;
    }
}

float SensorSGP30::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
        case Voc:
            return mTVOC;
        case Co2Calc:
            return mECO2;
        default:
            break;
    }
    return NO_NUM;
}

uint16_t SensorSGP30::sensorFlashSize()
{
    return SGP30_SAVE_SIZE;
}

void SensorSGP30::sensorReadFlash(const uint8_t* iBuffer, const uint16_t iSize)
{
    bool lValid = (iBuffer[0] == 1);
    logDebugP("Reading SGP30 baseline from Flash");
    for (uint8_t i = 1; i < 5 && lValid; i++)
        lValid = lValid && (iBuffer[i] == sMagicWord[i - 1]);

    mFlashBuffer = lValid ? iBuffer + 5 : nullptr;

    logIndentUp();
    if (!lValid)
        logDebugP("No valid baseline in Flash");
    else
    {
        logDebugP("SGP30 baseline found in Flash");
        openknx.flash.read(sensorFlashSize() - 1);
    }
    logIndentDown();
}

void SensorSGP30::sensorSavePower()
{
    uint16_t eco2Base = 0, tvocBase = 0;
    if (pSensorState == Running && Adafruit_SGP30::getIAQBaseline(&eco2Base, &tvocBase))
    {
        mWorkBuffer[0] = eco2Base & 0xFF;
        mWorkBuffer[1] = (eco2Base >> 8) & 0xFF;
        mWorkBuffer[2] = tvocBase & 0xFF;
        mWorkBuffer[3] = (tvocBase >> 8) & 0xFF;
        mWorkBufferInitialized = true;
    }
}

void SensorSGP30::sensorWriteFlash()
{
    if (!mWorkBufferInitialized) sensorSavePower();
    if (!mWorkBufferInitialized) return;
    openknx.flash.writeByte(1); // version
    for (uint8_t i = 0; i < 4; i++)
        openknx.flash.writeByte(sMagicWord[i]);
    openknx.flash.write(mWorkBuffer, 4);
    mWorkBufferInitialized = false;
    logDebugP("SGP30: Baseline written to Flash");
}

uint8_t SensorSGP30::crc8(const uint8_t* iData, uint8_t iLen)
{
    const uint8_t cPolynomial = 0x31;
    uint8_t lCrc = 0xFF;
    for (uint8_t j = iLen; j; --j)
    {
        lCrc ^= *iData++;
        for (uint8_t i = 8; i; --i)
            lCrc = (lCrc & 0x80) ? (lCrc << 1) ^ cPolynomial : (lCrc << 1);
    }
    return lCrc;
}

void SensorSGP30::sendHumidityCompensation()
{
    uint8_t buf[2] = {(uint8_t)(mAbsoluteHumidity >> 8), (uint8_t)(mAbsoluteHumidity & 0xFF)};
    pWire->beginTransmission(pI2CAddress);
    pWire->write(0x20); // CMD 0x2061
    pWire->write(0x61);
    pWire->write(buf[0]);
    pWire->write(buf[1]);
    pWire->write(crc8(buf, 2));
    pWire->endTransmission();
    mHumidityPending = false;
}

void SensorSGP30::triggerMeasure()
{
    pWire->beginTransmission(pI2CAddress);
    pWire->write(0x20); // CMD 0x2008
    pWire->write(0x08);
    pWire->endTransmission();
}

bool SensorSGP30::readMeasureResult()
{
    uint8_t buf[6];
    pWire->requestFrom(pI2CAddress, (uint8_t)6);
    if (pWire->available() != 6)
        return false;
    for (uint8_t i = 0; i < 6; i++)
        buf[i] = pWire->read();
    if (buf[2] != crc8(buf, 2) || buf[5] != crc8(buf + 3, 2))
        return false;
    mECO2 = (float)(((uint16_t)buf[0] << 8) | buf[1]);
    mTVOC = (float)(((uint16_t)buf[3] << 8) | buf[4]);
    return true;
}
#endif
