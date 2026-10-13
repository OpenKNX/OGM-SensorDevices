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
    Adafruit_SGP30::setHumidity(getAbsoluteHumidity(lTemp, lHum));
    logDebugP("Humidity compensation: T=%.1f RH=%.1f", lTemp, lHum);
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
            // measure every 1s during warmup (required by SGP30 internal algorithm)
            if (delayCheck(sMeasTimer, 1000))
            {
                Adafruit_SGP30::IAQmeasure();
                sMeasTimer = millis();
            }
            // transition to Running after 15s warmup
            if (delayCheck(pSensorStateDelay, 15000))
            {
                sMeasTimer = 0;
                pSensorState = Running;
                pSensorStateDelay = millis();
            }
            break;
        }
        case Running:
            if (delayCheck(pSensorStateDelay, 1000))
            {
                processHumidity();
                if (Adafruit_SGP30::IAQmeasure())
                {
                    mTVOC = (float)Adafruit_SGP30::TVOC;
                    mECO2 = (float)Adafruit_SGP30::eCO2;
                }
                pSensorStateDelay = millis();
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
#endif
