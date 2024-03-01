// #include "IncludeManager.h"
#ifdef SENSORMODULE
    #include "SensorIAQCore.h"
    #include <Wire.h>

SensorIAQCore::SensorIAQCore(uint16_t iMeasureTypes, TwoWire &iWire)
    : Sensor(iMeasureTypes, iWire, IAQCORE_I2C_ADDR){};

SensorIAQCore::SensorIAQCore(uint16_t iMeasureTypes, TwoWire &iWire, uint8_t iAddress)
    : Sensor(iMeasureTypes, iWire, iAddress){};

uint8_t SensorIAQCore::getSensorClass()
{
    return SENS_IAQCORE;
}

std::string SensorIAQCore::logPrefix()
{
    return "Sensor<IAQCore>";
}

void SensorIAQCore::sensorLoopInternal()
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
            if (delayCheck(pSensorStateDelay, 1000))
            {
                // start first measurement
                if (getSensorData())
                    pSensorState = Running;
                pSensorStateDelay = millis();
            }
            break;
        case Running:
            if (delayCheck(pSensorStateDelay, 2000))
            {
                getSensorData();
                pSensorStateDelay = millis();
            }
            break;
        default:
            pSensorStateDelay = millis();
            break;
    }
}

float SensorIAQCore::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
        case Voc:
            return mVoc;
            break;
        case Co2Calc:
            return mCo2;
            break;
        case Accuracy:
            return (mBuffer[IAQCORE_STATUS_OFFSET] & IAQCORE_STATUS_RUNIN) ? 0.0 : 100.0;
        default:
            break;
    }
    return NO_NUM;
}

bool SensorIAQCore::begin()
{
    bool lResult = false;
    logDebugP("Starting sensor IAQCore... ");
    lResult = Sensor::begin();
    logResult(lResult);
    return lResult;
}

bool SensorIAQCore::getSensorData()
{
    // clear read buffer
    memset(mBuffer, 0, sizeof(mBuffer));
    // request sensor data

    pWire.requestFrom(pI2CAddress, IAQCORE_READ_ALL);
    if (pWire.available() != IAQCORE_READ_ALL)
        return false;
    for (uint8_t i = 0; i < IAQCORE_READ_ALL; i++)
        mBuffer[i] = pWire.read();
    uint8_t lStatus = mBuffer[IAQCORE_STATUS_OFFSET];
    if (lStatus & IAQCORE_STATUS_ERROR)
        return false;
    if ((lStatus & IAQCORE_STATUS_BUSY) == 0)
    {
        mCo2 = (float)(((uint16_t)mBuffer[IAQCORE_CO2_PREDICTION_MSB_OFFSET] << 8) | mBuffer[IAQCORE_CO2_PREDICTION_LSB_OFFSET]);
        mVoc = (float)(((uint16_t)mBuffer[IAQCORE_TVOC_PREDICTION_MSB_OFFSET] << 8) | mBuffer[IAQCORE_TVOC_PREDICTION_LSB_OFFSET]);
        return true;
    }
    return false;
}
#endif
