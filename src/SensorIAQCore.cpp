// #include "IncludeManager.h"
#ifdef SENSORMODULE
#include <Wire.h>
#include "SensorIAQCore.h"

SensorIAQCore::SensorIAQCore(uint16_t iMeasureTypes)
    : Sensor(iMeasureTypes, IAQCORE_I2C_ADDR){};

SensorIAQCore::SensorIAQCore(uint16_t iMeasureTypes, uint8_t iAddress)
    : Sensor(iMeasureTypes, iAddress){};

uint8_t SensorIAQCore::getSensorClass()
{
    return SENS_IAQCORE;
}

void SensorIAQCore::sensorLoopInternal()
{
    switch (gSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
        case Calibrate:
            Sensor::sensorLoopInternal();
            break;
        case Finalize:
            if (delayCheck(pSensorStateDelay, 1000)) {
                // start first measurement
                if (getSensorData()) 
                    gSensorState = Running;
                pSensorStateDelay = millis();
            }
            break;
        case Running:
            if (delayCheck(pSensorStateDelay, 2000)) {
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
    printDebug("Starting sensor IAQCore... ");
    lResult = Sensor::begin();
    printResult(lResult);
    return lResult;
}

bool SensorIAQCore::getSensorData()
{
    // clear read buffer
    memset(mBuffer, 0, sizeof(mBuffer));
    // request sensor data

    gWire.requestFrom(gAddress, IAQCORE_READ_ALL);
    if (gWire.available() != IAQCORE_READ_ALL)
        return false;
    for (uint8_t i = 0; i < IAQCORE_READ_ALL; i++)
        mBuffer[i] = gWire.read();
    uint8_t lStatus = mBuffer[IAQCORE_STATUS_OFFSET];
    if ( lStatus & IAQCORE_STATUS_ERROR)
        return false;
    if ((lStatus & IAQCORE_STATUS_BUSY) == 0) {
        mCo2 = (float)(((uint16_t)mBuffer[IAQCORE_CO2_PREDICTION_MSB_OFFSET] << 8) | mBuffer[IAQCORE_CO2_PREDICTION_LSB_OFFSET]);
        mVoc = (float)(((uint16_t)mBuffer[IAQCORE_TVOC_PREDICTION_MSB_OFFSET] << 8) | mBuffer[IAQCORE_TVOC_PREDICTION_LSB_OFFSET]);
        return true;
    }
    return false;
}
#endif
