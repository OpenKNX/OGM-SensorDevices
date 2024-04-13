// #include "IncludeManager.h"
#ifdef SENSORMODULE
    #include "SensorSCD40.h"
    #include <Wire.h>

SensorSCD40::SensorSCD40(uint16_t iMeasureTypes, TwoWire* iWire)
    : Sensor(iMeasureTypes, iWire, SCD40_I2C_ADDR), SensirionI2CScd4x(){};

SensorSCD40::SensorSCD40(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress)
    : Sensor(iMeasureTypes, iWire, iAddress), SensirionI2CScd4x(){};

uint8_t SensorSCD40::getSensorClass()
{
    return SENS_SCD40;
}

std::string SensorSCD40::logPrefix()
{
    return "Sensor<SCD40>";
}

void SensorSCD40::sensorLoopInternal()
{
    switch (pSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
        case Calibrate:

            startLowPowerPeriodicMeasurement();
            Sensor::sensorLoopInternal();
            break;
        case Finalize:
            // we ask for value until we get a valid value
            if (delayCheck(pSensorStateDelay, 2000))
            {
                if (getSensorData())
                    pSensorState = Running;
                pSensorStateDelay = millis();
            }
            break;
        case Running:
            if (delayCheck(pSensorStateDelay, 35000))
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

float SensorSCD40::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
        case Temperature:
            return (float)mTemp;
            break;
        case Humidity:
            return (float)mHum;
            break;
        case Co2:
            return (float)mCo2;
            break;
        default:
            break;
    }
    return NO_NUM;
}

bool SensorSCD40::begin()
{
    logDebugP("Starting sensor SCD40... ");
    return beginInternal();
}

bool SensorSCD40::beginInternal()
{
    SensirionI2CScd4x::begin(*pWire);
    bool lResult = false;
    lResult = (stopPeriodicMeasurement() == 0);
    if (lResult)
        lResult = (setTemperatureOffset(-pTempOffset) == 0);
    if (lResult)
        lResult = Sensor::begin();
    logResult(lResult);
    return lResult;
}

uint8_t SensorSCD40::getI2cSpeed()
{
    return 1; // n * 100kHz
}

bool SensorSCD40::getSensorData()
{
    bool lDataReady;
    bool lResult = (SensirionI2CScd4x::getDataReadyFlag(lDataReady) == 0);

    if (lResult)
    {
        if (lDataReady)
        {
            uint16_t lTemp;
            uint16_t lHum;
            uint16_t lCo2;
            lResult = (SensirionI2CScd4x::readMeasurementTicks(lCo2, lTemp, lHum) == 0);
            lResult = (lCo2 > 0);
            if (lResult)
            {
                mTemp = lTemp * 175.0 / 65536.0 - 45.0;
                mHum = lHum * 100.0 / 65536.0;
                mCo2 = lCo2;
            }
        }
    }
    return lResult;
}

bool SensorSCD40::prepareTemperatureOffset(float iTempOffset)
{
    pTempOffset = -4.0 + iTempOffset;
    return true;
}

bool SensorSCD40::setPressure(float pressure)
{
    return setAmbientPressure(static_cast<uint16_t>(pressure * 100.0 + 0.5));
}
#endif
