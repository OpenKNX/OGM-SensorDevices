// #include "IncludeManager.h"
#ifdef SENSOR_SCD30_SUPPORT
    #include "SensorSCD30.h"

SensorSCD30::SensorSCD30(uint16_t iMeasureTypes, TwoWire* iWire)
    : Sensor(iMeasureTypes, iWire, SCD30_I2C_ADDR), SCD30(){};

SensorSCD30::SensorSCD30(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress)
    : Sensor(iMeasureTypes, iWire, iAddress), SCD30(){};

uint8_t SensorSCD30::getSensorClass()
{
    return SENS_SCD30;
}

std::string SensorSCD30::logPrefix()
{
    return "Sensor<SCD30>";
}

float SensorSCD30::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
        case Temperature:
            // hardware calibration
            return getTemperature();
            break;
        case Humidity:
            return getHumidity();
            break;
        case Co2:
            return getCO2();
            break;
        default:
            break;
    }
    return NO_NUM;
}

bool SensorSCD30::begin()
{
    logDebugP("Starting sensor SCD30... ");
    bool lResult = SCD30::begin();
    if (lResult)
    {
        lResult = Sensor::begin();
        setTemperatureOffset(-gTempOffset);
    }
    logResult(lResult);
    return lResult;
}

void SensorSCD30::sensorLoopInternal()
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
            // we ask for Temperature until we get a valid value
            if (delayCheck(pSensorStateDelay, 200))
            {
                if (getTemperature() > 0) gSensorState = Running;
                pSensorStateDelay = millis();
            }
            break;
        default:
            Sensor::sensorLoopInternal();
            break;
    }
}

bool SensorSCD30::prepareTemperatureOffset(float iTemp)
{
    gTempOffset = iTemp;
    return true;
}
#endif
