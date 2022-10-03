// #include "IncludeManager.h"
#if defined(SENSORMODULE) || defined(PMMODULE)
#include <Wire.h>
#include "HardwareDevices.h"
#include "SensorVEML7700.h"

SensorVEML7700::SensorVEML7700(uint16_t iMeasureTypes)
    : Sensor(iMeasureTypes, VEML7700_I2C_ADDR){};

SensorVEML7700::SensorVEML7700(uint16_t iMeasureTypes, uint8_t iAddress)
    : Sensor(iMeasureTypes, iAddress){};

uint8_t SensorVEML7700::getSensorClass()
{
    return SENS_OPT300X;
}

void SensorVEML7700::sensorLoopInternal()
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
                if (getSensorData())
                    gSensorState = Running;
                pSensorStateDelay = millis();
            }
            break;
        case Running:
            if (delayCheck(pSensorStateDelay, 1000))
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

// bool SensorVEML7700::checkSensorConnection()
// {
//     // just valid for i2c sensors, in other cases this should be overridden
//     bool lResult = false;
//     // if (gSensorState == Running) {
//     // check for I2C ack
//     Wire1.beginTransmission(gAddress);
//     lResult = (Wire1.endTransmission() == 0);
//     if (!lResult)
//         restartSensor();
//     // }
//     return lResult;
// }

float SensorVEML7700::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
    case Lux:
        // hardware calibration
        return mLux;
        break;
    default:
        break;
    }
    return NO_NUM;
}

bool SensorVEML7700::begin()
{
    printDebug("Starting sensor VEML7700... ");
// #ifdef SENSOR_I2C_VEML7700
//     gWire = SENSOR_I2C_VEML7700;
//     gWire.begin();
// #endif
    bool lResult = Sensor::begin();
    if (lResult) {
        gWire.setClock(400000);
        lResult = mVeml.begin(&gWire);
    }
    printResult(lResult);
    return lResult;
}

uint8_t SensorVEML7700::getI2cSpeed()
{
    return 4; // n * 100kHz
}

bool SensorVEML7700::getSensorData()
{
    mLux = mVeml.readLux(VEML_LUX_NORMAL_NOWAIT);
    return true;
}

#endif
