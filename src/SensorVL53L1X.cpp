// #include "IncludeManager.h"
#ifdef SENSORMODULE
#include <Wire.h>
#include "SensorVL53L1X.h"

SensorVL53L1X::SensorVL53L1X(uint16_t iMeasureTypes)
    : Sensor(iMeasureTypes, VL53L1X_I2C_ADDR), VL53L1X(){};

SensorVL53L1X::SensorVL53L1X(uint16_t iMeasureTypes, uint8_t iAddress)
    : Sensor(iMeasureTypes, iAddress), VL53L1X(){};

uint8_t SensorVL53L1X::getSensorClass()
{
    return SENS_VL53L1X;
}

void SensorVL53L1X::sensorLoopInternal()
{
    switch (gSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
        case Calibrate:
            Sensor::sensorLoopInternal();
            // start initial sensor reading
            this->readSingle(false);
            break;
        case Finalize:
            // we ask for distance until we get a valid value
            if (delayCheck(pSensorStateDelay, 2000))
            {
                if (getSensorData())
                    gSensorState = Running;
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

float SensorVL53L1X::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
    case Tof:
        return (float)mDistance;
        // return ((float)this->ranging_data.range_mm);
        break;
    default:
        break;
    }
    return NO_NUM;
}

bool SensorVL53L1X::begin()
{
    printDebug("Starting sensor VL53L1X... ");
    this->setTimeout(500);
    bool lResult = this->init();
    if (lResult) {
        // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
        // You can change these settings to adjust the performance of the sensor, but
        // the minimum timing budget is 20 ms for short distance mode and 33 ms for
        // medium and long distance modes. See the VL53L1X datasheet for more
        // information on range and timing limits.
        this->setDistanceMode(VL53L1X::Long);
        this->setMeasurementTimingBudget(100000);

        // Start continuous readings at a rate of one measurement every 50 ms (the
        // inter-measurement period). This period should be at least as long as the
        // timing budget.
        // this->startContinuous(50);
        lResult = Sensor::begin();
    }
    printResult(lResult);
    return lResult;
}

uint8_t SensorVL53L1X::getI2cSpeed()
{
    return 4; // n * 100kHz
}

bool SensorVL53L1X::getSensorData()
{
    bool lResult = this->dataReady();
    if (lResult) {
        mDistance = this->read(false);
        // do the next reading
        this->readSingle(false);
    }
    return lResult;
}

#endif
