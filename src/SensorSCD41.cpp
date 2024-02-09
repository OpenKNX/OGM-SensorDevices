// #include "IncludeManager.h"
#ifdef SENSORMODULE
#include <Wire.h>
#include "SensorSCD41.h"

SensorSCD41::SensorSCD41(uint16_t iMeasureTypes)
    : SensorSCD40(iMeasureTypes, SCD40_I2C_ADDR){};

SensorSCD41::SensorSCD41(uint16_t iMeasureTypes, uint8_t iAddress)
    : SensorSCD40(iMeasureTypes, iAddress){};

uint8_t SensorSCD41::getSensorClass()
{
    return SENS_SCD41;
}

void SensorSCD41::setMeasureInterval(uint32_t iMeasureInterval)
{
    mMeasureInterval = iMeasureInterval;
}

void SensorSCD41::sensorLoopInternal()
{
    switch (gSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
        case Calibrate:
            // check for standard intervals
            if (mMeasureInterval <= 5)
                startPeriodicMeasurement();
            else if (mMeasureInterval == 30)
                startLowPowerPeriodicMeasurement();
            else
            {
                // set ASC initial period according to data sheet:
                // 44 h for 5 min. intervals, scaled inversely proportional for other intervals
                uint16_t lAscInitial = 44 * (300 / mMeasureInterval);

                // it has to be dividable by 4
                lAscInitial = (lAscInitial + 3) & ~(decltype(lAscInitial))3;

                setAutomaticSelfCalibrationInitialPeriod(lAscInitial);

                // set ASC standard period according to data sheet:
                // 156 h for 5 min. intervals, scaled inversely proportional for other intervals
                uint16_t lAscStandard = 156 * (300 / mMeasureInterval);

                // it has to be dividable by 4
                lAscStandard = (lAscStandard + 3) & ~(decltype(lAscStandard))3;

                setAutomaticSelfCalibrationStandardPeriod(lAscStandard);
            }
            
            Sensor::sensorLoopInternal();
            break;
        case Finalize:
            // if standard intervals used, we wait for first data
            if (mMeasureInterval <= 5 || mMeasureInterval == 30)
            {
                // we ask for value until we get a valid value
                if (delayCheck(pSensorStateDelay, 2000))
                {
                    if (getSensorData())
                        gSensorState = Running;
                    pSensorStateDelay = millis();
                }
            }
            else
                gSensorState = Running;

            break;
        case Running:
            if (mMeasureInterval <= 5 || mMeasureInterval == 30)
            {
                // if standard intervals used, we just need to get data at defined intervals
                if (delayCheck(pSensorStateDelay, mMeasureInterval * 1000))
                {
                    getSensorData();
                    pSensorStateDelay = millis();
                }
            }
            else
            {
                // for other intervals, measurement is triggered manually
                if (mIsMeasuring)
                {
                    if (delayCheck(mMeasureDelay, MEASURE_DELAY))
                    {
                        getSensorData();
                        pSensorStateDelay = millis();
                        mIsMeasuring = false;
                    }
                }
                else
                {
                    if (delayCheck(pSensorStateDelay, mMeasureInterval * 1000))
                    {
                        measureSingleShot();
                        pSensorStateDelay = millis();
                        mIsMeasuring = true;
                    }
                }
            }

            break;
        default:
            pSensorStateDelay = millis();
            break;
    }
}

bool SensorSCD41::begin()
{
    logDebugP("Starting sensor SCD41... ");
    return beginInternal();
}
#endif