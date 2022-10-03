// #include "IncludeManager.h"
#if defined(SENSORMODULE) || defined(PMMODULE)
#include <Wire.h>
#include "SensorDummy.h"

SensorDummy::SensorDummy(uint16_t iMeasureTypes)
    : Sensor(iMeasureTypes, 0){};

SensorDummy::SensorDummy(uint16_t iMeasureTypes, uint8_t iAddress)
    : Sensor(iMeasureTypes, iAddress){};

uint8_t SensorDummy::getSensorClass()
{
    return SENS_NO;
}

float SensorDummy::measureValue(MeasureType iMeasureType)
{
    return -9.8765;
}

bool SensorDummy::begin()
{
    printDebug("Starting dummy sensor... ");
    printResult(true);
    return true;
}

uint8_t SensorDummy::getI2cSpeed()
{
    return 4; // n * 100kHz
}
#endif
