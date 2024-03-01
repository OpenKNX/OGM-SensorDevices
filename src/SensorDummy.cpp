// #include "IncludeManager.h"
#if defined(SENSORMODULE) || defined(PMMODULE)
    #include "SensorDummy.h"
    #include <Wire.h>

SensorDummy::SensorDummy(uint16_t iMeasureTypes, TwoWire &iWire)
    : Sensor(iMeasureTypes, iWire, 0){};

SensorDummy::SensorDummy(uint16_t iMeasureTypes, TwoWire &iWire, uint8_t iAddress)
    : Sensor(iMeasureTypes, iWire, iAddress){};

uint8_t SensorDummy::getSensorClass()
{
    return SENS_NO;
}

std::string SensorDummy::logPrefix()
{
    return "Sensor<Dummy>";
}

float SensorDummy::measureValue(MeasureType iMeasureType)
{
    return -9.8765;
}

bool SensorDummy::begin()
{
    logDebugP("Starting dummy sensor... ");
    logResult(true);
    return true;
}

uint8_t SensorDummy::getI2cSpeed()
{
    return 4; // n * 100kHz
}
#endif
