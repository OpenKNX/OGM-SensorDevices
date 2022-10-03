#pragma once
// #include "IncludeManager.h"
#if defined(SENSORMODULE) || defined(PMMODULE)
#include "Sensor.h"

// dummy sensor, returned for not implemented sensors

class SensorDummy : public Sensor
{
  protected:
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorDummy(uint16_t iMeasureTypes);
    SensorDummy(uint16_t iMeasureTypes, uint8_t iAddress);
    virtual ~SensorDummy() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
};
#endif
