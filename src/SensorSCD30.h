#pragma once
// #include "IncludeManager.h"
#ifdef SENSORMODULE
#include <SparkFun_SCD30_Arduino_Library.h>
#include "Sensor.h"

#define SCD30_I2C_ADDR 0x61

class SensorSCD30 : public Sensor, protected SCD30
{

protected:
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    float measureValue(MeasureType iMeasureType) override;

public:
    SensorSCD30(uint16_t iMeasureTypes);
    SensorSCD30(uint16_t iMeasureTypes, uint8_t iAddress);
    virtual ~SensorSCD30() {}

    bool begin() override; 
    void sensorLoopInternal() override;
    bool prepareTemperatureOffset(float iTemp) override;
};
#endif
