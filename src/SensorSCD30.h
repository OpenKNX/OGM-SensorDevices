#pragma once
// #include "IncludeManager.h"
#ifdef SENSOR_SCD30_SUPPORT
    #include "Sensor.h"
    #include <SparkFun_SCD30_Arduino_Library.h>

    #define SCD30_I2C_ADDR 0x61

class SensorSCD30 : public Sensor, protected SCD30
{

  protected:
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorSCD30(uint16_t iMeasureTypes, TwoWire &iWire);
    SensorSCD30(uint16_t iMeasureTypes, TwoWire &iWire, uint8_t iAddress);
    virtual ~SensorSCD30() {}

    bool begin() override;
    void sensorLoopInternal() override;
    bool prepareTemperatureOffset(float iTemp) override;
    std::string logPrefix() override;
};
#endif
