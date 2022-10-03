#pragma once
// #include "IncludeManager.h"
#ifdef SENSORMODULE
#include "Sensor.h"
#include <VL53L1X.h>

#define VL53L1X_I2C_ADDR 0x29

class SensorVL53L1X : public Sensor, protected VL53L1X
{
  private:
    uint16_t mDistance;
    bool getSensorData();

  protected:
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorVL53L1X(uint16_t iMeasureTypes);
    SensorVL53L1X(uint16_t iMeasureTypes, uint8_t iAddress);
    virtual ~SensorVL53L1X() {}
    
    bool begin() override;
    uint8_t getI2cSpeed() override;
};
#endif
