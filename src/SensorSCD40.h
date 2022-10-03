// #include "IncludeManager.h"
#pragma once
#ifdef SENSORMODULE
#include "Sensor.h"
#include <SensirionI2CScd4x.h>

#define SCD40_I2C_ADDR 0x62

class SensorSCD40 : public Sensor, protected SensirionI2CScd4x
{
  private:
    float mTemp = NO_NUM;
    float mHum = NO_NUM;
    float mCo2 = NO_NUM;
    bool getSensorData();

  protected:
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorSCD40(uint16_t iMeasureTypes);
    SensorSCD40(uint16_t iMeasureTypes, uint8_t iAddress);
    virtual ~SensorSCD40() {}
    
    bool begin() override;
    uint8_t getI2cSpeed() override;
    bool prepareTemperatureOffset(float iTempOffset) override;
};
#endif
