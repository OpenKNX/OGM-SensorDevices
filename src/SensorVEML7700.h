#pragma once
// #include "IncludeManager.h"
#if defined(SENSORMODULE) || defined(PMMODULE)
#include "Sensor.h"
#include <Adafruit_VEML7700.h>

#define VEML7700_I2C_ADDR 0x10

class SensorVEML7700 : public Sensor
{
  private:
    Adafruit_VEML7700 mVeml = Adafruit_VEML7700();
    bool getSensorData();

  protected:
    float mLux = NO_NUM;
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    float measureValue(MeasureType iMeasureType) override;
    // bool checkSensorConnection() override;

  public:
    SensorVEML7700(uint16_t iMeasureTypes);
    SensorVEML7700(uint16_t iMeasureTypes, uint8_t iAddress);
    virtual ~SensorVEML7700() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
};
#endif
