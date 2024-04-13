#pragma once
// #include "IncludeManager.h"
#ifdef SENSORMODULE
    #include "Sensor.h"
    #include <Adafruit_BME280.h>

    #define BME280_I2C_ADDR (0x76)

class SensorBME280 : public Sensor, protected Adafruit_BME280
{

  protected:
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    float measureValue(MeasureType iMeasureType) override;
    void sensorLoopInternal() override;
    bool initWakeup();
    bool initFinalize();

  public:
    SensorBME280(uint16_t iMeasureTypes, TwoWire* iWire);
    SensorBME280(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress);
    virtual ~SensorBME280() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
    bool prepareTemperatureOffset(float iTemp) override;
    std::string logPrefix() override;
};
#endif
