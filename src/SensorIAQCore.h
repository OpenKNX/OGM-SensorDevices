// #include "IncludeManager.h"
#pragma once
#ifdef SENSORMODULE
#include "Sensor.h"

#define IAQCORE_I2C_ADDR (0x5A)

// Bytes to read
#define IAQCORE_READ_ALL (9)

// Status codes
#define IAQCORE_STATUS_OK (0x00)
#define IAQCORE_STATUS_RUNIN (0x10)
#define IAQCORE_STATUS_BUSY (0x01)
#define IAQCORE_STATUS_ERROR (0x80)
// Memory addressing
#define IAQCORE_CO2_PREDICTION_MSB_OFFSET (0x00)
#define IAQCORE_CO2_PREDICTION_LSB_OFFSET (0x01)
#define IAQCORE_STATUS_OFFSET (0x02)
#define IAQCORE_TVOC_PREDICTION_MSB_OFFSET (0x07)
#define IAQCORE_TVOC_PREDICTION_LSB_OFFSET (0x08)

class SensorIAQCore : public Sensor
{
  private:
    uint8_t mBuffer[IAQCORE_READ_ALL];
    bool getSensorData();

  protected:
    float mCo2 = NO_NUM;
    float mVoc = NO_NUM;

    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorIAQCore(uint16_t iMeasureTypes);
    SensorIAQCore(uint16_t iMeasureTypes, uint8_t iAddress);
    virtual ~SensorIAQCore() {}

    bool begin() override;
};
#endif
