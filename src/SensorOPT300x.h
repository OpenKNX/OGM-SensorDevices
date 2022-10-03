#pragma once
// #include "IncludeManager.h"
#if defined(SENSORMODULE) || defined(PMMODULE)
#include "Sensor.h"

#define OPT300X_I2C_ADDR 0x45

#define OPT300X_REG_RESULT 0x00
#define OPT300X_REG_CONFIG 0x01
#define OPT300X_REG_LOW_LIMIT 0x02
#define OPT300X_REG_HIGH_LIMIT 0x03
#define OPT300X_REG_MANUFACTURER_ID 0x7E
#define OPT300X_REG_DEVICE_ID 0x7F

#define OPT300X_CONF_AUTO_FULL_RANGE 0b1100

#define OPT300X_CONF_CONV_TIME_800 0b1
#define OPT300X_CONF_CONV_TIME_100 0b0

#define OPT300X_CONF_CONV_OPERATION_CONTINIOUS 0b11
#define OPT300X_CONF_CONV_OPERATION_ONE 0b01
#define OPT300X_CONF_CONV_OPERATION_OFF 0b00

#define OPT300X_CONF_LATCH_TRANSPARENT 0b0
#define OPT300X_CONF_LATCH_WINDOW 0b1

typedef union
{
    struct
    {
        uint8_t faultCount : 2;
        uint8_t maskExponent : 1;
        uint8_t polarity : 1;
        uint8_t latch : 1;
        uint8_t flagLow : 1;
        uint8_t flagHigh : 1;
        uint8_t conversionReady : 1;
        uint8_t overflowFlag : 1;
        uint8_t modeOfConversionOperation : 2;
        uint8_t conversionTime : 1;
        uint8_t rangeNumber : 4;
    };
    uint16_t rawData;
} OPT300xConfig;

class SensorOPT300x : public Sensor
{
  private:
    uint8_t mBuffer[2];
    bool getSensorData();
    bool writeConfig(OPT300xConfig iConfig);


  protected:
    float mLux = NO_NUM;
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorOPT300x(uint16_t iMeasureTypes);
    SensorOPT300x(uint16_t iMeasureTypes, uint8_t iAddress);
    virtual ~SensorOPT300x() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
};
#endif
