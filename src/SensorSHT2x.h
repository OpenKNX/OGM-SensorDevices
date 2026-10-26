#pragma once
#ifdef SENSORMODULE
    #include "Sensor.h"

    #define SHT2X_ADDR           0x40

    #define SHT2X_MEAS_T_NOHOLD  0xF3
    #define SHT2X_MEAS_RH_NOHOLD 0xF5
    #define SHT2X_SOFTRESET      0xFE

// non-blocking implementation of SHT2x Temp/Hum-Sensor (SHT20/SHT21/SHT25)
class SensorSHT2x : public Sensor
{
  protected:
    uint8_t crc8(const uint8_t* iData, uint8_t iLen);
    bool readRawValue(float& eValue, bool iIsTemp);
    void writeCommand(uint8_t iCmd);

    float mTemp = NO_NUM;
    float mHumidity = NO_NUM;
    uint32_t mCommandSent = 0;
    uint8_t mPhase = 0;

    uint8_t getSensorClass() override;
    void sensorLoopInternal() override;
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorSHT2x(uint16_t iMeasureTypes, TwoWire* iWire);
    virtual ~SensorSHT2x() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
    std::string logPrefix() override;
    bool prepareTemperatureOffset(float iTempOffset) override;
};
#endif
