#pragma once
#ifdef SENSORMODULE
    #include "Sensor.h"

    #define SHT4X_ADDR         0x44
    #define SHT4X_ADDR_ALT     0x45
    #define SHT4X_MEAS_HIGHREP 0xFD
    #define SHT4X_MEAS_MEDREP  0xF6
    #define SHT4X_MEAS_LOWREP  0xE0
    #define SHT4X_SOFTRESET    0x94

// non-blocking implementation of SHT4x Temp/Hum-Sensor
class SensorSHT4x : public Sensor
{
  protected:
    uint8_t crc8(const uint8_t* iData, uint8_t iLen);
    bool getTempHum();
    void writeCommand(uint8_t iCmd);

    float mTemp = NO_NUM;
    float mHumidity = NO_NUM;

    uint8_t getSensorClass() override;
    void sensorLoopInternal() override;
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorSHT4x(uint16_t iMeasureTypes, TwoWire* iWire);
    SensorSHT4x(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress);
    virtual ~SensorSHT4x() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
    std::string logPrefix() override;
    bool prepareTemperatureOffset(float iTempOffset) override;
};
#endif
