#pragma once
// #include "IncludeManager.h"
#ifdef SENSORMODULE
#include "Sensor.h"

#define SHT3X_ADDR 0x44

#define SHT3X_MEAS_HIGHREP 0x2400
#define SHT3X_MEAS_MEDREP 0x240B
#define SHT3X_MEAS_LOWREP 0x2416
#define SHT3X_READSTATUS 0xF32D
#define SHT3X_CLEARSTATUS 0x3041
#define SHT3X_SOFTRESET 0x30A2
#define SHT3X_HEATER_ENABLE 0x306D
#define SHT3X_HEATER_DISABLE 0x3066

// non-blocking implementation of SHT3x Temp/Hum-Sensor
class SensorSHT3x : public Sensor
{
  protected:
    float readTemperature();
    float readHumidity();
    void reset();
    void heater(bool iOn);
    uint8_t crc8(const uint8_t *iData, uint8_t iLen);
    bool getTempHum();
    void writeCommand(uint16_t iCmd);

    float mTemp = NO_NUM;
    float mHumidity = NO_NUM;

    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorSHT3x(uint16_t iMeasureTypes);
    SensorSHT3x(uint16_t iMeasureTypes, uint8_t iAddress);
    virtual ~SensorSHT3x() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
};
#endif
