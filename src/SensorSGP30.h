#pragma once
#ifdef SENSORMODULE
    #include "Sensor.h"
    #include <Adafruit_SGP30.h>

    #define SGP30_ADDR      0x58
    #define SGP30_SAVE_SIZE 9   // 1 version + 4 magic + 4 baseline bytes

class SensorSGP30 : public Sensor, protected Adafruit_SGP30
{
  protected:
    float mTVOC = NO_NUM;
    float mECO2 = NO_NUM;

    const uint8_t* mFlashBuffer = nullptr;
    bool mWorkBufferInitialized = false;
    uint8_t mWorkBuffer[4];

    uint32_t getAbsoluteHumidity(float iTemp, float iHum);
    void processHumidity();

    uint8_t getSensorClass() override;
    void sensorLoopInternal() override;
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorSGP30(uint16_t iMeasureTypes, TwoWire* iWire);
    virtual ~SensorSGP30() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
    std::string logPrefix() override;

    uint16_t sensorFlashSize() override;
    void sensorReadFlash(const uint8_t* iBuffer, const uint16_t iSize) override;
    void sensorWriteFlash() override;
    void sensorSavePower() override;
};
#endif
