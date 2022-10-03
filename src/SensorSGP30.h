#pragma once
// #include "IncludeManager.h"
#ifdef SENSORMODULE

#include "Sensor.h"
#include "EepromManager.h"

#define SGP30_I2C_ADDR (0x58)

class SensorSGP30 : public Sensor
{

protected:
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    float measureValue(MeasureType iMeasureType) override;
    void sensorSaveState() override;
    void sensorLoopInternal() override;
    bool checkIaqSensorStatus(void);
    void sensorLoadState();
    void sensorUpdateState();
    uint32_t stateUpdateTimer = 0;

  public:
    SensorSGP30(uint16_t iMeasureTypes);
    SensorSGP30(uint16_t iMeasureTypes, uint8_t iAddress);
    SensorSGP30(uint16_t iMeasureTypes, uint8_t iAddress, uint8_t iMagicKeyOffset);
    virtual ~SensorSGP30() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
    void setMagicKeyOffset(uint8_t iMagicKeyOffset);

  private:
    static uint8_t sMagicWord[];
    static uint8_t bsec_config_iaq[454];
    EepromManager *mEEPROM;
    uint8_t mLastAccuracy = 0;
};
#endif
