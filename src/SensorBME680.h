#pragma once
// #include "IncludeManager.h"
#ifdef SENSORMODULE

#include "bsec/bsec.h"
#include "Sensor.h"
#include "EepromManager.h"

#define BME680_I2C_ADDR (0x76)

class SensorBME680 : public Sensor, protected Bsec
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
    static bsec_virtual_sensor_t sensorList[];
    bme680_delay_fptr_t mDelayCallback = 0;

  public:
    SensorBME680(uint16_t iMeasureTypes);
    SensorBME680(uint16_t iMeasureTypes, uint8_t iAddress, bme680_delay_fptr_t iDelayCallback);
    SensorBME680(uint16_t iMeasureTypes, uint8_t iAddress, bme680_delay_fptr_t iDelayCallback, uint8_t iMagicKeyOffset);
    virtual ~SensorBME680() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
    void delayCallback(bme680_delay_fptr_t iDelayCallback);
    void setMagicKeyOffset(uint8_t iMagicKeyOffset);
    bool prepareTemperatureOffset(float iTemp) override;

  private:
    static uint8_t sMagicWord[];
    static uint8_t bsec_config_iaq[454];
    EepromManager *mEEPROM;
    uint8_t mLastAccuracy = 0;
};
#endif
