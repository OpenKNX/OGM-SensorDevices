#pragma once
// #include "IncludeManager.h"
// #ifdef SENSORMODULE

#include "Sensor.h"
#include "bsec/bsec.h"
// #include "EepromManager.h"

#define BME680_I2C_ADDR (0x76)
#define BME680_CALIBRATION_DATA_SIZE 454
#define BME680_SAVE_SIZE 145

class SensorBME680 : public Sensor, protected Bsec
{

  protected:
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    float measureValue(MeasureType iMeasureType) override;
    // void sensorSaveState() override;
    void sensorLoopInternal() override;
    bool checkIaqSensorStatus(void);
    void sensorLoadState();
    // void sensorUpdateState();
    uint32_t stateUpdateTimer = 0;
    static bsec_virtual_sensor_t sensorList[];
    bme680_delay_fptr_t mDelayCallback = 0;
    bool mDelayCallbackIsActive = false;

    const uint8_t* mFlashBuffer = nullptr; // Pointer to stored flash content
    // new flash handling
    void sensorReadFlash(const uint8_t* iBuffer, const uint16_t iSize) override;
    void sensorWriteFlash() override;
    uint16_t sensorFlashSize() override;

  public:
    SensorBME680(uint16_t iMeasureTypes, TwoWire* iWire);
    SensorBME680(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress, bme680_delay_fptr_t iDelayCallback);
    SensorBME680(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress, bme680_delay_fptr_t iDelayCallback, uint8_t iMagicKeyOffset);
    virtual ~SensorBME680() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
    void delayCallback(bme680_delay_fptr_t iDelayCallback);
    void setMagicKeyOffset(uint8_t iMagicKeyOffset);
    bool prepareTemperatureOffset(float iTemp) override;
    virtual std::string logPrefix() override;
    void delayCallbackActive(bool iOn);

  private:
    static uint8_t sMagicWord[];
    static uint8_t bsec_config_iaq[BME680_CALIBRATION_DATA_SIZE];
    // EepromManager *mEEPROM;
    uint8_t mLastAccuracy = 0;
};
// #endif
