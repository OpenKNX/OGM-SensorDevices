#pragma once
// #include "IncludeManager.h"
// #ifdef SENSORMODULE

#include "Sensor.h"
#include "bsec2/bsec2.h"
// #include "EepromManager.h"

#define BME680_I2C_ADDR (0x76)
#define BME680_CALIBRATION_DATA_SIZE BSEC_MAX_PROPERTY_BLOB_SIZE
#define BME680_SAVE_SIZE (BSEC_MAX_STATE_BLOB_SIZE + 6)

class SensorBME680 : public Sensor, protected Bsec2
{
  protected:
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    float measureValue(MeasureType iMeasureType) override;
    // void sensorSaveState() override;
    void sensorLoopInternal() override;
    bool checkIaqSensorStatus(void);
    // ugly, but works
    static void bme680DataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);

    bool reinitialize();
    void sensorLoadState();
    // void sensorUpdateState();
    uint32_t stateUpdateTimer = 0;
    static bsec_virtual_sensor_t sensorList[];
    bme68x_delay_us_fptr_t mDelayCallback = 0;
    bool mDelayCallbackIsActive = false;

    const uint8_t* mFlashBuffer = nullptr; // Pointer to stored flash content
    bool mWorkBufferInitialized = false;
    uint8_t mWorkBuffer[BME680_SAVE_SIZE - 5]; // working buffer for reading/writing BME calibration data

    // new flash handling
    void sensorReadFlash(const uint8_t* iBuffer, const uint16_t iSize) override;
    void sensorSavePower() override;
    void sensorWriteFlash() override;
    uint16_t sensorFlashSize() override;
    static float temperature;
    static float humidity;
    static float pressure;
    static float iaq;
    static float co2Equivalent;
    static float iaqAccuracy;

  public:
    SensorBME680(uint16_t iMeasureTypes, TwoWire* iWire);
    SensorBME680(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress, bme68x_delay_us_fptr_t iDelayCallback);
    SensorBME680(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress, bme68x_delay_us_fptr_t iDelayCallback, uint8_t iMagicKeyOffset);
    virtual ~SensorBME680() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
    void delayCallback(bme68x_delay_us_fptr_t iDelayCallback);
    void setMagicKeyOffset(uint8_t iMagicKeyOffset);
    bool prepareTemperatureOffset(float iTemp) override;
    virtual std::string logPrefix() override;
    void delayCallbackActive(bool iOn);
    // just as interface between Bsec2 and Bsec
  private:
    static uint8_t sMagicWord[];
    static uint8_t bsec_config_iaq[BME680_CALIBRATION_DATA_SIZE];
    // EepromManager *mEEPROM;
    uint8_t mLastAccuracy = 0;   
};
// #endif
