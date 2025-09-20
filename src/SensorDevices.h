#include "OpenKNX.h"
#include "Sensor.h"

#ifndef MAX_SUPPORTED_SENSORS
    #define MAX_SUPPORTED_SENSORS 10
#endif

class SensorDevices : public OpenKNX::Module
{
  private:
    Sensor *mSensors[MAX_SUPPORTED_SENSORS];
    uint8_t mNumSensors = 0;
    uint8_t mMaxI2cSpeed = 255;
    uint8_t mCurrentSensorIterator = 0;
    uint8_t mMeasureValueIterator = 0;
    TwoWire *mWire = &Wire;
    uint32_t mSensorTestDelayTimer = 0;
    uint32_t mSensorTestDuration = 0;

    // forward declaration
    // this allows to avoid include all sensors as in sensorFactory
    Sensor *newSensor(uint8_t iSensorClass, MeasureType iMeasureType, TwoWire *iWire);
    void showHelp() override;
    bool processCommand(const std::string iCmd, bool iDiagnoseKo) override;
    void testSensorMeasurement();
    
  public:
    SensorDevices(/* args */);
    ~SensorDevices();

    void setup(bool iConfigured) override;
    void loop(bool iConfigured) override;
    const std::string name() override;
    const std::string version() override;
    uint16_t flashSize() override;
    void savePower() override;
    bool restorePower() override;
    void writeFlash() override;
    void readFlash(const uint8_t *iBuffer, const uint16_t iSize) override;

    Sensor *factory(uint8_t iSensorClass, MeasureType iMeasureType);
    bool measureValue(MeasureType iMeasureType, float &eValue);
    uint8_t getError();
    void restartSensors();
    bool beginSensors();
    uint8_t getMaxI2cSpeed();
    void defaultWire(TwoWire &iWire);

    void testSensors();
};

extern SensorDevices openknxSensorDevicesModule;