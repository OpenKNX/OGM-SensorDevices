#pragma once
// #include <knx/bits.h>
#include "OpenKNX.h"
#include <Wire.h>

#define SENSOR_COUNT 5

#define BIT_1WIRE 1
#define BIT_Temp 2
#define BIT_Hum 4
#define BIT_Pre 8
#define BIT_Voc 16
#define BIT_Co2 32
#define BIT_RESERVE 64
#define BIT_LOGIC 128
#define BIT_LUX 256
#define BIT_TOF 512
#define BIT_PRESENCE 1024

#define SENS_NO 0
#define SENS_SHT3X 1      // Temp/Hum
#define SENS_BME280 2     // Temp/Hum/Pressure
#define SENS_BME680 3     // Temp/Hum/Pressure/Voc
#define SENS_SCD30 4      // Temp/Hum/Co2
#define SENS_IAQCORE 5    // Voc
#define SENS_OPT300X 6    // Lux
#define SENS_VL53L1X 7    // Tof
#define SENS_SGP30 8      // Temp/Hum/Voc
#define SENS_SCD40 9      // Temp/Hum/Co2
#define SENS_SCD41 13     // Temp/Hum/Co2
#define SENS_MR24xxB1 10  // HF-Presence
#define SENS_VEML7700 11  // Lux
#define SENS_HLKLD2420 12 // HF-Presence
#define SENS_XEND107H 13  // HF-Presence

enum SensorState
{
    Off,
    Wakeup,
    Calibrate,
    Finalize,
    Running
};

enum MeasureType
{
    OneWireBM = 1,
    Temperature = 2,
    Humidity = 4,
    Pressure = 8,
    Voc = 16,
    Co2 = 32,
    Co2Calc = 64, // calculated Co2 from VOC
    Accuracy = 128,
    Lux = 256,
    Tof = 512,
    Pres = 1024,
    Speed = 2048,
    Sensitivity = 4096,
    Scenario = 8192,
    Distance = 16384
};

#if defined(SENSORMODULE) || defined(PMMODULE)
// #include "OpenKNX/Helper.h"
struct sSensorInfo
{
    float lastValue;
    float lastSentValue;
    uint32_t sendDelay;
    uint32_t readDelay;
};

struct sActorInfo
{
    uint8_t lastOutputValue;
    uint8_t lastInputValue;
    uint32_t sendDelay;
    uint32_t readDelay;
};

union uData
{
    sSensorInfo sensor;
    sActorInfo actor;
};

class Sensor
{
  protected:
    // Sensor();
    Sensor(uint16_t iMeasureTypes, TwoWire *iWire, uint8_t iAddress);
    virtual ~Sensor() {}

    TwoWire *pWire = &Wire;

    uint16_t pMeasureTypes;
    uint8_t pI2CAddress;
    SensorState pSensorState = Wakeup;
    uint32_t pSensorStateDelay = 0;
    float pTempOffset = 0.0;

    virtual bool checkSensorConnection();
    virtual float measureValue(MeasureType iMeasureType) = 0; // pure
    // virtual void sensorSaveState();
    // non blocking restart approach for a sensor
    virtual bool begin(); // first initialization, may be blocking, should be called during setup(), not during loop()
    void logResult(bool iResult);

  public:
    virtual uint8_t getI2cSpeed();
    virtual bool prepareTemperatureOffset(float iTempOffset);
    virtual void sensorReadFlash(const uint8_t *iBuffer, const uint16_t iSize);
    virtual void sensorWriteFlash();
    virtual uint16_t sensorFlashSize();
    virtual std::string logPrefix();
    virtual uint8_t getSensorClass() = 0; // pure; returns unique ID for this sensor type
    void addMeasureType(MeasureType iMeasureType);
    bool checkMeasureType(MeasureType iMeasureType);
    virtual void sensorLoopInternal();
    void restartSensor();
    bool measureValue(MeasureType iMeasureType, float &eValue);
};
#endif
