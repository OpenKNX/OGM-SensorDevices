#pragma once
// #include <knx/bits.h>
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
#define SENS_SHT3X 1     // Temp/Hum
#define SENS_BME280 2    // Temp/Hum/Pressure
#define SENS_BME680 3    // Temp/Hum/Pressure/Voc
#define SENS_SCD30 4     // Temp/Hum/Co2
#define SENS_IAQCORE 5   // Voc
#define SENS_OPT300X 6   // Lux
#define SENS_VL53L1X 7   // Tof
#define SENS_SGP30 8     // Temp/Hum/Voc
#define SENS_SCD41 9     // Temp/Hum/Co2
#define SENS_MR24xxB1 10 // HF-Presence
#define SENS_VEML7700 11 // Lux

enum SensorState
{
    Off,
    Wakeup,
    Calibrate,
    Finalize,
    Running
};

enum MeasureType {
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
    Scenario = 8192
};

#if defined(SENSORMODULE) || defined(PMMODULE)
#include "Helper.h"

class Sensor
{
  private:
    static Sensor* sSensors[SENSOR_COUNT];
    static uint8_t sNumSensors;
    static uint8_t sMaxI2cSpeed;

  protected:
    // Sensor();
    Sensor(uint16_t iMeasureTypes, uint8_t iAddress);
    virtual ~Sensor() {}
    uint16_t gMeasureTypes;

    uint8_t gAddress;
    static TwoWire &sWire;
    TwoWire &gWire = Wire;
    SensorState gSensorState = Wakeup;
    uint32_t pSensorStateDelay = 0;
    float gTempOffset = 0.0;

    virtual uint8_t getSensorClass() = 0; // pure; returns unique ID for this sensor type
    virtual bool checkSensorConnection();
    virtual float measureValue(MeasureType iMeasureType) = 0; //pure
    virtual void sensorLoopInternal();
    virtual void sensorSaveState();
    // non blocking restart approach for a sensor
    void restartSensor();
    virtual bool begin(); // first initialization, may be blocking, should be called druing setup(), not during loop()

  public:
    // static 
    static void SetWire(TwoWire &iWire);
    static Sensor* factory(uint8_t iSensorClass, MeasureType iMeasureType);
    static void sensorLoop();
    static bool measureValue(MeasureType iMeasureType, float& eValue);
    static uint8_t getError();
    static void saveState();
    static void restartSensors();
    static bool beginSensors();
    static uint8_t getMaxI2cSpeed();
    
    virtual uint8_t getI2cSpeed();
    virtual bool prepareTemperatureOffset(float iTempOffset);
};
#endif
