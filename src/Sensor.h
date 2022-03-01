#pragma once
// #include <knx/bits.h>

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

#define SENS_NO 0
#define SENS_SHT3X 1
#define SENS_BME280 2
#define SENS_BME680 3
#define SENS_SCD30 4
#define SENS_IAQCORE 5
#define SENS_OPT300X 6
#define SENS_VL53L1X 7
#define SENS_SGP30 8
#define SENS_SCD41 9

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
    Tof = 512
};

#ifdef SENSORMODULE
#include "Helper.h"

class Sensor
{
  private:
    static Sensor* sSensors[SENSOR_COUNT];
    static uint8_t sNumSensors;
    static uint8_t sMaxI2cSpeed;
    uint16_t gMeasureTypes;

  protected:
    // Sensor();
    Sensor(uint16_t iMeasureTypes, uint8_t iAddress);
    virtual ~Sensor() {}

    uint8_t gAddress; 
    SensorState gSensorState = Wakeup;
    uint32_t pSensorStateDelay = 0;
    float gTempOffset = 0.0;

    virtual uint8_t getSensorClass() = 0; // pure; returns unique ID for this sensor type
    bool checkSensorConnection();
    virtual float measureValue(MeasureType iMeasureType) = 0; //pure
    virtual void sensorLoopInternal();
    virtual void sensorSaveState();
    // non blocking restart approach for a sensor
    void restartSensor();
    virtual bool begin(); // first initialization, may be blocking, should be called druing setup(), not during loop()

  public:
    // static 
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
