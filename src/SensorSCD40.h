// #include "IncludeManager.h"
#pragma once
#ifdef SENSORMODULE
    #include "Sensor.h"
    #include <SensirionI2cScd4x.h>

    #define SCD40_I2C_ADDR 0x62

class SensorSCD40 : public Sensor, protected SensirionI2cScd4x
{
  private:  
    float mTemp = NO_NUM;
    float mHum = NO_NUM;
    float mCo2 = NO_NUM;
    uint32_t mPressure = 0;
    uint8_t mRetryCounter = 0;

  protected:
    static int16_t sError;
    static int8_t sInitStep;
    static char sErrorMessage[64];
    static uint8_t communication_buffer[9];
    uint8_t mSensorVariant = 0; // SCD40 or SCD41
    bool pSensorStateStopped = false; // additional state, we use this for diagnose commands

    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    float measureValue(MeasureType iMeasureType) override;

    int16_t stopPeriodicMeasurement(bool blocking);

    uint8_t calibrate(uint8_t iStep);
    virtual uint8_t calibrateExtended();
    bool beginInternal();
    bool getSensorData();
    void processPressure();
    void logSensorError(int16_t iError, const char* iErrorMessage);
    void sensorShowHelp() override;
    bool sensorProcessCommand(const std::string iCmd, bool iDebugKo) override;

    bool isInt(const std::string& s);
    int16_t getIntArg(const std::string& s);

  public:
    SensorSCD40(uint16_t iMeasureTypes, TwoWire* iWire);
    SensorSCD40(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress);
    virtual ~SensorSCD40() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
    bool prepareTemperatureOffset(float iTempOffset) override;
    bool setPressure(uint32_t pressure);
    std::string logPrefix() override;
};
#endif
