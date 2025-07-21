// Sensor MAX31865 - PT1000 interface

#pragma once
#if defined(SENSORMODULE) && defined(OPENKNX_SPI_MISO)
  #include "Adafruit_MAX31865.h"
  #include "Sensor.h"
  
  // The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
  #define RREF      4300.0
  // The 'nominal' 0-degrees-C resistance of the sensor
  // 100.0 for PT100, 1000.0 for PT1000
  #define RNOMINAL  1000.0

class SensorMAX31865 : public Sensor
{

  enum ReadState
  {
      Setup,
      Prepare,
      GetSensorData
  };

  private:
    Adafruit_MAX31865 *mPT1000;
    bool getSensorData();

  protected:
    float mTemp = NO_NUM;
    ReadState pReadState = Setup;

    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    float measureValue(MeasureType iMeasureType) override;
    bool checkSensorConnection() override;
    void sensorReadLoop();
    void setupReadRTD(void); 
    void prepareReadRTD(void); 
    uint16_t doReadRTD(void);
    
    uint8_t readRegister8(uint8_t addr); 
    uint16_t readRegister16(uint8_t addr);
    void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);
    void writeRegister8(uint8_t addr, uint8_t data);

  public:
    SensorMAX31865(uint16_t iMeasureTypes, TwoWire* iWire);
    SensorMAX31865(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress);
    virtual ~SensorMAX31865() {}
    
    bool begin() override;
    uint8_t getI2cSpeed() override;
    std::string logPrefix() override;
  };
#endif
