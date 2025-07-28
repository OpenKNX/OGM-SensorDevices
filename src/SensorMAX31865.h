// Sensor MAX31865 - PT1000 interface

#pragma once
#ifdef SENSORMODULE
#include "Sensor.h"
#ifdef OPENKNX_SPI_MISO
  #include "Sensor.h"
  #include "Adafruit_SPIDevice.h"

  // The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
  #define RREF      4300.0
  // The 'nominal' 0-degrees-C resistance of the sensor
  // 100.0 for PT100, 1000.0 for PT1000
  #define RNOMINAL  1000.0

  #define MAX31865_CONFIG_REG 0x00
  #define MAX31865_CONFIG_BIAS 0x80
  #define MAX31865_CONFIG_MODEAUTO 0x40
  #define MAX31865_CONFIG_MODEOFF 0x00
  #define MAX31865_CONFIG_1SHOT 0x20
  #define MAX31865_CONFIG_3WIRE 0x10
  #define MAX31865_CONFIG_24WIRE 0x00
  #define MAX31865_CONFIG_FAULTSTAT 0x02
  #define MAX31865_CONFIG_FILT50HZ 0x01
  #define MAX31865_CONFIG_FILT60HZ 0x00

  #define MAX31865_RTDMSB_REG 0x01
  #define MAX31865_RTDLSB_REG 0x02
  #define MAX31865_HFAULTMSB_REG 0x03
  #define MAX31865_HFAULTLSB_REG 0x04
  #define MAX31865_LFAULTMSB_REG 0x05
  #define MAX31865_LFAULTLSB_REG 0x06
  #define MAX31865_FAULTSTAT_REG 0x07

  #define MAX31865_FAULT_HIGHTHRESH 0x80
  #define MAX31865_FAULT_LOWTHRESH 0x40
  #define MAX31865_FAULT_REFINLOW 0x20
  #define MAX31865_FAULT_REFINHIGH 0x10
  #define MAX31865_FAULT_RTDINLOW 0x08
  #define MAX31865_FAULT_OVUV 0x04

  #define RTD_A 3.9083e-3
  #define RTD_B -5.775e-7

typedef enum max31865_numwires {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;

typedef enum {
  MAX31865_FAULT_NONE = 0,
  MAX31865_FAULT_AUTO,
  MAX31865_FAULT_MANUAL_RUN,
  MAX31865_FAULT_MANUAL_FINISH
} max31865_fault_cycle_t;


class SensorMAX31865 : public Sensor
{

  enum ReadState
  {
      Setup,
      Prepare,
      GetSensorData
  };

  private:
    Adafruit_SPIDevice *spi_dev;
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
    void setWires(max31865_numwires_t wires);
    void enableBias(bool b);
    void autoConvert(bool b);
    void setThresholds(uint16_t lower, uint16_t upper);
    void clearFault(void);
    float calculateTemperature(uint16_t RTDraw, float RTDnominal, float refResistor);
    uint8_t readFault(max31865_fault_cycle_t fault_cycle = MAX31865_FAULT_AUTO);

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
#endif
