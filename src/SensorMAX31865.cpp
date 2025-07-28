// Sensor MAX31865 - PT1000 interface
#include "SensorMAX31865.h"
#if defined(SENSORMODULE) && defined(OPENKNX_SPI_MISO)
    #include "SPI.h"

SensorMAX31865::SensorMAX31865(uint16_t iMeasureTypes, TwoWire* iWire)
    : Sensor(iMeasureTypes, iWire, 0){};

SensorMAX31865::SensorMAX31865(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress)
    : Sensor(iMeasureTypes, iWire, iAddress){};

uint8_t SensorMAX31865::getSensorClass()
{
    return SENS_MAX31865;
}

std::string SensorMAX31865::logPrefix()
{
    return "Sensor<MAX31865>";
}

void SensorMAX31865::sensorLoopInternal()
{
    switch (pSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
        case Calibrate:
            Sensor::sensorLoopInternal();
            break;
        case Finalize:
            // we ask for Temperature until we get a valid value
            if (delayCheck(pSensorStateDelay, 200))
            {
                if (getSensorData())
                    pSensorState = Running;
                pSensorStateDelay = millis();
            }
            break;
        case Running:
            sensorReadLoop();
            break;
        default:
            pSensorStateDelay = millis();
            break;
    }
}

void SensorMAX31865::sensorReadLoop()
{
    switch (pReadState)
    {
        case Setup:
            if (delayCheck(pSensorStateDelay, 900))
            {
                setupReadRTD();
                pReadState = Prepare;
                pSensorStateDelay = millis();
            }
            break;
        case Prepare:
            if (delayCheck(pSensorStateDelay, 10))
            {
                prepareReadRTD();
                pReadState = GetSensorData;
                pSensorStateDelay = millis();
            }
            break;
        case GetSensorData:
            if (delayCheck(pSensorStateDelay, 65))
            {
                getSensorData();
                pReadState = Setup;
                pSensorStateDelay = millis();
            }
            break;
        default:
            pSensorStateDelay = millis();
            break;
    }
}

bool SensorMAX31865::checkSensorConnection()
{
    return true; // always true for SPI sensors
}

float SensorMAX31865::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
        case Temperature:
            // hardware calibration
            return mTemp;
            break;
        default:
            break;
    }
    return NO_NUM;
}

bool SensorMAX31865::begin()
{
    logDebugP("Starting sensor MAX31865... ");
    bool lResult = Sensor::begin();
    if (lResult)
    {
        SPI.setRX(OPENKNX_SPI_MISO); // or setMISO()
        SPI.setCS(OPENKNX_SPI_CS);
        SPI.setSCK(OPENKNX_SPI_SCK);
        SPI.setTX(OPENKNX_SPI_MOSI); // or setMOSI()
        spi_dev = new Adafruit_SPIDevice(OPENKNX_SPI_CS, 1000000, SPI_BITORDER_MSBFIRST, SPI_MODE1, &SPI);
        spi_dev->begin();

        setWires(MAX31865_3WIRE);
        enableBias(false);
        autoConvert(false);
        setThresholds(0, 0xFFFF);
        clearFault();
    }
    logResult(lResult);
    return lResult;
}

uint8_t SensorMAX31865::getI2cSpeed()
{
    return 10; // n * 100kHz
}

bool SensorMAX31865::getSensorData()
{
    mTemp = calculateTemperature(doReadRTD(),RNOMINAL, RREF);
    uint8_t fault = readFault();

    return (fault == 0);
}

void SensorMAX31865::setWires(max31865_numwires_t wires) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (wires == MAX31865_3WIRE) {
    t |= MAX31865_CONFIG_3WIRE;
  } else {
    // 2 or 4 wire
    t &= ~MAX31865_CONFIG_3WIRE;
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
}

void SensorMAX31865::enableBias(bool b) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_BIAS; // enable bias
  } else {
    t &= ~MAX31865_CONFIG_BIAS; // disable bias
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
}

void SensorMAX31865::autoConvert(bool b) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  if (b) {
    t |= MAX31865_CONFIG_MODEAUTO; // enable autoconvert
  } else {
    t &= ~MAX31865_CONFIG_MODEAUTO; // disable autoconvert
  }
  writeRegister8(MAX31865_CONFIG_REG, t);
}

void SensorMAX31865::setThresholds(uint16_t lower, uint16_t upper) {
  writeRegister8(MAX31865_LFAULTLSB_REG, lower & 0xFF);
  writeRegister8(MAX31865_LFAULTMSB_REG, lower >> 8);
  writeRegister8(MAX31865_HFAULTLSB_REG, upper & 0xFF);
  writeRegister8(MAX31865_HFAULTMSB_REG, upper >> 8);
}

void SensorMAX31865::clearFault(void) {
  uint8_t t = readRegister8(MAX31865_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31865_CONFIG_FAULTSTAT;
  writeRegister8(MAX31865_CONFIG_REG, t);
}

float SensorMAX31865::calculateTemperature(uint16_t RTDraw, float RTDnominal, float refResistor) {
  float Z1, Z2, Z3, Z4, Rt, temp;

  Rt = RTDraw;
  Rt /= 32768;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0)
    return temp;

  // ugh.
  Rt /= RTDnominal;
  Rt *= 100; // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}

uint8_t SensorMAX31865::readFault(max31865_fault_cycle_t fault_cycle) {
  if (fault_cycle) {
    uint8_t cfg_reg = readRegister8(MAX31865_CONFIG_REG);
    cfg_reg &= 0x11; // mask out wire and filter bits
    switch (fault_cycle) {
    case MAX31865_FAULT_AUTO:
      writeRegister8(MAX31865_CONFIG_REG, (cfg_reg | 0b10000100));
      delay(1);
      break;
    case MAX31865_FAULT_MANUAL_RUN:
      writeRegister8(MAX31865_CONFIG_REG, (cfg_reg | 0b10001000));
      return 0;
    case MAX31865_FAULT_MANUAL_FINISH:
      writeRegister8(MAX31865_CONFIG_REG, (cfg_reg | 0b10001100));
      return 0;
    case MAX31865_FAULT_NONE:
    default:
      break;
    }
  }
  return readRegister8(MAX31865_FAULTSTAT_REG);
}




void SensorMAX31865::setupReadRTD(void) {

    clearFault();
    enableBias(true);
    // delay(10);
}

void SensorMAX31865::prepareReadRTD(void) {

    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    t |= MAX31865_CONFIG_1SHOT;
    writeRegister8(MAX31865_CONFIG_REG, t);
    // delay(65);
}

uint16_t SensorMAX31865::doReadRTD(void) {

    uint16_t rtd = readRegister16(MAX31865_RTDMSB_REG);
    enableBias(false); // Disable bias current again to reduce selfheating.
    // remove fault
    rtd >>= 1;
    return rtd;
}

/**********************************************/

uint8_t SensorMAX31865::readRegister8(uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);

  return ret;
}

uint16_t SensorMAX31865::readRegister16(uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];

  return ret;
}

void SensorMAX31865::readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n)
{
  addr &= 0x7F; // make sure top bit is not set
  spi_dev->write_then_read(&addr, 1, buffer, n);
}

void SensorMAX31865::writeRegister8(uint8_t addr, uint8_t data) {
  addr |= 0x80; // make sure top bit is set
  uint8_t buffer[2] = {addr, data};
  spi_dev->write(buffer, 2);
}

#endif
