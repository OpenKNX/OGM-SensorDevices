// Sensor MAX31865 - PT1000 interface
#if defined(SENSORMODULE) && defined(OPENKNX_SPI_MISO)
    #include "SPI.h"
    #include "SensorMAX31865.h"

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
        mPT1000 = new Adafruit_MAX31865(OPENKNX_SPI_CS,&SPI);
        lResult = mPT1000->begin(MAX31865_3WIRE);
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
    mTemp = mPT1000->calculateTemperature(doReadRTD(),RNOMINAL, RREF);
    uint8_t fault = mPT1000->readFault();

    return (fault == 0);
}

void SensorMAX31865::setupReadRTD(void) {

    mPT1000->clearFault();
    mPT1000->enableBias(true);
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
    mPT1000->enableBias(false); // Disable bias current again to reduce selfheating.
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
  spi_dev.write_then_read(&addr, 1, buffer, n);
}

void SensorMAX31865::writeRegister8(uint8_t addr, uint8_t data) {
  addr |= 0x80; // make sure top bit is set
  uint8_t buffer[2] = {addr, data};
  spi_dev.write(buffer, 2);
}

#endif
