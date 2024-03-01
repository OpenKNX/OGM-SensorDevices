// #include "IncludeManager.h"
#ifdef SENSORMODULE
    #include "SensorBME280.h"

SensorBME280::SensorBME280(uint16_t iMeasureTypes, TwoWire &iWire)
    : Sensor(iMeasureTypes, iWire, BME280_I2C_ADDR), Adafruit_BME280(){};

SensorBME280::SensorBME280(uint16_t iMeasureTypes, TwoWire &iWire, uint8_t iAddress)
    : Sensor(iMeasureTypes, iWire, iAddress), Adafruit_BME280(){};

uint8_t SensorBME280::getSensorClass()
{
    return SENS_BME280;
}

std::string SensorBME280::logPrefix()
{
    return "Sensor<BME280>";
}

/*!
 * @brief implements a state engine to restart the sensor without delays
 */
void SensorBME280::sensorLoopInternal()
{
    switch (pSensorState)
    {
        case Wakeup:
            if (pSensorStateDelay == 0 || delayCheck(pSensorStateDelay, 1000))
            {
                if (SensorBME280::begin() && initWakeup()) pSensorState = Calibrate;
                pSensorStateDelay = millis();
            }
            break;
        case Calibrate:
            if (delayCheck(pSensorStateDelay, 100))
            {
                if (!isReadingCalibration())
                {
                    initFinalize();
                    pSensorState = Finalize;
                    logResult(true);
                }
                pSensorStateDelay = millis();
            }
            break;
        case Finalize:
            Sensor::sensorLoopInternal();
            break;
        default:
            Sensor::sensorLoopInternal();
            break;
    }
}

/*!
 *   @brief  Initialise sensor with given parameters / settings, skips all delay functions, expects correct external call sequence
 *   @returns true on success, false otherwise
 */
bool SensorBME280::initWakeup()
{
    // check if sensor, i.e. the chip ID is correct
    _sensorID = read8(BME280_REGISTER_CHIPID);
    if (_sensorID != 0x60)
        return false;

    // reset the device using soft-reset
    // this makes sure the IIR is off, etc.
    write8(BME280_REGISTER_SOFTRESET, 0xB6);
    return true;
}

bool SensorBME280::initFinalize()
{
    readCoefficients(); // read trimming parameters, see DS 4.2.2
    setSampling();      // use defaults
    setTemperatureCompensation(pTempOffset);
    return true;
}

float SensorBME280::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
        case Temperature:
            // hardware calibration
            return readTemperature();
            break;
        case Humidity:
            return readHumidity();
            break;
        case Pressure:
            return readPressure();
        default:
            break;
    }
    return -1000.0;
}

bool SensorBME280::begin()
{
    logDebugP("Starting sensor BME280... ");
    _i2caddr = pI2CAddress;
    _wire = &pWire;
    bool lResult = Sensor::begin();
    pSensorState = Wakeup;
    // logResult(lResult);
    return lResult;
}

uint8_t SensorBME280::getI2cSpeed()
{
    return 4; // n * 100kHz
}

bool SensorBME280::prepareTemperatureOffset(float iTemp)
{
    pTempOffset = iTemp;
    return true;
}
#endif
