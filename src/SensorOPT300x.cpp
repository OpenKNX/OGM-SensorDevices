// #include "IncludeManager.h"
#if defined(SENSORMODULE) || defined(PMMODULE)
    #include <Wire.h>
    // #include "OpenKNX/Hardware.h"
    #include "SensorOPT300x.h"

SensorOPT300x::SensorOPT300x(uint16_t iMeasureTypes, TwoWire* iWire)
    : Sensor(iMeasureTypes, iWire, OPT300X_I2C_ADDR){};

SensorOPT300x::SensorOPT300x(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress)
    : Sensor(iMeasureTypes, iWire, iAddress){};

uint8_t SensorOPT300x::getSensorClass()
{
    return SENS_OPT300X;
}

std::string SensorOPT300x::logPrefix()
{
    return "Sensor<OPT300x>";
}

void SensorOPT300x::sensorLoopInternal()
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
            if (delayCheck(pSensorStateDelay, 2000))
            {
                getSensorData();
                pSensorStateDelay = millis();
            }
            break;
        default:
            pSensorStateDelay = millis();
            break;
    }
}

float SensorOPT300x::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
        case Lux:
            // hardware calibration
            return mLux;
            break;
        default:
            break;
    }
    return NO_NUM;
}

bool SensorOPT300x::begin()
{
    logDebugP("Starting sensor OPT300x... ");
    // #ifdef SENSOR_I2C_OPT300x
    //     gWire = SENSOR_I2C_OPT300x;
    //     gWire.begin();
    // #endif
    bool lResult = Sensor::begin();
    if (lResult)
    {
        OPT300xConfig lConfig;
        lConfig.rangeNumber = OPT300X_CONF_AUTO_FULL_RANGE;
        lConfig.conversionTime = OPT300X_CONF_CONV_TIME_800;
        lConfig.modeOfConversionOperation = OPT300X_CONF_CONV_OPERATION_CONTINIOUS;
        lConfig.latch = OPT300X_CONF_LATCH_WINDOW;
        writeConfig(lConfig);
    }
    logResult(lResult);
    return lResult;
}

uint8_t SensorOPT300x::getI2cSpeed()
{
    return 4; // n * 100kHz
}

bool SensorOPT300x::getSensorData()
{
    // clear read buffer
    mBuffer[0] = 0;
    mBuffer[1] = 0;
    uint16_t lRaw;

    pWire->beginTransmission(pI2CAddress);
    pWire->write(OPT300X_REG_RESULT); // Send result register address
    if (pWire->endTransmission() != 0)
        return false;

    // request sensor data
    pWire->requestFrom(pI2CAddress, 2);
    if (pWire->available() != 2)
        return false;
    pWire->readBytes(mBuffer, 2);
    lRaw = (mBuffer[0] << 8) | mBuffer[1];
    mLux = (lRaw & 0x0FFF) * (0.01 * pow(2, (lRaw & 0xF000) >> 12));
    return true;
}

bool SensorOPT300x::writeConfig(OPT300xConfig iConfig)
{
    pWire->beginTransmission(pI2CAddress);
    pWire->write(OPT300X_REG_CONFIG);
    pWire->write(iConfig.rawData >> 8);
    pWire->write(iConfig.rawData & 0x00FF);
    return (pWire->endTransmission() == 0);
}
#endif
