// #include "IncludeManager.h"
#if defined(SENSORMODULE) || defined(PMMODULE)
    #include "Sensor.h"
    #include <Arduino.h>
    #include <Wire.h>
// #include "OpenKNX/Hardware.h"

Sensor::Sensor(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress)
{
    pMeasureTypes = iMeasureTypes;
    pI2CAddress = iAddress;
    pWire = iWire;
}

void Sensor::restartSensor()
{
    // pSensorStateDelay = 0;
    // gSensorState = Wakeup;
}

// void Sensor::changeSensorOrder(Sensor *iSensor, uint8_t iPosition){
//     // first check, if the sensor is already at his position
//     if (sSensors[iPosition] == iSensor) return;
//     // as long as we have just 2 Sensors, new position is a simple exchange
//     int8_t lNewPosition = abs(iPosition - 1);
//     sSensors[lNewPosition] = sSensors[iPosition];
//     sSensors[iPosition] = iSensor;
// }

bool Sensor::checkSensorConnection()
{
    // just valid for i2c sensors, in other cases this should be overridden
    bool lResult = false;
    // if (gSensorState == Running) {
    // check for I2C ack
    pWire->beginTransmission(pI2CAddress);
    lResult = (pWire->endTransmission() == 0);
    if (!lResult)
        restartSensor();
    // }
    return lResult;
}

void Sensor::sensorLoopInternal()
{
    switch (pSensorState)
    {
        case Wakeup:
            // try immediately to start the sensor, then every second
            if (pSensorStateDelay == 0 || delayCheck(pSensorStateDelay, 1000))
            {
                // if (begin()) gSensorState = Calibrate;
                pSensorState = begin() ? Calibrate : Off;
                pSensorStateDelay = millis();
            }
            break;
        case Calibrate:
            // no calibration necessary
            pSensorState = Finalize;
            break;
        case Finalize:
            // give the sensor 100 ms before querying starts
            if (delayCheck(pSensorStateDelay, 100)) pSensorState = Running;
            break;
        default:
            pSensorStateDelay = 0;
            break;
    }
}

bool Sensor::begin()
{
    // gSensorState = Running;
    return checkSensorConnection();
}

uint8_t Sensor::getI2cSpeed()
{
    return 1; // n * 100kHz
}

bool Sensor::prepareTemperatureOffset(float iTemp)
{
    return false;
}

// // should be overridden, if there is a state to save before power failure
// void Sensor::sensorSaveState(){};

// should be overridden, if there is a state to save before power failure
void Sensor::sensorReadFlash(const uint8_t* iBuffer, const uint16_t iSize)
{
    if (iSize == 0) // first call - without data
        return;
}

// should be overridden, if there is a state to save before power failure
void Sensor::sensorWriteFlash()
{
    // openknx.flash.writeByte(1); // Version
}

// should be overridden, if there is a state to save before power failure
uint16_t Sensor::sensorFlashSize()
{
    return 0;
}

void Sensor::logResult(bool iResult)
{
    logDebugP(iResult ? "OK" : "FAIL");
}

void Sensor::addMeasureType(MeasureType iMeasureType)
{
    pMeasureTypes |= iMeasureType;
}

bool Sensor::checkMeasureType(MeasureType iMeasureType)
{
    return pMeasureTypes & iMeasureType;
}

bool Sensor::measureValue(MeasureType iMeasureType, float& eValue)
{
    bool lResult = false;
    if (pMeasureTypes & iMeasureType)
    {
        lResult = checkSensorConnection();
        if (lResult)
            lResult = pSensorState == Running;
        if (lResult) eValue = measureValue(iMeasureType);
        if (lResult)
            lResult = (eValue > (NO_NUM + 1.0));
    }
    return lResult;
}

#endif

std::string Sensor::logPrefix()
{
    return "Sensor";
}
