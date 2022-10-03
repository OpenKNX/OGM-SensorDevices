// #include "IncludeManager.h"
#if defined(SENSORMODULE) || defined(PMMODULE)
#include <Arduino.h>
#include <Wire.h>
#include "Sensor.h"
#include "HardwareDevices.h"

// static
uint8_t Sensor::sNumSensors = 0;
uint8_t Sensor::sMaxI2cSpeed = 255;
Sensor* Sensor::sSensors[SENSOR_COUNT];
TwoWire &Sensor::sWire = Wire;

Sensor* newSensor(uint8_t iSensorClass, MeasureType iMeasureType);

Sensor* Sensor::factory(uint8_t iSensorClass, MeasureType iMeasureType) {
    Sensor* lSensor = nullptr;
    // first check, if there is already an instance
    for (size_t lCount = 0; lCount < sNumSensors; lCount++)
    {
        if (sSensors[lCount]->getSensorClass() == iSensorClass)
        {
            lSensor = sSensors[lCount];
            break;
        }
    }
    if (lSensor == nullptr) {
        lSensor = newSensor(iSensorClass, iMeasureType);
    }
    // at this point we have a valid sensor or a sensor dummy!
    lSensor->gMeasureTypes |= iMeasureType;
    uint8_t lI2cSpeed = lSensor->getI2cSpeed();
    // determine maximum available i2c speed
    if (lI2cSpeed < sMaxI2cSpeed)
        sMaxI2cSpeed = lI2cSpeed;
    return lSensor;
}

Sensor::Sensor(uint16_t iMeasureTypes, uint8_t iAddress)
{
    if (sNumSensors >= SENSOR_COUNT)
    {
        // println("Sensor::Sensor() - Currently only 2 (Hardware)Sensors are supported");
        //fatal error handling
        return;
    }
    gMeasureTypes = iMeasureTypes;
    gAddress = iAddress;
    gWire = sWire;
    sSensors[sNumSensors++] = this;
};

// static
void Sensor::SetWire(TwoWire &iWire) 
{
    sWire = iWire;
}

void Sensor::sensorLoop() {
    for (uint8_t lCounter = 0; lCounter < sNumSensors; lCounter++)
        sSensors[lCounter]->sensorLoopInternal();
    }

// static
void Sensor::restartSensors() {
    for (uint8_t lCounter = 0; lCounter < sNumSensors; lCounter++)
        sSensors[lCounter]->restartSensor();
}

// static
bool Sensor::beginSensors()
{
    bool lResult = true;
    // fist we start i2c with the right speed
    if (sNumSensors > 0) {
        sWire.begin();
        sWire.setClock(sMaxI2cSpeed * 100000);
        delay(1);
        // we use standard Wakeup procedure to start single sensors
        // for (uint8_t lCounter = 0; lCounter < sNumSensors; lCounter++)
        //     lResult = sSensors[lCounter]->begin() && lResult;
    }
    return lResult;
}

// static
uint8_t Sensor::getMaxI2cSpeed()
{
    return sMaxI2cSpeed; // n * 100kHz
}

void Sensor::restartSensor() {
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
        gWire.beginTransmission(gAddress);
        lResult = (gWire.endTransmission() == 0);
        if (!lResult)
            restartSensor();
    // }
    return lResult;
}

void Sensor::sensorLoopInternal() {
    switch (gSensorState)
    {
    case Wakeup:
        // try immediately to start the sensor, then every second
        if (pSensorStateDelay == 0 || delayCheck(pSensorStateDelay, 1000)) {
            // if (begin()) gSensorState = Calibrate;
            gSensorState = begin() ? Calibrate : Off;
            pSensorStateDelay = millis();
        }
        break;
    case Calibrate:
        // no calibration necessary
        gSensorState = Finalize;
        break;
    case Finalize:
        // give the sensor 100 ms before querying starts
        if (delayCheck(pSensorStateDelay, 100)) gSensorState = Running;
        break;
    default:
        pSensorStateDelay = 0;
        break;
    }
}

bool Sensor::begin() {
    // gSensorState = Running;
    return checkSensorConnection();
}

uint8_t Sensor::getI2cSpeed() {
    return 1;  // n * 100kHz
}

bool Sensor::prepareTemperatureOffset(float iTemp)
{
    return false;
}

// should be overridden, if there is a state to save before power failure
void Sensor::sensorSaveState() {};

// static
void Sensor::saveState() {
    // dispatch the call to all sensors
    for (uint8_t lCounter = 0; lCounter < sNumSensors; lCounter++)
        sSensors[lCounter]->sensorSaveState();
}

// static
bool Sensor::measureValue(MeasureType iMeasureType, float& eValue) {
    bool lResult = false;
    for (uint8_t lCounter = 0; lCounter < sNumSensors; lCounter++)
    {
        if (sSensors[lCounter]->gMeasureTypes & iMeasureType) {
            lResult = sSensors[lCounter]->checkSensorConnection();
            if (lResult)
                lResult = sSensors[lCounter]->gSensorState == Running;
            if (lResult) eValue = sSensors[lCounter]->measureValue(iMeasureType);
            if (lResult)
                lResult = (eValue > (NO_NUM + 1.0));
            break;
        }
    }
    return lResult;
}

//static
uint8_t Sensor::getError() {
    uint8_t lResult = 0;
    // for (uint8_t lCounter = 0; lCounter < sNumSensors; lCounter++)
    // {
    //     if (sSensors[lCounter]->gSensorState != Running) {
    //         lResult |= sSensors[lCounter]->gMeasureTypes;
    //     }
    // }
    return lResult;
}
#endif