#include "SensorDevices.h"

SensorDevices openknxSensorDevicesModule;

SensorDevices::SensorDevices(/* args */)
{
}

SensorDevices::~SensorDevices()
{
}

void SensorDevices::setup()
{
}

const std::string SensorDevices::name()
{
    return "SensorDevices";
}

const std::string SensorDevices::version()
{
    return MODULE_SensorDevices_Version;
}

Sensor* SensorDevices::factory(uint8_t iSensorClass, MeasureType iMeasureType)
{
    Sensor* lSensor = nullptr;
    // first check, if there is already an instance
    for (size_t lCount = 0; lCount < mNumSensors; lCount++)
    {
        if (mSensors[lCount]->getSensorClass() == iSensorClass)
        {
            lSensor = mSensors[lCount];
            break;
        }
    }
    if (lSensor == nullptr)
    {
        if (mNumSensors >= MAX_SUPPORTED_SENSORS)
        {
            logErrorP("Too many Sensor instances in SensorDevices");
            iSensorClass = SENS_NO;
        }
        lSensor = newSensor(iSensorClass, iMeasureType, mWire);
        mSensors[mNumSensors++] = lSensor;
    }
    // at this point we have a valid sensor or a sensor dummy!
    lSensor->addMeasureType(iMeasureType);
    uint8_t lI2cSpeed = lSensor->getI2cSpeed();
    // determine maximum available i2c speed
    if (lI2cSpeed < mMaxI2cSpeed)
        mMaxI2cSpeed = lI2cSpeed;
    return lSensor;
}

void SensorDevices::loop()
{
    uint8_t lProcessedSensors = 0;
    if (mNumSensors == 0) return;
    do
    {
        mSensors[mCurrentSensorIterator]->sensorLoopInternal();
    }
    while (openknx.common.freeLoopIterate(mNumSensors, mCurrentSensorIterator, lProcessedSensors));
}

// static
void SensorDevices::restartSensors()
{
    for (uint8_t lCounter = 0; lCounter < mNumSensors; lCounter++)
        mSensors[lCounter]->restartSensor();
}

// static
bool SensorDevices::beginSensors()
{
    bool lResult = true;
    // fist we start i2c with the right speed
    if (mNumSensors > 0)
    {
        mWire.begin();
        mWire.setClock(mMaxI2cSpeed * 100000);
        delay(1);
        // we use standard Wakeup procedure to start single sensors
        // for (uint8_t lCounter = 0; lCounter < sNumSensors; lCounter++)
        //     lResult = sSensors[lCounter]->begin() && lResult;
    }
    return lResult;
}

// static
uint8_t SensorDevices::getMaxI2cSpeed()
{
    return mMaxI2cSpeed; // n * 100kHz
}

void SensorDevices::readFlash(const uint8_t* iBuffer, const uint16_t iSize)
{
    // dispatch the call to all sensors
    for (uint8_t lCounter = 0; lCounter < mNumSensors; lCounter++)
        mSensors[lCounter]->sensorReadFlash(iBuffer, iSize);
}

void SensorDevices::writeFlash()
{
    // dispatch the call to all sensors
    for (uint8_t lCounter = 0; lCounter < mNumSensors; lCounter++)
        mSensors[lCounter]->sensorWriteFlash();
}

uint16_t SensorDevices::flashSize()
{
    uint16_t lResult = 0;
    // dispatch the call to all sensors
    for (uint8_t lCounter = 0; lCounter < mNumSensors; lCounter++)
        lResult += mSensors[lCounter]->sensorFlashSize();
    return lResult;
}

bool SensorDevices::measureValue(MeasureType iMeasureType, float& eValue)
{
    bool lResult = false;
    for (uint8_t lCounter = 0; lCounter < mNumSensors; lCounter++)
        if (mSensors[lCounter]->checkMeasureType(iMeasureType))
        {
            lResult = mSensors[lCounter]->measureValue(iMeasureType, eValue);
            break;
        }
    return lResult;
}

uint8_t SensorDevices::getError()
{
    uint8_t lResult = 0;
    // for (uint8_t lCounter = 0; lCounter < sNumSensors; lCounter++)
    // {
    //     if (sSensors[lCounter]->gSensorState != Running) {
    //         lResult |= sSensors[lCounter]->gMeasureTypes;
    //     }
    // }
    return lResult;
}

void SensorDevices::defaultWire(TwoWire& iWire)
{
    mWire = iWire;
}
