#include "SensorDevices.h"
#include "SensorSCD41.h"

SensorDevices openknxSensorDevicesModule;

SensorDevices::SensorDevices(/* args */)
{
}

SensorDevices::~SensorDevices()
{
}

void SensorDevices::setup(bool iConfigured)
{
#ifdef ARDUINO_ARCH_RP2040
    #ifndef I2C_WIRE
        #define I2C_WIRE Wire
    #endif
    #ifdef I2C_SDA_PIN
        #ifdef I2C_SCL_PIN
    I2C_WIRE.setSDA(I2C_SDA_PIN);
    I2C_WIRE.setSCL(I2C_SCL_PIN);
        #endif
    #endif
    defaultWire(I2C_WIRE);
#endif
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

void SensorDevices::loop(bool iConfigured)
{
    if (iConfigured || mSensorTestDelayTimer > 0) {
        uint8_t lProcessedSensors = 0;
        if (mNumSensors == 0) return;
        do
        {
            mSensors[mCurrentSensorIterator]->sensorLoopInternal();
        }
        while (openknx.common.freeLoopIterate(mNumSensors, mCurrentSensorIterator, lProcessedSensors));
    }
    if (mSensorTestDelayTimer > 0 && delayCheck(mSensorTestDelayTimer, 500)) testSensorMeasurement();
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
        mWire->begin();
        mWire->setClock(mMaxI2cSpeed * 100000);
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

void SensorDevices::savePower()
{
    // dispatch the call to all sensors
    for (uint8_t lCounter = 0; lCounter < mNumSensors; lCounter++)
        mSensors[lCounter]->sensorSavePower();
}

bool SensorDevices::restorePower()
{
    bool lResult = true;
    // dispatch the call to all sensors
    for (uint8_t lCounter = 0; lCounter < mNumSensors; lCounter++)
        lResult = mSensors[lCounter]->sensorRestorePower() && lResult;
    return lResult;
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
    mWire = &iWire;
}

void SensorDevices::showHelp()
{
    if (knx.configured())
        // dispatch the call to all sensors
        for (uint8_t lCounter = 0; lCounter < mNumSensors; lCounter++)
            mSensors[lCounter]->sensorShowHelp();
    else
        openknx.console.printHelpLine("sen test mode", "Test all connected sensors - just for development, do not use while device is running!");
}

bool SensorDevices::processCommand(const std::string iCmd, bool iDiagnoseKo)
{
    bool lResult = false;
    if (iCmd.length() == 13 && iCmd == "sen test mode") {
        logInfoP("Testing all sensors");
        mSensorTestDelayTimer = delayTimerInit() + 5000;
        testSensors();
        lResult = true;
    }
    else {
        if (knx.configured()) 
            for (uint8_t lCounter = 0; lCounter < mNumSensors && !lResult; lCounter++)
                lResult = mSensors[lCounter]->sensorProcessCommand(iCmd, iDiagnoseKo) || lResult;
    }
    return lResult;
}

void SensorDevices::testSensors()
{
    // this method tests all sensors 
    // each sensor is initialized and should provide at least one measure value
    factory(SENS_SHT3X, Temperature);
    factory(SENS_BME280, Temperature); // tests also BMP280 and BME680
    SensorSCD41 *lSensor = (SensorSCD41 *)factory(SENS_SCD41, Temperature);
    lSensor->setMeasureInterval(5); // 5 seconds interval for test
    factory(SENS_VL53L1X, Tof);
    factory(SENS_OPT300X, Lux);
    factory(SENS_VEML7700, Lux);

#ifdef HF_POWER_PIN
    pinMode(HF_POWER_PIN, OUTPUT);
    // at startup, we turn HF-Sensor on
    digitalWrite(HF_POWER_PIN, HIGH);
    delay(100);

    // ensure no data lost even for sensor raw data
    // up to 1288 bytes are send by the sensor at once
    HF_SERIAL.setFIFOSize(1300);
    HF_SERIAL.setRX(HF_UART_RX_PIN);
    HF_SERIAL.setTX(HF_UART_TX_PIN);
    HF_SERIAL.begin(HF_SERIAL_SPEED);
    // factory(SENS_MR24xxB1, Pres);
    factory(SENS_HLKLD2420, Pres);
#endif
    delay(100);

    beginSensors();
    mSensorTestDuration = delayTimerInit();
}

void SensorDevices::testSensorMeasurement() {
    // slow, but its just test mode...
    MeasureType lMeasureTypes[] = {Temperature, Tof, Lux, Pres}; 
    static bool sCheckedSensors[MAX_SUPPORTED_SENSORS] = {false};
    bool lFinished = true;
    for (uint8_t lCounter = 0; lCounter < mNumSensors; lCounter++) {
        if (!sCheckedSensors[lCounter]) {
            Sensor* lSensor = mSensors[lCounter];
            lFinished = false;
            SensorState lSensorState = lSensor->getSensorState();
            logDebug(lSensor->logPrefix(), "Current state is: %s", lSensor->getSensorStateAsString());
            if (lSensorState == SensorState::Off) 
            {
                sCheckedSensors[lCounter] = true;
            }
            if (lSensorState == SensorState::Running) {
                for (uint8_t lMeasureIndex = 0; lMeasureIndex < 4; lMeasureIndex++)
                {
                    if (lSensor->checkMeasureType(lMeasureTypes[lMeasureIndex])) {
                        float lValue = 0.0;
                        bool lSuccess = lSensor->measureValue(lMeasureTypes[lMeasureIndex], lValue);
                        if (lSuccess) {
                            logInfo(lSensor->logPrefix(), "OK: %f", lValue);
                            sCheckedSensors[lCounter] = true;
                        }
                    }
                }
            }
        }
    }
    if (lFinished) {
        logInfoP("============================");
        logInfoP("Sensor test finished");
        mSensorTestDelayTimer = 0;
    }
    else if (delayCheck(mSensorTestDuration, 60000)) {
        logInfoP("============================");
        logInfoP("Sensor test timeout");
        mSensorTestDelayTimer = 0;
    }
    else {
        mSensorTestDelayTimer = delayTimerInit();
    }
    if (mSensorTestDelayTimer == 0) {
        // output summary of checked sensors
        logIndentUp();
        for (uint8_t lCounter = 0; lCounter < mNumSensors; lCounter++) {
            Sensor* lSensor = mSensors[lCounter];
            if (sCheckedSensors[lCounter]) 
                if (lSensor->getSensorState() == SensorState::Off)
                    logInfo(lSensor->logPrefix(), "Sensor not connected");
                else
                    logInfo(lSensor->logPrefix(), "Sensor measurement OK");
            else
                logError(lSensor->logPrefix(), "Sensor measurement FAILED");
        }
        logIndentDown();
    }
}


