// #include "IncludeManager.h"
#ifdef SENSORMODULE
#include "Wire.h"
#include "SensorSGP30.h"
#include "EepromManager.h"

#define STATE_SAVE_PERIOD  UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day
#define EEPROM_BME680_START_ADDRESS 0xC80

SensorSGP30::SensorSGP30(uint16_t iMeasureTypes)
    : Sensor(iMeasureTypes, SGP30_I2C_ADDR)
{
    mEEPROM = new EepromManager(100, 5, sMagicWord);
}

SensorSGP30::SensorSGP30(uint16_t iMeasureTypes, uint8_t iAddress)
    : Sensor(iMeasureTypes, iAddress)
{
    mEEPROM = new EepromManager(100, 5, sMagicWord);
}

SensorSGP30::SensorSGP30(uint16_t iMeasureTypes, uint8_t iAddress, uint8_t iMagicKeyOffset)
    : Sensor(iMeasureTypes, iAddress)
{
    mEEPROM = new EepromManager(100, 5, sMagicWord);
    sMagicWord[0] ^= iMagicKeyOffset;
};

// EEPROM memory start id
uint8_t SensorSGP30::sMagicWord[] = {0xCA, 0xFE, 0x3D, 0x76};

uint8_t SensorSGP30::getSensorClass()
{
    return SENS_SGP30;
}

void SensorSGP30::setMagicKeyOffset(uint8_t iMagicKeyOffset)
{
    sMagicWord[0] ^= iMagicKeyOffset;
}

/*!
* @brief implements a state engine to restart the sensor without delays
*/
void SensorSGP30::sensorLoopInternal()
{
    bool lResult = false;
    switch (gSensorState)
    {
        case Wakeup:
            if (pSensorStateDelay == 0 || delayCheck(pSensorStateDelay, 1000))
            {
                pSensorStateDelay = millis();
                printDebug("Restarting Sensor BME680... ");
                // Bsec::setConfig(bsec_config_iaq);
                lResult = checkIaqSensorStatus();
                if (lResult)
                {
                    // Bsec::updateSubscription(sensorList, sizeof(sensorList) / sizeof(bsec_virtual_sensor_t), BSEC_SAMPLE_RATE_LP);
                    lResult = checkIaqSensorStatus();
                }
                gSensorState = lResult ? Calibrate : Off;
            }
            break;
        case Calibrate:
            if (delayCheck(pSensorStateDelay, 100))
            {
                sensorLoadState();
                lResult = checkIaqSensorStatus();
                gSensorState = lResult ? Finalize : Off;
                pSensorStateDelay = millis();
                if (lResult)
                    // Bsec::run();
                printResult(lResult);
            }
            break;
        case Finalize:
            if (delayCheck(pSensorStateDelay, 100))
            {
                // as long as there are no new values, sensor is not yet ready
                // gSensorState = Bsec::run() ? Running : Finalize;
                pSensorStateDelay = millis();
            }
            break;
        case Running:
            if (true) // Bsec::run())
            {
                sensorUpdateState();
                checkIaqSensorStatus();
            }
            break;
        default:
            pSensorStateDelay = millis();
            break;
    }
}

float SensorSGP30::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
        case Temperature:
            // hardware calibration
            return 0.0; // Bsec::temperature - 3.5f;
            break;
        case Humidity:
            return 0.0; // Bsec::humidity;
            break;
        case Pressure:
            return 0.0; // Bsec::pressure;
            break;
        case Voc:
            return 0.0; // Bsec::iaq;
            break;
        case Co2Calc:
            return 0.0; // Bsec::co2Equivalent;
            break;
        case Accuracy:
            return 0.0; // (Bsec::iaqAccuracy / 3.0f) * 100.0f ;
            break;
        default:
            break;
    }
    return -1000.0f;
}

bool SensorSGP30::begin()
{
    printDebug("Starting sensor BME680... ");
    bool lResult = checkIaqSensorStatus();
    if (lResult) {
        lResult = checkIaqSensorStatus();
    }
    if (lResult) {
        lResult = checkIaqSensorStatus();
    }
    if (lResult) {
        sensorLoadState();
        lResult = checkIaqSensorStatus();
        if (!lResult)
            restartSensor();
    }
    if (lResult)
        lResult = Sensor::begin();
    gSensorState = Finalize;
    pSensorStateDelay = millis();
    printResult(lResult);
    return lResult;
}

uint8_t SensorSGP30::getI2cSpeed()
{
    return 4; // n * 100kHz
}

bool SensorSGP30::checkIaqSensorStatus(void)
{
    // if (Bsec::status < BSEC_OK)
    // {
    //     printDebug("BSEC error code : %d\n", Bsec::status);
    //     return false;
    //     // fatalError(-iaqSensor.status, "BSEC error code");
    // }
    // else if (Bsec::status > BSEC_OK)
    //     printDebug("BSEC warning code : %d\n", Bsec::status);

    // if (Bsec::bme680Status < BME680_OK)
    // {
    //     printDebug("BME680 error code : %d\n", Bsec::bme680Status);
    //     return false;
    //     // fatalError(-iaqSensor.bme680Status, "BME680 error code");
    // }
    // else if (Bsec::bme680Status > BME680_OK)
    //     printDebug("BME680 warning code : %d\n", Bsec::bme680Status);

    return true;
}

void SensorSGP30::sensorLoadState()
{
    uint8_t buffer[144]; //[BSEC_MAX_STATE_BLOB_SIZE];
    // Existing state in EEPROM
    printDebug("Reading BME680 state from EEPROM\n");
    mEEPROM->prepareRead(EEPROM_BME680_START_ADDRESS, 144);
    if (gWire.available()) gWire.readBytes(buffer, 144);

    for (uint8_t i = 0; i < 144; i+=16)
        printHEX("<-- ", buffer + i, 16);
    bool lCheck = true;
    for (uint8_t lIndex = 0; lIndex < 4 && lCheck; lIndex++)
        lCheck = (buffer[lIndex] == sMagicWord[lIndex]);
    
    if (lCheck) {
        // Bsec::setState(buffer + 4);
        bool lResult = checkIaqSensorStatus();
        if (lResult) {
            printDebug("BME680 was successfully calibrated from EEPROM\n");
        } else {
            // if sensor state was not correctly loaded, we delete EEPROM data 
            mEEPROM->beginPage(EEPROM_BME680_START_ADDRESS);
            mEEPROM->write4Bytes(sMagicWord, 1); // this is correct, it deletes the last 3 bytes of magic word
            mEEPROM->endPage();
            printDebug("*** BME680 calibration from EEPROM failed! ***\n");
        }
    } else {
        printDebug("BME680 calibration data in EEPROM has wrong ID and will be deleted!\n");
    }
}

// We store sensor data in EEPROM stating at page 100 (100 * 32 = 3200 = 0xC80).
// We write in 16 Byte chunks, a maximum of 139 bytes, means 139/16 = 9 chunks (which is in fact 144 Bytes).
// Timing is 9 * 5 ms = 45 ms write time
void SensorSGP30::sensorSaveState()
{
    // buffer gets freed inside knx object after saved
    uint8_t buffer[144] = {0}; //[BSEC_MAX_STATE_BLOB_SIZE];
    for (uint8_t lIndex = 0; lIndex < 4; lIndex++)
        buffer[lIndex] = sMagicWord[lIndex];

    // Bsec::getState(buffer + 4);
    bool lCheck = checkIaqSensorStatus();
    if (lCheck) { 
    printDebug("Writing BME680 state to EEPROM\n");

    for (uint8_t lCount = 0; lCount < 144; lCount += 16)
        {
            mEEPROM->beginPage(EEPROM_BME680_START_ADDRESS + lCount);
            gWire.write(buffer + lCount, 16);
            mEEPROM->endPage();
            printHEX("--> ", buffer + lCount, 16);
        }
    }
}

void SensorSGP30::sensorUpdateState(void)
{
    // we save the sensor state each time accuracy is raised 
    // and 4 times a day if accuracy is 3 (this might be senseless, if we save with each restart)
    // if ((Bsec::iaqAccuracy > mLastAccuracy && mLastAccuracy == 2) || (Bsec::iaqAccuracy >= 3 && delayCheck(stateUpdateTimer, STATE_SAVE_PERIOD)))
    // {
    //     sensorSaveState();
    //     stateUpdateTimer = millis();
    // }
    // mLastAccuracy = Bsec::iaqAccuracy;
}
#endif
