// #include "IncludeManager.h"
#ifdef SENSORMODULE
#include "SensorBME680.h"
#include "bsec/bme680/bme680.h"
#include "EepromManager.h"

#define STATE_SAVE_PERIOD  UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day
#define EEPROM_BME680_START_ADDRESS 0xC80

SensorBME680::SensorBME680(uint16_t iMeasureTypes)
    : Sensor(iMeasureTypes, BME680_I2C_ADDR), Bsec()
{
    mEEPROM = new EepromManager(100, 5, sMagicWord);
}

SensorBME680::SensorBME680(uint16_t iMeasureTypes, uint8_t iAddress, bme680_delay_fptr_t iDelayCallback)
    : Sensor(iMeasureTypes, iAddress), Bsec(), mDelayCallback(iDelayCallback)
{
    mEEPROM = new EepromManager(100, 5, sMagicWord);
}

SensorBME680::SensorBME680(uint16_t iMeasureTypes, uint8_t iAddress, bme680_delay_fptr_t iDelayCallback, uint8_t iMagicKeyOffset)
    : Sensor(iMeasureTypes, iAddress), Bsec(), mDelayCallback(iDelayCallback)
{
    mEEPROM = new EepromManager(100, 5, sMagicWord);
    sMagicWord[0] ^= iMagicKeyOffset;
};

bsec_virtual_sensor_t SensorBME680::sensorList[] = {
    BSEC_OUTPUT_IAQ,
    // BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    // BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    // BSEC_OUTPUT_STABILIZATION_STATUS,
    // BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY
    // BSEC_OUTPUT_COMPENSATED_GAS,
    // BSEC_OUTPUT_GAS_PERCENTAGE
};

// from 33v_3s_4d example
uint8_t SensorBME680::bsec_config_iaq[454] = {
    0, 8, 4, 1, 61, 0, 0, 0, 0, 0, 0, 0, 174, 1, 0, 0, 48, 0, 1, 0, 0, 192, 168, 71, 64, 49, 119, 76, 0, 0, 225, 68, 137, 65, 0, 191, 205, 204, 204, 190, 0, 0, 64, 191, 225, 122, 148, 190, 0, 0, 0, 0, 216, 85, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 28, 0, 2, 0, 0, 244, 1, 225, 0, 25, 0, 0, 128, 64, 0, 0, 32, 65, 144, 1, 0, 0, 112, 65, 0, 0, 0, 63, 16, 0, 3, 0, 10, 215, 163, 60, 10, 215, 35, 59, 10, 215, 35, 59, 9, 0, 5, 0, 0, 0, 0, 0, 1, 88, 0, 9, 0, 229, 208, 34, 62, 0, 0, 0, 0, 0, 0, 0, 0, 218, 27, 156, 62, 225, 11, 67, 64, 0, 0, 160, 64, 0, 0, 0, 0, 0, 0, 0, 0, 94, 75, 72, 189, 93, 254, 159, 64, 66, 62, 160, 191, 0, 0, 0, 0, 0, 0, 0, 0, 33, 31, 180, 190, 138, 176, 97, 64, 65, 241, 99, 190, 0, 0, 0, 0, 0, 0, 0, 0, 167, 121, 71, 61, 165, 189, 41, 192, 184, 30, 189, 64, 12, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 229, 0, 254, 0, 2, 1, 5, 48, 117, 100, 0, 44, 1, 112, 23, 151, 7, 132, 3, 197, 0, 92, 4, 144, 1, 64, 1, 64, 1, 144, 1, 48, 117, 48, 117, 48, 117, 48, 117, 100, 0, 100, 0, 100, 0, 48, 117, 48, 117, 48, 117, 100, 0, 100, 0, 48, 117, 48, 117, 100, 0, 100, 0, 100, 0, 100, 0, 48, 117, 48, 117, 48, 117, 100, 0, 100, 0, 100, 0, 48, 117, 48, 117, 100, 0, 100, 0, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 255, 255, 255, 255, 255, 255, 255, 255, 220, 5, 220, 5, 220, 5, 255, 255, 255, 255, 255, 255, 220, 5, 220, 5, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 44, 1, 0, 0, 0, 0, 237, 52, 0, 0
//  4, 7, 4, 1, 61, 0, 0, 0, 0, 0, 0, 0, 174, 1, 0, 0, 48, 0, 1, 0, 0, 192, 168, 71, 64, 49, 119, 76, 0, 0, 225, 68, 137, 65, 0,  63, 205, 204, 204,  62, 0, 0, 64,  63, 205, 204, 204,  62, 0, 0, 0, 0, 216, 85, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 28, 0, 2, 0, 0, 244, 1, 225, 0, 25, 0, 0, 128, 64, 0, 0, 32, 65, 144, 1, 0, 0, 112, 65, 0, 0, 0, 63, 16, 0, 3, 0, 10, 215, 163, 60, 10, 215, 35, 59, 10, 215, 35, 59, 9, 0, 5, 0, 0, 0, 0, 0, 1, 88, 0, 9, 0, 229, 208, 34, 62, 0, 0, 0, 0, 0, 0, 0, 0, 218, 27, 156, 62, 225, 11, 67, 64, 0, 0, 160, 64, 0, 0, 0, 0, 0, 0, 0, 0, 94, 75, 72, 189, 93, 254, 159, 64, 66, 62, 160, 191, 0, 0, 0, 0, 0, 0, 0, 0, 33, 31, 180, 190, 138, 176, 97, 64, 65, 241, 99, 190, 0, 0, 0, 0, 0, 0, 0, 0, 167, 121, 71, 61, 165, 189, 41, 192, 184, 30, 189, 64, 12, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 229, 0, 254, 0, 2, 1, 5, 48, 117, 100, 0, 44, 1, 112, 23, 151, 7, 132, 3, 197, 0, 92, 4, 144, 1, 64, 1, 64, 1, 144, 1, 48, 117, 48, 117, 48, 117, 48, 117, 100, 0, 100, 0, 100, 0, 48, 117, 48, 117, 48, 117, 100, 0, 100, 0, 48, 117, 48, 117, 100, 0, 100, 0, 100, 0, 100, 0, 48, 117, 48, 117, 48, 117, 100, 0, 100, 0, 100, 0, 48, 117, 48, 117, 100, 0, 100, 0, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 255, 255, 255, 255, 255, 255, 255, 255, 220, 5, 220, 5, 220, 5, 255, 255, 255, 255, 255, 255, 220, 5, 220, 5, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 44, 1, 0, 0, 0, 0, 138, 80, 0, 0
//  3, 7, 4, 1, 61, 0, 0, 0, 0, 0, 0, 0, 174, 1, 0, 0, 48, 0, 1, 0, 0, 168,  19, 73, 64, 49, 119, 76, 0, 0, 225, 68, 137, 65, 0, 63, 205, 204, 204, 62, 0, 0, 64, 63, 205, 204, 204, 62, 0, 0, 0, 0,   0, 80, 5,  95, 0, 0, 0, 0, 0, 0, 0, 0, 28, 0, 2, 0, 0, 244, 1, 225, 0, 25, 0, 0, 128, 64, 0, 0, 32, 65, 144, 1, 0, 0, 112, 65, 0, 0, 0, 63, 16, 0, 3, 0, 10, 215, 163, 60, 10, 215, 35, 59, 10, 215, 35, 59, 9, 0, 5, 0, 0, 0, 0, 0, 1, 88, 0, 9, 0, 229, 208, 34, 62, 0, 0, 0, 0, 0, 0, 0, 0, 218, 27, 156, 62, 225, 11, 67, 64, 0, 0, 160, 64, 0, 0, 0, 0, 0, 0, 0, 0, 94, 75, 72, 189, 93, 254, 159, 64, 66, 62, 160, 191, 0, 0, 0, 0, 0, 0, 0, 0, 33, 31, 180, 190, 138, 176, 97, 64, 65, 241, 99, 190, 0, 0, 0, 0, 0, 0, 0, 0, 167, 121, 71, 61, 165, 189, 41, 192, 184, 30, 189, 64, 12, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 229, 0, 254, 0, 2, 1, 5, 48, 117, 100, 0, 44, 1, 112, 23, 151, 7, 132, 3, 197, 0, 92, 4, 144, 1, 64, 1, 64, 1, 144, 1, 48, 117, 48, 117, 48, 117, 48, 117, 100, 0, 100, 0, 100, 0, 48, 117, 48, 117, 48, 117, 100, 0, 100, 0, 48, 117, 48, 117, 100, 0, 100, 0, 100, 0, 100, 0, 48, 117, 48, 117, 48, 117, 100, 0, 100, 0, 100, 0, 48, 117, 48, 117, 100, 0, 100, 0, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 44, 1, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 8, 7, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 112, 23, 255, 255, 255, 255, 255, 255, 255, 255, 220, 5, 220, 5, 220, 5, 255, 255, 255, 255, 255, 255, 220, 5, 220, 5, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 44, 1, 0, 0, 0, 0, 222, 38, 0, 0
};

// EEPROM memory start id
uint8_t SensorBME680::sMagicWord[] = {0xCA, 0xFE, 0x3D, 0x76};

uint8_t SensorBME680::getSensorClass()
{
    return SENS_BME680;
}

void SensorBME680::delayCallback(bme680_delay_fptr_t iDelayCallback)
{
    mDelayCallback = iDelayCallback;
}

void SensorBME680::setMagicKeyOffset(uint8_t iMagicKeyOffset)
{
    sMagicWord[0] ^= iMagicKeyOffset;
}

/*!
* @brief implements a state engine to restart the sensor without delays
*/
void SensorBME680::sensorLoopInternal() {
    bool lResult = false;
    switch (gSensorState)
    {
        case Wakeup:
            if (pSensorStateDelay == 0 || delayCheck(pSensorStateDelay, 1000))
            {
                Sensor::sensorLoopInternal();
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
                    Bsec::run();
                printResult(lResult);
            }
            break;
        case Finalize:
            if (delayCheck(pSensorStateDelay, 100))
            {
                // as long as there are no new values, sensor is not yet ready
                gSensorState = Bsec::run() ? Running : Finalize;
                pSensorStateDelay = millis();
            }
            break;
        case Running:
            if (Bsec::run())
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

float SensorBME680::measureValue(MeasureType iMeasureType) {
    switch (iMeasureType)
    {
        case Temperature:
            // hardware calibration
            return Bsec::temperature;
            break;
        case Humidity:
            return Bsec::humidity;
            break;
        case Pressure:
            return Bsec::pressure;
            break;
        case Voc:
            return Bsec::iaq;
            break;
        case Co2Calc:
            return Bsec::co2Equivalent;
            break;
        case Accuracy:
            return (Bsec::iaqAccuracy / 3.0f) * 100.0f ;
            break;
        default:
            break;
    }
    return NO_NUM;
}

bool SensorBME680::begin() {
    printDebug("Starting sensor BME680... ");
    bool lResult = Sensor::begin();
    if (lResult) {
        Bsec::begin(gAddress, gWire, mDelayCallback);
        lResult = checkIaqSensorStatus();
    }
    if (lResult) {
        Bsec::setConfig(bsec_config_iaq);
        lResult = checkIaqSensorStatus();
    }
    if (lResult) {
        Bsec::updateSubscription(sensorList, sizeof(sensorList), BSEC_SAMPLE_RATE_LP);
        lResult = checkIaqSensorStatus();
    }
    Bsec::setTemperatureOffset(-gTempOffset);
    return lResult;
}

uint8_t SensorBME680::getI2cSpeed()
{
    return 4; // n * 100kHz
}

bool SensorBME680::checkIaqSensorStatus(void)
{
    if (Bsec::status < BSEC_OK)
    {
        printDebug("BSEC error code : %d\n", Bsec::status);
        return false;
        // fatalError(-iaqSensor.status, "BSEC error code");
    }
    else if (Bsec::status > BSEC_OK)
        printDebug("BSEC warning code : %d\n", Bsec::status);

    if (Bsec::bme680Status < BME680_OK)
    {
        printDebug("BME680 error code : %d\n", Bsec::bme680Status);
        return false;
        // fatalError(-iaqSensor.bme680Status, "BME680 error code");
    }
    else if (Bsec::bme680Status > BME680_OK)
        printDebug("BME680 warning code : %d\n", Bsec::bme680Status);

    return true;
}

void SensorBME680::sensorLoadState()
{
    uint8_t buffer[144]; //[BSEC_MAX_STATE_BLOB_SIZE]; // header-size + blob
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
        Bsec::setState(buffer + 4);
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
void SensorBME680::sensorSaveState()
{
    // buffer gets freed inside knx object after saved
    uint8_t buffer[144] = {0}; //[BSEC_MAX_STATE_BLOB_SIZE];
    for (uint8_t lIndex = 0; lIndex < 4; lIndex++)
        buffer[lIndex] = sMagicWord[lIndex];

    Bsec::getState(buffer + 4);
    bool lCheck = checkIaqSensorStatus();
    if (lCheck) { 
        printDebug("Writing BME680 state to EEPROM\n");

        for (uint8_t lCount = 0; lCount < 144; lCount += 16) {
            mEEPROM->beginPage(EEPROM_BME680_START_ADDRESS + lCount);
            gWire.write(buffer + lCount, 16);
            mEEPROM->endPage();
            printHEX("--> ", buffer + lCount, 16);
        }
    }
}

void SensorBME680::sensorUpdateState(void)
{
    // we save the sensor state each time accuracy is raised 
    // and 4 times a day if accuracy is 3 (this might be senseless, if we save with each restart)
    if ((Bsec::iaqAccuracy > mLastAccuracy && mLastAccuracy == 2) || (Bsec::iaqAccuracy >= 3 && delayCheck(stateUpdateTimer, STATE_SAVE_PERIOD)))
    {
        sensorSaveState();
        stateUpdateTimer = millis();
    }
    mLastAccuracy = Bsec::iaqAccuracy;
}

bool SensorBME680::prepareTemperatureOffset(float iTemp)
{
    gTempOffset = iTemp;
    return true;
}
#endif
