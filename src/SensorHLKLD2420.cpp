#ifdef PMMODULE
    #ifdef HF_SERIAL
        #include "SensorHLKLD2420.h"
        #include <Arduino.h>
        #include <OpenKNX.h>
        #include <Wire.h>

SensorHLKLD2420::SensorHLKLD2420(uint16_t iMeasureTypes, TwoWire *iWire)
    : SensorHLKLD2420(iMeasureTypes, &Wire, 0) {};

SensorHLKLD2420::SensorHLKLD2420(uint16_t iMeasureTypes, TwoWire *iWire, uint8_t iAddress)
    : Sensor(iMeasureTypes, &Wire, iAddress)
{
    pMeasureTypes |= Pres | Sensitivity | Distance;
};

std::string SensorHLKLD2420::logPrefix()
{
    return "Sensor<HLKLD2420>";
}

void SensorHLKLD2420::defaultSensorParameters(uint8_t iSensitivity, uint16_t iDelayTime, uint8_t iRangeGateMin, uint8_t iRangeGateMax)
{
    mSensitivity = iSensitivity;
    mDelayTime = iDelayTime;
    mRangeGateMin = iRangeGateMin;

    if (iRangeGateMax > iRangeGateMin)
        mRangeGateMax = iRangeGateMax;
}

void SensorHLKLD2420::writeSensitivity(int8_t iSensitivity)
{
    mSensitivity = iSensitivity;

    sendCalibrationData();
}

uint8_t SensorHLKLD2420::getSensorClass()
{
    return SENS_HLKLD2420;
}

void SensorHLKLD2420::sensorLoopInternal()
{
    switch (pSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
        case Calibrate:
            uartGetPacket();
            startupLoop();

            if (mHfSensorStartupState == START_FINISHED)
                pSensorState = Finalize;

            break;
        case Finalize:
            uartGetPacket();

            // close command mode and resume normal operation
            sendCommand(CMD_CLOSE_COMMAND_MODE);

            pSensorState = Running;
            break;
        case Running:
            uartGetPacket();
            break;
        default:
            pSensorStateDelay = millis();
            break;
    }
}

// state machine handles startup behavior and calibration of HF sensor
void SensorHLKLD2420::startupLoop()
{
    switch (mHfSensorStartupState)
    {
        case START_INIT:
            if (lastDetectedRange > NO_NUM)
            {
                pSensorStateDelay = millis();
                mHfSensorStartupState = START_SENSOR_ACTIVE;

                sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE_LENGTH);
            }
            break;
        case START_SENSOR_ACTIVE:
            // Communication is established, we wait for version info from Sensor
            if (!moduleVersion.empty())
            {
                pSensorStateDelay = millis();
                mHfSensorStartupState = START_VERSION_RECEIVED;
            }
            else if (delayCheck(pSensorStateDelay, 1000))
            {
                pSensorStateDelay = millis();
                sendCommand(CMD_READ_VERSION);
            }
            break;
        case START_VERSION_RECEIVED:
            // We got version, we wait for read 1 done
            if (storedDistanceMin > NO_NUM)
            {
                pSensorStateDelay = millis();
                mHfSensorStartupState = START_READ1_DONE;
            }
            else if (delayCheck(pSensorStateDelay, 2000))
            {
                pSensorStateDelay = millis();
                sendCommand(CMD_READ_MODULE_CONFIG, PARAM_READ_DISTANCE_TRIGGER, PARAM_READ_DISTANCE_TRIGGER_LENGTH);
            }
            break;
        case START_READ1_DONE:
            // Read 1 is done, we wait for read 2 done
            if (storedDelayTime > NO_NUM)
            {
                pSensorStateDelay = millis();
                mHfSensorStartupState = START_READ2_DONE;
            }
            else if (delayCheck(pSensorStateDelay, 2000))
            {
                pSensorStateDelay = millis();
                sendCommand(CMD_READ_MODULE_CONFIG, PARAM_READ_DELAY_MAINTAIN, PARAM_READ_DELAY_MAINTAIN_LENGTH);
            }
            break;
        case START_READ2_DONE:
            // All done, close command mode again
            if (delayCheck(pSensorStateDelay, 50))
            {
                pSensorStateDelay = millis();

                if (calibrationCompleted)
                {
                    // skip calibration if valid calibration values have been read from flash
                    mHfSensorStartupState = START_FINISHED;

                    sendCalibrationData();
                }
                else
                {
                    mHfSensorStartupState = START_CALIBRATING;

                    resetRawDataRecording();
                    sendCommand(CMD_RAW_DATA_MODE, PARAM_RAW_DATA_MODE, PARAM_RAW_DATA_MODE_LENGTH);
                }

                sendCommand(CMD_CLOSE_COMMAND_MODE);
            }
            break;
        case START_CALIBRATING:
            if (calibrationCompleted)
            {
                mHfSensorStartupState = START_FINISHED;
            }
            else if (delayCheck(pSensorStateDelay, 40000))
            {
                // if not complete after 40 sec., restart calibration
                mHfSensorStartupState = START_READ2_DONE;
            }
            break;
    }
}

void SensorHLKLD2420::forceCalibration()
{
    logDebugP("Force sensor calibration");

    sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE_LENGTH);
    delay(100);

    pSensorState = Calibrate;
    mHfSensorStartupState = START_CALIBRATING;
    pSensorStateDelay = millis();

    resetRawDataRecording();
    sendCommand(CMD_RAW_DATA_MODE, PARAM_RAW_DATA_MODE, PARAM_RAW_DATA_MODE_LENGTH);
    sendCommand(CMD_CLOSE_COMMAND_MODE);
}

void SensorHLKLD2420::uartGetPacket()
{
    uint8_t rxByte;
    long lastDataReceived = millis();
    switch (mPacketState)
    {
        case GET_SYNC_STATE:
            // wait for a valid header
            while (
                mPacketState == GET_SYNC_STATE &&
                HF_SERIAL.available() > 0 && HF_SERIAL.readBytes(&rxByte, 1) == 1)
            {
                lastDataReceived = millis();

                mBuffer[mBufferIndex] = rxByte;
                mBufferIndex++;

                if (BUFFER_LENGTH == HEADER_FOOTER_SIZE)
                {
                    /*logTraceP("Header:");
                    logIndentUp();
                    logHexTraceP(mBuffer, BUFFER_LENGTH);
                    logIndentDown();*/

                    if (memcmp(mBuffer, HEADER_ON, HEADER_FOOTER_SIZE) == 0)
                    {
                        mPacketType = ON;
                        mPacketState = GET_PACKET_DATA;
                        break;
                    }
                    if (memcmp(mBuffer, HEADER_OFF, HEADER_FOOTER_SIZE) == 0)
                    {
                        mPacketType = OFF;
                        mPacketState = GET_PACKET_DATA;
                        break;
                    }
                    if (memcmp(mBuffer, HEADER_COMMAND, HEADER_FOOTER_SIZE) == 0)
                    {
                        mPacketType = COMMAND_RESPONSE;
                        mPacketState = GET_PACKET_DATA;
                        break;
                    }
                    if (memcmp(mBuffer, HEADER_RAW_DATA, HEADER_FOOTER_SIZE) == 0)
                    {
                        mPacketType = RAW_DATA;
                        mPacketState = GET_PACKET_DATA;
                        break;
                    }
                }

                // if the current buffer is the expected size of the header,
                // but we did not find a valid header yet, we remove the first element
                if (BUFFER_LENGTH == HEADER_FOOTER_SIZE)
                {
                    memmove(mBuffer, mBuffer + 1, BUFFER_LENGTH - 1);
                    mBufferIndex -= 1;
                }
            }

            if (delayCheck(lastDataReceived, 100))
            {
                // no data received for 100 millisec., cancel header sync
                mBufferIndex = 0;
                mPacketState = GET_SYNC_STATE;
                break;
            }

            break;
        case GET_PACKET_DATA:
            // add data till valid footer received
            while (
                mPacketState == GET_PACKET_DATA &&
                HF_SERIAL.available() > 0 && HF_SERIAL.readBytes(&rxByte, 1) == 1)
            {
                lastDataReceived = millis();

                mBuffer[mBufferIndex] = rxByte;
                mBufferIndex++;

                if (mPacketType == COMMAND_RESPONSE && memcmp(mBuffer + mBufferIndex - HEADER_FOOTER_SIZE, FOOTER, HEADER_FOOTER_SIZE) == 0 ||
                    mPacketType == RAW_DATA && memcmp(mBuffer + mBufferIndex - HEADER_FOOTER_SIZE, FOOTER_RAW_DATA, HEADER_FOOTER_SIZE) == 0)
                {
                    mPacketState = PROCESS_PACKET_STATE;
                    break;
                }

                if ((mPacketType == OFF || mPacketType == ON && BUFFER_LENGTH > 10) &&
                    mBuffer[mBufferIndex - 2] == 13 && mBuffer[mBufferIndex - 1] == 10)
                { // new line
                    mPacketState = PROCESS_PACKET_STATE;
                    break;
                }
            }

            if (delayCheck(lastDataReceived, 100))
            {
                // no data received for 100 millisec., cancel get packet data
                mBufferIndex = 0;
                mPacketState = GET_SYNC_STATE;
                break;
            }

            break;
        case PROCESS_PACKET_STATE:
            getSensorData();
            mBufferIndex = 0;
            mPacketState = GET_SYNC_STATE;
            break;
    }
}

int SensorHLKLD2420::bytesToInt(byte byte0, byte byte1, byte byte2, byte byte3)
{
    return int((unsigned char)(byte3) << 24 |
               (unsigned char)(byte2) << 16 |
               (unsigned char)(byte1) << 8 |
               (unsigned char)(byte0));
}

float SensorHLKLD2420::rawToDb(int rawValue)
{
    return float(10 * log10(rawValue));
}

int SensorHLKLD2420::dBToRaw(float dbValue)
{
    if (dbValue > maxDbValue)
        dbValue = maxDbValue;

    return int(pow(10, dbValue / 10));
}

void SensorHLKLD2420::rebootSensorSoft()
{
    // enter command mode to stop raw data transfer
    sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE_LENGTH);

    // reboot module to return to normal (not raw data) operation
    sendCommand(CMD_REBOOT_MODULE);
    delay(1000);

    restartStartupLoop();
}

void SensorHLKLD2420::rebootSensorHard()
{
    switchPower(false);
    delay(1000);

    switchPower(true);
    openknx.console.writeDiagenoseKo("HLK reb hard");
}

void SensorHLKLD2420::switchPower(bool on)
{
    logDebugP("Switch power on: %u", on);
    digitalWrite(HF_POWER_PIN, on ? HIGH : LOW);

    if (on)
    {
        delay(1000);
        restartStartupLoop();
    }
}

void SensorHLKLD2420::restartStartupLoop()
{
    moduleVersion = "";
    storedDistanceMin = NO_NUM;
    storedDistanceMax = NO_NUM;
    storedDelayTime = NO_NUM;
    pSensorState = Calibrate;
    mHfSensorStartupState = START_INIT;
}

void SensorHLKLD2420::resetRawDataRecording()
{
    for (uint8_t i = 0; i < 16; i++)
    {
        rawDataRangeTempSumDb[i] = 0;
        rawDataRangeTempSquareSumDb[i] = 0;
        rawDataRangeMaxDb[i] = 0;
    }

    rawDataLastRecordingReceived = millis();
    rawDataRecordingCount = 0;
    calibrationCompleted = false;

    if (calibrationTestRunOnly)
    {
        logDebugP("Start sensor calibration - test run only");
        openknx.console.writeDiagenoseKo("HLK calt start");
    }
    else
    {
        logDebugP("Start sensor calibration");
        openknx.console.writeDiagenoseKo("HLK cal start");
    }
}

bool SensorHLKLD2420::getSensorData()
{
    bool result = false;

    std::string rangeString;
    float newDetectedRange;

    bool success;
    std::string successMessage;

    uint8_t payloadSize;

    int rangeMax[16] = {};
    int dopplerOffset;
    int rangeOffset;
    int rangeValue;

    // if (calibrationOnOffTimer == 0 || delayCheck(calibrationOnOffTimer, 15000))
    //     calibrationOnOffTimer = 0;
    // else
    //     return result;

    switch (mPacketType)
    {
        case ON:
        {
            /*logTraceP("Content ON:");
            logIndentUp();
            logHexTraceP(mBuffer, BUFFER_LENGTH);
            logIndentDown();*/

            // cut 10 bytes at the beginning: 4F 4E 0D 0A 52 61 6E 67 65 20 ("ON  RANGE ")
            // and 2 bytes at the end
            memmove(mBuffer, mBuffer + 10, BUFFER_LENGTH - 10);
            mBufferIndex -= 12;
            // exceptions not possible, we check if string consists of digits
            bool justDigits = true;
            for (uint8_t i = 0; i < BUFFER_LENGTH; i++)
                if (mBuffer[i] < 0x30 || mBuffer[i] > 0x39)
                {
                    justDigits = false;
                    break;
                }
            if (justDigits)
            {
                // mBuffer now holds the detection range decimal value as string
                rangeString = std::string(reinterpret_cast<const char *>(&mBuffer[0]), BUFFER_LENGTH);
                newDetectedRange = stoi(rangeString) / (float)100;

                if (lastDetectedRange != newDetectedRange)
                {
                    lastDetectedRange = newDetectedRange;
                    logDebugP("Presence detected, range: %.1f m", lastDetectedRange);
                }
            }
            else
            {
                logDebugP("Presence range contains non-digit-characters");
                logIndentUp();
                logHexTraceP(mBuffer, BUFFER_LENGTH);
                logIndentDown();
            }

            result = true;
            break;
        }
        case OFF:
            /*logTraceP("Content OFF:");
            logIndentUp();
            logHexTraceP(mBuffer, BUFFER_LENGTH);
            logIndentDown();*/

            if (lastDetectedRange != -1)
            {
                lastDetectedRange = -1;
                logDebugP("No presence detected");
            }

            result = true;
            break;

        case COMMAND_RESPONSE:
            payloadSize = mBuffer[4];

            // cut 6 bytes (4 header + 2 size) at the beginning and 4 bytes footer at the end
            memmove(mBuffer, mBuffer + 6, BUFFER_LENGTH - 6);
            mBufferIndex -= 10;

            if (BUFFER_LENGTH != payloadSize)
            {
                logDebugP("Invalid command reponse packet size: %d", BUFFER_LENGTH);
                break;
            }

            // we expect "01 00 00" after the command code for success
            success = mBuffer[1] == 1 && mBuffer[2] == 0 && mBuffer[3] == 0;
            successMessage = success ? "success" : "failed";

            switch (mBuffer[0])
            {
                case CMD_OPEN_COMMAND_MODE:
                    logDebugP("Received response: CMD_OPEN_COMMAND_MODE (%s)", successMessage.c_str());
                    result = true;
                    break;
                case CMD_CLOSE_COMMAND_MODE:
                    logDebugP("Received response: CMD_CLOSE_COMMAND_MODE (%s)", successMessage.c_str());
                    result = true;
                    break;
                case CMD_READ_VERSION:
                    logDebugP("Received response: CMD_READ_VERSION (%s)", successMessage.c_str());

                    if (!success)
                        break;

                    // cut 6 bytes at the beginning: 00 01 00 00 06 00
                    memmove(mBuffer, mBuffer + 6, BUFFER_LENGTH - 6);
                    mBufferIndex -= 6;

                    // mBuffer now holds the version value as string
                    moduleVersion = std::string(reinterpret_cast<const char *>(&mBuffer[0]), BUFFER_LENGTH);
                    logDebugP("Module version: %s", moduleVersion.c_str());
                    result = true;
                    break;
                case CMD_READ_MODULE_CONFIG:
                    logDebugP("Received response: CMD_READ_MODULE_CONFIG (%s)", successMessage.c_str());

                    if (!success)
                        break;

                    // We only support 2 types of read requests:
                    // - Read both distances and all trigger thresholds (= in total 18)
                    // - Read the delay and all metain thresholds (= in total 17)
                    // This way, based on the returned amount of data we can recognize which read request was fullfilled.

                    // cut 4 bytes at the beginning: 08 01 00 00
                    memmove(mBuffer, mBuffer + 4, BUFFER_LENGTH - 4);
                    mBufferIndex -= 4;

                    if (payloadSize == 0x4C)
                    {
                        // 76 - 4 = 72 bytes (18 x 4 bytes)
                        // - Read both distances and all trigger thresholds (= in total 18)

                        logDebugP("Read config part 1 from sensor:");
                        logIndentUp();

                        storedDistanceMin = bytesToInt(mBuffer[0], mBuffer[1], mBuffer[2], mBuffer[3]);
                        logDebugP("minDistance: %d", storedDistanceMin);

                        storedDistanceMax = bytesToInt(mBuffer[4], mBuffer[5], mBuffer[6], mBuffer[7]);
                        logDebugP("maxDistance: %d", storedDistanceMax);

                        logDebugP("triggerThreshold:");
                        logIndentUp();
                        for (uint8_t i = 0; i < 16; i++)
                        {
                            storedTriggerThreshold[i] = bytesToInt(mBuffer[i * 4 + 8], mBuffer[i * 4 + 9], mBuffer[i * 4 + 10], mBuffer[i * 4 + 11]);
                            logDebugP("Gate %i: %.2f", i, rawToDb(storedTriggerThreshold[i]));
                        }
                        logIndentDown();

                        logIndentDown();
                    }
                    else if (payloadSize == 0x48)
                    {
                        // 72 - 4 = 68 bytes (17 x 4 bytes)
                        // - Read the delay and all metain thresholds (= in total 17)

                        logDebugP("Read config part 2 from sensor:");
                        logIndentUp();

                        storedDelayTime = bytesToInt(mBuffer[0], mBuffer[1], mBuffer[2], mBuffer[3]);
                        logDebugP("delayTime: %d", storedDelayTime);

                        logDebugP("holdThreshold:");
                        logIndentUp();
                        for (uint8_t i = 0; i < 16; i++)
                        {
                            storedHoldThreshold[i] = bytesToInt(mBuffer[i * 4 + 4], mBuffer[i * 4 + 5], mBuffer[i * 4 + 6], mBuffer[i * 4 + 7]);
                            logDebugP("Gate %i: %.2f", i, rawToDb(storedHoldThreshold[i]));
                        }
                        logIndentDown();

                        logIndentDown();
                    }
                    else
                    {
                        logDebugP("Unknown read response, payload size: %d", payloadSize);
                        break;
                    }

                    result = true;
                    break;
                case CMD_WRITE_MODULE_CONFIG:
                    logDebugP("Received response: CMD_WRITE_MODULE_CONFIG (%s)", successMessage.c_str());
                    result = true;
                    break;
                case CMD_RAW_DATA_MODE:
                    logDebugP("Received response: CMD_RAW_DATA_MODE (%s)", successMessage.c_str());
                    result = true;
                    break;
                default:
                    logDebugP("Unknown response code: %d (%s)", mBuffer[0], successMessage.c_str());
                    break;
            }

            break;

        case RAW_DATA:
            // cut 4 bytes at the beginning and 4 bytes footer at the end
            memmove(mBuffer, mBuffer + 4, BUFFER_LENGTH - 4);
            mBufferIndex -= 8;

            for (uint8_t i = 0; i < 20; i++)
            {
                dopplerOffset = i * 64;

                for (uint8_t j = 0; j < 16; j++)
                {
                    rangeOffset = dopplerOffset + j * 4;
                    rangeValue = bytesToInt(mBuffer[rangeOffset], mBuffer[rangeOffset + 1], mBuffer[rangeOffset + 2], mBuffer[rangeOffset + 3]);
                    rangeMax[j] = max(rangeMax[j], rangeValue);
                }
            }

            logTraceP("Range values received (%d):", rawDataRecordingCount);
            logIndentUp();
            for (uint8_t i = 0; i < 16; i++)
            {
                logTraceP("Gate %i: %.2f", i, rawToDb(rangeMax[i]));
            }
            logIndentDown();

            // if more than 1 sec. past since last raw data value (should be 4-5/sec.),
            // something when wrong, start recording from scratch
            if (delayCheck(rawDataLastRecordingReceived, 1000))
            {
                logDebugP("Too long without new data, restarting calibration");
                resetRawDataRecording();
            }
            else if (rawDataRecordingCount < CALIBRATION_VALUE_COUNT)
            {
                float tempDb;
                for (uint8_t i = 0; i < 16; i++)
                {
                    tempDb = rawToDb(rangeMax[i]);
                    rawDataRangeTempSumDb[i] += tempDb;
                    rawDataRangeTempSquareSumDb[i] += pow(tempDb, 2);
                    rawDataRangeMaxDb[i] = max(rawDataRangeMaxDb[i] , tempDb);
                }
                rawDataRecordingCount++;

                rawDataLastRecordingReceived = millis();
            }

            if (!calibrationCompleted &&
                rawDataRecordingCount == CALIBRATION_VALUE_COUNT)
            {
                logIndentDown();

                // enter command mode to stop raw data transfer
                sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE_LENGTH);

                // reboot module to return to normal (not raw data) operation
                sendCommand(CMD_REBOOT_MODULE);
                delay(1000);

                if (calibrationTestRunOnly)
                {
                    openknx.console.writeDiagenoseKo("HLK calt done");
                    logDebugP("Sensor calibrarion test finished, data not stored");
                    calibrationTestRunOnly = false;
                    calibrationCompleted = true;
                    calibrationOnOffTimer = delayTimerInit();

                    for (uint8_t i = 0; i < 16; i++)
                        rawDataRangeDifferencesDb[i] = (rawDataRangeTempSumDb[i] / (float)rawDataRecordingCount) - rawDataRangeAverageDb[i];
                }
                else
                {
                    sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE_LENGTH);
                    delay(500);

                    for (uint8_t i = 0; i < 16; i++)
                    {
                        rawDataRangeAverageDb[i] = rawDataRangeTempSumDb[i] / (float)rawDataRecordingCount;
                        rawDataRangeDeviationDb[i] = sqrtf((rawDataRangeTempSquareSumDb[i] - (pow(rawDataRangeTempSumDb[i], 2) / rawDataRecordingCount)) / (rawDataRecordingCount - 1));
                        rawDataRangeVarianceDb[i] = rawDataRangeTempSquareSumDb[i] - (pow(rawDataRangeTempSumDb[i], 2) / rawDataRecordingCount);
                    }

                    openknx.console.writeDiagenoseKo("HLK cal done");
                    // will be followed by "HLK cal send"
                    openknx.console.writeDiagenoseKo("");
                    logDebugP("Sensor calibration finished");
                    calibrationCompleted = true;
                    calibrationOnOffTimer = delayTimerInit();

                    // persist new calibration data in flash
                    openknx.flash.save(true);

                    useFactoryDefaultThresholds = false;
                    sendCalibrationData();
                }
            }

            result = true;
            break;

        default:
            logDebugP("Unknown packet type: %d", mPacketType);
            break;
    }

    return result;
}

void SensorHLKLD2420::sendCalibrationData()
{
    logDebugP("Saving calibration data");
    logIndentUp();

    if (useFactoryDefaultThresholds)
    {
        triggerThresholdDb[0] = 47.78;
        triggerThresholdDb[1] = 44.77;
        triggerThresholdDb[2] = 34.77;
        triggerThresholdDb[3] = 33.01;
        triggerThresholdDb[4] = 26.99;
        triggerThresholdDb[5] = 26.02;
        triggerThresholdDb[6] = 26.02;
        triggerThresholdDb[7] = 24.77;
        triggerThresholdDb[8] = 24.77;
        triggerThresholdDb[9] = 24.77;
        triggerThresholdDb[10] = 24.77;
        triggerThresholdDb[11] = 23.98;
        triggerThresholdDb[12] = 23.98;
        triggerThresholdDb[13] = 23.01;
        triggerThresholdDb[14] = 23.01;
        triggerThresholdDb[15] = 23.01;

        holdThresholdDb[0] = 46.02;
        holdThresholdDb[1] = 43.01;
        holdThresholdDb[2] = 26.02;
        holdThresholdDb[3] = 24.77;
        holdThresholdDb[4] = 24.77;
        holdThresholdDb[5] = 23.01;
        holdThresholdDb[6] = 23.01;
        holdThresholdDb[7] = 21.76;
        holdThresholdDb[8] = 21.76;
        holdThresholdDb[9] = 20.00;
        holdThresholdDb[10] = 20.00;
        holdThresholdDb[11] = 20.00;
        holdThresholdDb[12] = 20.00;
        holdThresholdDb[13] = 20.00;
        holdThresholdDb[14] = 20.00;
        holdThresholdDb[15] = 20.00;

        logDebugP("Factory default thresholds used");
    }
    else
    {
        logTraceP("rawDataRangeAverage:");
        logIndentUp();

        for (uint8_t i = 0; i < 16; i++)
            logTraceP("Gate %i:  %.2f", i, rawDataRangeAverageDb[i]);

        logIndentDown();

        bool shouldUseCustomOffsets = useCustomOffsets();

        int8_t lSensitivity = mSensitivity > 0 && mSensitivity <= 10 ? mSensitivity : SENSITIVITY_DEFAULT;
        triggerOffsetDb = 6 / log10(lSensitivity + 1);

        // calculate trigger thresholds
        for (uint8_t i = 0; i < 16; i++)
        {
            if (shouldUseCustomOffsets)
                triggerThresholdDb[i] = rawDataRangeAverageDb[i] + customeTriggerOffsetDb[i];
            else
                triggerThresholdDb[i] = rawDataRangeAverageDb[i] + triggerOffsetDb;
        }

        holdOffsetDb = 3 * (1 / log10(lSensitivity + 1)) - 1.5;

        // calculate hold thresholds
        for (uint8_t i = 0; i < 16; i++)
        {
            if (shouldUseCustomOffsets)
                holdThresholdDb[i] = rawDataRangeAverageDb[i] + customHoldOffsetDb[i];
            else
                holdThresholdDb[i] = rawDataRangeAverageDb[i] + holdOffsetDb;
        }

        logDebugP("Sensitivity used:");
        logIndentUp();
        if (shouldUseCustomOffsets)
            logDebugP("Sensitivity ignored, custom offsets used");
        else
        {
            logDebugP("User sensitivity setting: %d", mSensitivity);
            logDebugP("Calculated trigger offset: %.2f", triggerOffsetDb);
            logDebugP("Calculated hold offset: %.2f", holdOffsetDb);
        }
        logIndentDown();
    }

    bool storedCurrentDistanceTime =
        (storedDistanceMin == mRangeGateMin) &&
        (storedDistanceMax == mRangeGateMax) &&
        (storedDelayTime == mDelayTime);

    bool storedCurrentCalibration = true;
    for (uint8_t i = 0; i < 16; i++)
    {
        if ((storedTriggerThreshold[i] != dBToRaw(triggerThresholdDb[i])) ||
            (storedHoldThreshold[i] != dBToRaw(holdThresholdDb[i])))
        {
            storedCurrentCalibration = false;
            break;
        }
    }

    logDebugP("Write config to sensor:");
    logIndentUp();

    uint8_t param[48];

    if (storedCurrentDistanceTime)
        logDebugP("Skip writing distance/time to sensor as data is current");
    else
    {
        logDebugP("Range gate min.: %d", mRangeGateMin);
        logDebugP("Range gate max.: %d", mRangeGateMax);
        logDebugP("Delay time: %d", mDelayTime);

        // write range gate min./max. and delay time, for each:
        // first 2 bytes parameter offset, then 4 bytes value

        param[0] = OFFSET_PARAM_RANGE_GATE_MIN;
        param[1] = 0;
        param[2] = mRangeGateMin;
        param[3] = 0;
        param[4] = 0;
        param[5] = 0;
        param[6] = OFFSET_PARAM_RANGE_GATE_MAX;
        param[7] = 0;
        param[8] = mRangeGateMax;
        param[9] = 0;
        param[10] = 0;
        param[11] = 0;

        param[12] = OFFSET_PARAM_DELAY_TIME;
        param[13] = 0;
        param[14] = (uint8_t)(mDelayTime & 0xFF);
        param[15] = (uint8_t)((mDelayTime >> 8) & 0xFF);
        param[16] = 0;
        param[17] = 0;

        sendCommand(CMD_WRITE_MODULE_CONFIG, param, 18);
        delay(500);

        openknx.console.writeDiagenoseKo("HLK par send");
    }

    if (storedCurrentCalibration)
        logDebugP("Skip writing calibration to sensor as data is current");
    else
    {
        logDebugP("triggerThreshold:");
        logIndentUp();

        // write back trigger thresholds, for each:
        // first 2 bytes parameter offset, then 4 bytes value
        // write in 2 steps as it seems 100 bytes are maximum in one package
        uint8_t offset;
        int rawValue;
        for (uint8_t i = 0; i < 8; i++)
        {
            offset = i * 6;
            param[offset] = OFFSET_PARAM_TRIGGERS + i;
            param[offset + 1] = 0;

            rawValue = dBToRaw(triggerThresholdDb[i]);
            param[offset + 2] = (uint8_t)(rawValue & 0xFF);
            param[offset + 3] = (uint8_t)((rawValue >> 8) & 0xFF);
            param[offset + 4] = (uint8_t)((rawValue >> 16) & 0xFF);
            param[offset + 5] = (uint8_t)((rawValue >> 24) & 0xFF);

            logDebugP("Gate %i:  %.2f", i, triggerThresholdDb[i]);
        }
        sendCommand(CMD_WRITE_MODULE_CONFIG, param, 48);
        delay(500);
        for (uint8_t i = 8; i < 16; i++)
        {
            offset = (i - 8) * 6;
            param[offset] = OFFSET_PARAM_TRIGGERS + i;
            param[offset + 1] = 0;

            rawValue = dBToRaw(triggerThresholdDb[i]);
            param[offset + 2] = (uint8_t)(rawValue & 0xFF);
            param[offset + 3] = (uint8_t)((rawValue >> 8) & 0xFF);
            param[offset + 4] = (uint8_t)((rawValue >> 16) & 0xFF);
            param[offset + 5] = (uint8_t)((rawValue >> 24) & 0xFF);

            logDebugP("Gate %i:  %.2f", i, triggerThresholdDb[i]);
        }
        sendCommand(CMD_WRITE_MODULE_CONFIG, param, 48);
        delay(500);
        logIndentDown();

        logDebugP("holdThreshold:");
        logIndentUp();

        // write back hold thresholds, for each:
        // first 2 bytes parameter offset, then 4 bytes value
        // write in 2 steps as it seems 100 bytes are maximum in one package
        for (uint8_t i = 0; i < 8; i++)
        {
            offset = i * 6;
            param[offset] = OFFSET_PARAM_HOLDS + i;
            param[offset + 1] = 0;

            rawValue = dBToRaw(holdThresholdDb[i]);
            param[offset + 2] = (uint8_t)(rawValue & 0xFF);
            param[offset + 3] = (uint8_t)((rawValue >> 8) & 0xFF);
            param[offset + 4] = (uint8_t)((rawValue >> 16) & 0xFF);
            param[offset + 5] = (uint8_t)((rawValue >> 24) & 0xFF);

            logDebugP("Gate %i:  %.2f", i, holdThresholdDb[i]);
        }
        sendCommand(CMD_WRITE_MODULE_CONFIG, param, 48);
        delay(500);
        for (uint8_t i = 8; i < 16; i++)
        {
            offset = (i - 8) * 6;
            param[offset] = OFFSET_PARAM_HOLDS + i;
            param[offset + 1] = 0;

            rawValue = dBToRaw(holdThresholdDb[i]);
            param[offset + 2] = (uint8_t)(rawValue & 0xFF);
            param[offset + 3] = (uint8_t)((rawValue >> 8) & 0xFF);
            param[offset + 4] = (uint8_t)((rawValue >> 16) & 0xFF);
            param[offset + 5] = (uint8_t)((rawValue >> 24) & 0xFF);

            logDebugP("Gate %i:  %.2f", i, holdThresholdDb[i]);
        }
        sendCommand(CMD_WRITE_MODULE_CONFIG, param, 48);
        delay(500);
        logIndentDown();

        if (!storedCurrentDistanceTime)
            openknx.console.writeDiagenoseKo("");
        openknx.console.writeDiagenoseKo("HLK cal send");
        openknx.console.writeDiagenoseKo("");
        rebootSensorHard();
    }

    logDebugP("Writing config to sensor finished");
    logIndentDown();

    logIndentDown();

    if (!storedCurrentDistanceTime ||
        !storedCurrentCalibration)
    {
        // re-start startup loop to readback values from sensor
        restartStartupLoop();
        calibrationOnOffTimer = delayTimerInit();
    }
}

bool SensorHLKLD2420::useCustomOffsets()
{
    for (uint8_t i = 0; i < 16; i++)
    {
        if (customeTriggerOffsetDb[i] != 0 || customHoldOffsetDb[i] != 0)
            return true;
    }

    return false;
}

void SensorHLKLD2420::sendCommand(uint8_t command, const uint8_t parameter[], uint8_t parameterLength)
{
    uint8_t payloadSize = 2 + parameterLength;
    uint8_t totalLength = HEADER_FOOTER_SIZE + 2 + payloadSize + HEADER_FOOTER_SIZE;

    uint8_t cmdData[totalLength];
    memcpy(cmdData, HEADER_COMMAND, HEADER_FOOTER_SIZE);
    uint8_t cmdDataIndex = HEADER_FOOTER_SIZE;

    cmdData[cmdDataIndex] = payloadSize;
    cmdData[cmdDataIndex + 1] = 0x00;
    cmdDataIndex += 2;

    cmdData[cmdDataIndex] = command;
    cmdData[cmdDataIndex + 1] = 0x00;
    cmdDataIndex += 2;

    memcpy(cmdData + cmdDataIndex, parameter, parameterLength);
    cmdDataIndex += parameterLength;

    memcpy(cmdData + cmdDataIndex, FOOTER, HEADER_FOOTER_SIZE);

    if (HF_SERIAL.availableForWrite())
    {
        HF_SERIAL.write(cmdData, totalLength);

        logTraceP("Sending to sensor:");
        logIndentUp();
        logHexTraceP(cmdData, totalLength);
        logIndentDown();
    }
    else
    {
        logInfoP("Cannot send command, serial not ready to write!");
    }
}

float SensorHLKLD2420::measureValue(MeasureType iMeasureType)
{
    if (mHfSensorStartupState < START_FINISHED)
        return NO_NUM;
    switch (iMeasureType)
    {
        case Pres:
            return lastDetectedRange > 0 ? 1 : 0;
        case Sensitivity:
            if (mSensitivity >= 0)
                return mSensitivity;
            break;
        case Distance:
            return lastDetectedRange;
        default:
            break;
    }
    return NO_NUM;
}

bool SensorHLKLD2420::checkSensorConnection()
{
    return true;
}

bool SensorHLKLD2420::begin()
{
    logDebugP("Starting sensor HLK-LD2420 (Presence)... ");
    bool lResult = Sensor::begin();
    logResult(lResult);
    return lResult;
}

uint8_t SensorHLKLD2420::getI2cSpeed()
{
    return 10; // n * 100kHz // no I2C, so we support "all" frequencies :-)
}

void SensorHLKLD2420::sensorReadFlash(const uint8_t *buffer, const uint16_t size)
{
    if (size == 0)
        return;

    logDebugP("Reading state from flash");
    logIndentUp();

    uint8_t version = openknx.flash.readByte();
    if (version != HLKLD2420_FLASH_VERSION)
    {
        logDebugP("Invalid flash version %u", version);
        return;
    }

    uint32_t magicWord = openknx.flash.readInt();
    if (magicWord != HLKLD2420_FLASH_MAGIC_WORD)
    {
        logDebugP("Flash content invalid");
        return;
    }

    for (uint8_t i = 0; i < 16; i++)
        rawDataRangeAverageDb[i] = openknx.flash.readFloat();

    calibrationCompleted = true;

    logDebugP("Calibration data read from flash");
    logIndentDown();
}

void SensorHLKLD2420::sensorWriteFlash()
{
    if (!calibrationCompleted)
    {
        logDebugP("No data written to flash as calibration not completed");
        return;
    }

    openknx.flash.writeByte(HLKLD2420_FLASH_VERSION);
    openknx.flash.writeInt(HLKLD2420_FLASH_MAGIC_WORD);

    for (uint8_t i = 0; i < 16; i++)
        openknx.flash.writeFloat(rawDataRangeAverageDb[i]);

    logDebugP("Calibration data written to flash");
}

uint16_t SensorHLKLD2420::sensorFlashSize()
{
    return HLKLD2420_FLASH_SIZE;
}

void SensorHLKLD2420::showHelp()
{
    openknx.console.printHelpLine("hlk ver", "Print firmware version of HLK-LD2420 sensor.");
    openknx.console.printHelpLine("hlk sens", "Print sensitivity defined by ETS app.");
    openknx.console.printHelpLine("hlk rmin", "Print min. range defined by ETS app.");
    openknx.console.printHelpLine("hlk rmax", "Print max. range defined by ETS app.");
    openknx.console.printHelpLine("hlk delay", "Print delay time defined by ETS app.");
    openknx.console.printHelpLine("hlk ct read", "Print all 16 calibration trigger thresholds in dB.");
    openknx.console.printHelpLine("hlk cNNt read", "Print calibration trigger threshold in dB at index NN (00-15).");
    openknx.console.printHelpLine("hlk ch read", "Print all 16 calibration hold thresholds in dB.");
    openknx.console.printHelpLine("hlk cNNh read", "Print calibration hold threshold in dB at index NN (00-15).");
    openknx.console.printHelpLine("hlk cr read", "Print all 16 calibration raw data averages in dB.");
    openknx.console.printHelpLine("hlk cNNr read", "Print calibration raw data average in dB at index NN (00-15).");
    openknx.console.printHelpLine("hlk cNNr 00.00", "Set calibration raw data average in dB at index NN (00-15).");
    openknx.console.printHelpLine("hlk cd read", "Print all 16 calibration raw data average differences from last test run in dB.");
    openknx.console.printHelpLine("hlk cNNd read", "Print calibration raw data average difference from last test run in dB at index NN (00-15).");
    openknx.console.printHelpLine("hlk cs read", "Print all 16 calibration raw data standard deviations in dB.");
    openknx.console.printHelpLine("hlk cNNs read", "Print calibration raw data standard deviation in dB at index NN (00-15).");
    openknx.console.printHelpLine("hlk cv read", "Print all 16 calibration raw data variances in dB.");
    openknx.console.printHelpLine("hlk cNNv read", "Print calibration raw data variance in dB at index NN (00-15).");
    openknx.console.printHelpLine("hlk cm read", "Print all 16 calibration raw data maximums in dB.");
    openknx.console.printHelpLine("hlk cNNm read", "Print calibration raw data maximum in dB at index NN (00-15).");
    openknx.console.printHelpLine("hlk ot read", "Print calculated trigger offset in dB based on sensitivity or all custom trigger offsets.");
    openknx.console.printHelpLine("hlk ot 00.00", "Set trigger offset in dB for all 16 indices.");
    openknx.console.printHelpLine("hlk oNNt read", "Print trigger offset in dB at index NN (00-15).");
    openknx.console.printHelpLine("hlk oNNt 00.00", "Set trigger offset in dB at index NN (00-15).");
    openknx.console.printHelpLine("hlk oh read", "Print calculated hold offset based in dB on sensitivity or all custom hold offsets.");
    openknx.console.printHelpLine("hlk oh 00.00", "Set hold offset in dB for all 16 indices.");
    openknx.console.printHelpLine("hlk oNNh read", "Print hold offset in dB in dB at index NN (00-15).");
    openknx.console.printHelpLine("hlk oNNh 00.00", "Set hold offset in dB at index NN (00-15).");
    openknx.console.printHelpLine("hlk o reset", "Reset custom trigger and hold offsets.");
    openknx.console.printHelpLine("hlk cal send", "Send stored calibration data to sensor (e. g. changed by hlk cNNr).");
    openknx.console.printHelpLine("hlk cal run", "Start new sensor calibration run and send data to sensor.");
    openknx.console.printHelpLine("hlk calt run", "Start new sensor calibration test run (no data send to sensor).");
    openknx.console.printHelpLine("hlk cal def", "Send factory default calibration data to sensor.");
    openknx.console.printHelpLine("hlk reb soft", "Reboot sensor via software.");
    openknx.console.printHelpLine("hlk reb hard", "Reboot sensor via hardware (cut power).");
}

bool SensorHLKLD2420::processCommand(const std::string iCmd, bool iDebugKo)
{
    bool lResult = false;
    if (iCmd.length() == 5 && iCmd.substr(4, 1) == "h")
    {
        // Command help
        if (iDebugKo)
        {
            openknx.console.writeDiagenoseKo("-> ver");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> sens");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> rmin");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> rmax");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> delay");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> ct read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cNNt read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> ch read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cNNh read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cr read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cNNr read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cNNr 00.00");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cd read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cNNd read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cs read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cNNs read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cv read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cNNv read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cm read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cNNm read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> ot read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> ot 00.00");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> oNNt read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> oNNt 00.00");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> oh read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> oh 00.00");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> oNNh read");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> oNNh 00.00");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> o reset");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cal send");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cal run");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> calt run");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> cal def");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> reb soft");
            openknx.console.writeDiagenoseKo("");
            openknx.console.writeDiagenoseKo("-> reb hard");
        }
    }
    else if (iCmd.length() == 7 && iCmd.substr(4, 3) == "ver")
    {
        logInfoP("Module version: %s", moduleVersion.c_str());
        if (iDebugKo)
            openknx.console.writeDiagenoseKo("HLK ver %s", moduleVersion.c_str());
        lResult = true;
    }
    else if (iCmd.length() == 8 && iCmd.substr(4, 4) == "sens")
    {
        logInfoP("Sensitivity: %u", mSensitivity);
        if (iDebugKo)
            openknx.console.writeDiagenoseKo("HLK sens %u", mSensitivity);
        lResult = true;
    }
    else if (iCmd.length() == 8 && iCmd.substr(4, 4) == "rmin")
    {
        logInfoP("Range gate min.: %u", mRangeGateMin);
        if (iDebugKo)
            openknx.console.writeDiagenoseKo("HLK rmin %u", mRangeGateMin);
        lResult = true;
    }
    else if (iCmd.length() == 8 && iCmd.substr(4, 4) == "rmax")
    {
        logInfoP("Range gate max.: %u", mRangeGateMax);
        if (iDebugKo)
            openknx.console.writeDiagenoseKo("HLK rmax %u", mRangeGateMax);
        lResult = true;
    }
    else if (iCmd.length() == 9 && iCmd.substr(4, 5) == "delay")
    {
        logInfoP("Delay time: %u", mDelayTime);
        if (iDebugKo)
            openknx.console.writeDiagenoseKo("HLK delay %u", mDelayTime);
        lResult = true;
    }
    else if (iCmd.length() == 12 && iCmd.substr(4, 10) == "calt run")
    {
        calibrationTestRunOnly = true;
        forceCalibration();
        lResult = true;
    }
    else if (iCmd.length() >= 8 && iCmd.substr(4, 4) == "calt")
    {
        // other calt commands (typos) might be interpreted
        // as cNNt commands (with fatal error), so we prevent them
        lResult = false;
    }
    else if (iCmd.length() == 11 && iCmd.substr(4, 9) == "cal run")
    {
        forceCalibration();
        lResult = true;
    }
    else if (iCmd.length() == 12 && iCmd.substr(4, 10) == "cal send")
    {
        sendCalibrationData();
        lResult = true;
    }
    else if (iCmd.length() == 11 && iCmd.substr(4, 9) == "cal def")
    {
        useFactoryDefaultThresholds = true;
        sendCalibrationData();
        lResult = true;
    }
    else if (iCmd.length() == 12 && iCmd.substr(4, 10) == "reb soft")
    {
        rebootSensorSoft();
        lResult = true;
    }
    else if (iCmd.length() == 12 && iCmd.substr(4, 10) == "reb hard")
    {
        rebootSensorHard();
        lResult = true;
    }
    else if (iCmd.length() == 11)
    {
        if (iCmd.substr(4, 7) == "ct read")
        {
            // calibration trigger thresholds: read all
            for (uint8_t i = 0; i < 16; i++)
            {
                logInfoP("triggerThresholdDb, gate %u: %.2f", i, triggerThresholdDb[i]);
                if (iDebugKo)
                {
                    openknx.console.writeDiagenoseKo("HLK c%02ut %.2f", i, triggerThresholdDb[i]);
                    if (i < 15)
                        openknx.console.writeDiagenoseKo("");
                }
            }
            lResult = true;
        }
        else if (iCmd.substr(4, 7) == "ch read")
        {
            // calibration hold thresholds: read all
            for (uint8_t i = 0; i < 16; i++)
            {
                logInfoP("holdThresholdDb, gate %u: %.2f", i, holdThresholdDb[i]);
                if (iDebugKo)
                {
                    openknx.console.writeDiagenoseKo("HLK c%02uh %.2f", i, holdThresholdDb[i]);
                    if (i < 15)
                        openknx.console.writeDiagenoseKo("");
                }
            }
            lResult = true;
        }
        else if (iCmd.substr(4, 7) == "cr read")
        {
            // calibration raw data averages: read all
            for (uint8_t i = 0; i < 16; i++)
            {
                logInfoP("rawDataRangeAverageDb, gate %u: %.2f", i, rawDataRangeAverageDb[i]);
                if (iDebugKo)
                {
                    openknx.console.writeDiagenoseKo("HLK c%02ur %.2f", i, rawDataRangeAverageDb[i]);
                    if (i < 15)
                        openknx.console.writeDiagenoseKo("");
                }
            }
            lResult = true;
        }
        else if (iCmd.substr(4, 7) == "cd read")
        {
            // calibration raw data average differences: read all
            for (uint8_t i = 0; i < 16; i++)
            {
                if (rawDataRangeDifferencesDb[i] < 0)
                {
                    logInfoP("rawDataRangeDifferencesDb, gate %u:%.2f", i, rawDataRangeDifferencesDb[i]);
                    if (iDebugKo)
                        openknx.console.writeDiagenoseKo("HLK c%02ud%.2f", i, rawDataRangeDifferencesDb[i]);
                }
                else
                {
                    logInfoP("rawDataRangeDifferencesDb, gate %u: %.2f", i, rawDataRangeDifferencesDb[i]);
                    if (iDebugKo)
                        openknx.console.writeDiagenoseKo("HLK c%02ud %.2f", i, rawDataRangeDifferencesDb[i]);
                }
                if (iDebugKo && i < 15)
                    openknx.console.writeDiagenoseKo("");
            }
            lResult = true;
        }
        else if (iCmd.substr(4, 7) == "cs read")
        {
            // calibration raw data standard deviation: read all
            for (uint8_t i = 0; i < 16; i++)
            {
                logInfoP("rawDataRangeDeviationDb, gate %u: %.2f", i, rawDataRangeDeviationDb[i]);
                if (iDebugKo)
                {
                    openknx.console.writeDiagenoseKo("HLK c%02us %.2f", i, rawDataRangeDeviationDb[i]);
                    if (i < 15)
                        openknx.console.writeDiagenoseKo("");
                }
            }
            lResult = true;
        }
        else if (iCmd.substr(4, 7) == "cv read")
        {
            // calibration raw data variance: read all
            for (uint8_t i = 0; i < 16; i++)
            {
                logInfoP("rawDataRangeVarianceDb, gate %u: %.2f", i, rawDataRangeVarianceDb[i]);
                if (iDebugKo)
                {
                    openknx.console.writeDiagenoseKo("HLK c%02uv %.2f", i, rawDataRangeVarianceDb[i]);
                    if (i < 15)
                        openknx.console.writeDiagenoseKo("");
                }
            }
            lResult = true;
        }
        else if (iCmd.substr(4, 7) == "cm read")
        {
            // calibration raw data max: read all
            for (uint8_t i = 0; i < 16; i++)
            {
                logInfoP("rawDataRangeMaxDb, gate %u: %.2f", i, rawDataRangeMaxDb[i]);
                if (iDebugKo)
                {
                    openknx.console.writeDiagenoseKo("HLK c%02um %.2f", i, rawDataRangeMaxDb[i]);
                    if (i < 15)
                        openknx.console.writeDiagenoseKo("");
                }
            }
            lResult = true;
        }
        else if (iCmd.substr(4, 7) == "ot read")
        {
            // trigger offsets: read calculated or all custom
            if (useCustomOffsets())
            {
                for (uint8_t i = 0; i < 16; i++)
                {
                    logInfoP("customeTriggerOffsetDb, gate %u: %.2f", i, customeTriggerOffsetDb[i]);
                    if (iDebugKo)
                    {
                        openknx.console.writeDiagenoseKo("HLK o%02ut %.2f", i, customeTriggerOffsetDb[i]);
                        if (i < 15)
                            openknx.console.writeDiagenoseKo("");
                    }
                }
            }
            else
            {
                logInfoP("triggerOffsetDb: %.2f", triggerOffsetDb);
                if (iDebugKo)
                    openknx.console.writeDiagenoseKo("HLK ot %.2f", triggerOffsetDb);
            }
            lResult = true;
        }
        else if (iCmd.substr(4, 7) == "oh read")
        {
            // hold offsets: read calculated or all custom
            if (useCustomOffsets())
            {
                for (uint8_t i = 0; i < 16; i++)
                {
                    logInfoP("customHoldOffsetDb, gate %u: %.2f", i, customHoldOffsetDb[i]);
                    if (iDebugKo)
                    {
                        openknx.console.writeDiagenoseKo("HLK o%02uh %.2f", i, customHoldOffsetDb[i]);
                        if (i < 15)
                            openknx.console.writeDiagenoseKo("");
                    }
                }
            }
            else
            {
                logInfoP("holdOffsetDb: %.2f", holdOffsetDb);
                if (iDebugKo)
                    openknx.console.writeDiagenoseKo("HLK oh %.2f", holdOffsetDb);
            }
            lResult = true;
        }
        else if (iCmd.substr(4, 7) == "o reset")
        {
            // reset custom trigger and hold offsets
            for (uint8_t i = 0; i < 16; i++)
            {
                customeTriggerOffsetDb[i] = 0;
                customHoldOffsetDb[i] = 0;
            }

            lResult = true;
        }
    }
    else if (iCmd.length() == 12 && iCmd.substr(4, 1) == "o")
    {
        // set all offset values to the same new value
        float valueNew = stod(iCmd.substr(7, 5));

        if (iCmd.substr(5, 1) == "t")
        {
            // trigger offsets
            for (uint8_t i = 0; i < 16; i++)
                customeTriggerOffsetDb[i] = valueNew;
            lResult = true;
        }
        else if (iCmd.substr(5, 1) == "h")
        {
            // hold offsets
            for (uint8_t i = 0; i < 16; i++)
                customHoldOffsetDb[i] = valueNew;
            lResult = true;
        }
    }
    else if (iCmd.length() == 13 && iCmd.substr(4, 1) == "c")
    {
        // read single calibration data value
        uint8_t valueIndex = stoi(iCmd.substr(5, 2));

        if (iCmd.substr(7, 1) == "t")
        {
            // trigger thresholds
            if (iCmd.substr(9, 4) == "read")
            {
                // read value
                logInfoP("triggerThresholdDb, gate %u: %.2f", valueIndex, triggerThresholdDb[valueIndex]);
                if (iDebugKo)
                    openknx.console.writeDiagenoseKo("HLK c%02ut %.2f", valueIndex, triggerThresholdDb[valueIndex]);
                lResult = true;
            }
        }
        else if (iCmd.substr(7, 1) == "h")
        {
            // hold thresholds
            if (iCmd.substr(9, 4) == "read")
            {
                // read value
                logInfoP("holdThresholdDb, gate %u: %.2f", valueIndex, holdThresholdDb[valueIndex]);
                if (iDebugKo)
                    openknx.console.writeDiagenoseKo("HLK c%02uh %.2f", valueIndex, holdThresholdDb[valueIndex]);
                lResult = true;
            }
        }
        else if (iCmd.substr(7, 1) == "r")
        {
            // calibration raw data averages
            if (iCmd.substr(9, 4) == "read")
            {
                // read value
                logInfoP("rawDataRangeAverageDb, gate %u: %.2f", valueIndex, rawDataRangeAverageDb[valueIndex]);
                if (iDebugKo)
                    openknx.console.writeDiagenoseKo("HLK c%02ur %.2f", valueIndex, rawDataRangeAverageDb[valueIndex]);
                lResult = true;
            }
        }
        else if (iCmd.substr(7, 1) == "d")
        {
            // calibration raw data average differences
            if (iCmd.substr(9, 4) == "read")
            {
                // read value
                if (rawDataRangeDifferencesDb[valueIndex] < 0)
                {
                    logInfoP("rawDataRangeDifferencesDb, gate %u:%.2f", valueIndex, rawDataRangeDifferencesDb[valueIndex]);
                    if (iDebugKo)
                        openknx.console.writeDiagenoseKo("HLK c%02ud%.2f", valueIndex, rawDataRangeDifferencesDb[valueIndex]);
                }
                else
                {
                    logInfoP("rawDataRangeDifferencesDb, gate %u: %.2f", valueIndex, rawDataRangeDifferencesDb[valueIndex]);
                    if (iDebugKo)
                        openknx.console.writeDiagenoseKo("HLK c%02ud %.2f", valueIndex, rawDataRangeDifferencesDb[valueIndex]);
                }
                lResult = true;
            }
        }
    }
    else if (iCmd.length() == 13 && iCmd.substr(4, 1) == "o")
    {
        // read single custom offset value
        uint8_t valueIndex = stoi(iCmd.substr(5, 2));

        if (iCmd.substr(7, 1) == "t")
        {
            // trigger thresholds
            if (iCmd.substr(9, 4) == "read")
            {
                // read value
                logInfoP("customeTriggerOffsetDb, gate %u: %.2f", valueIndex, customeTriggerOffsetDb[valueIndex]);
                if (iDebugKo)
                    openknx.console.writeDiagenoseKo("HLK o%02ut %.2f", valueIndex, customeTriggerOffsetDb[valueIndex]);
                lResult = true;
            }
        }
        else if (iCmd.substr(7, 1) == "h")
        {
            // hold thresholds
            if (iCmd.substr(9, 4) == "read")
            {
                // read value
                logInfoP("customHoldOffsetDb, gate %u: %.2f", valueIndex, customHoldOffsetDb[valueIndex]);
                if (iDebugKo)
                    openknx.console.writeDiagenoseKo("HLK c%02uh %.2f", valueIndex, customHoldOffsetDb[valueIndex]);
                lResult = true;
            }
        }
    }
    else if (iCmd.length() == 14 && iCmd.substr(4, 1) == "c")
    {
        // set single calibration data value
        uint8_t valueIndex = stoi(iCmd.substr(5, 2));

        if (iCmd.substr(7, 1) == "r")
        {
            rawDataRangeAverageDb[valueIndex] = stod(iCmd.substr(9, 5));
            lResult = true;
        }
    }
    else if (iCmd.length() == 14 && iCmd.substr(4, 1) == "o")
    {
        // set single offset value
        uint8_t valueIndex = stoi(iCmd.substr(5, 2));

        if (iCmd.substr(7, 1) == "t")
        {
            customeTriggerOffsetDb[valueIndex] = stod(iCmd.substr(9, 5));
            lResult = true;
        }
        else if (iCmd.substr(7, 1) == "h")
        {
            customHoldOffsetDb[valueIndex] = stod(iCmd.substr(9, 5));
            lResult = true;
        }
    }
    return lResult;
}

    #endif
#endif