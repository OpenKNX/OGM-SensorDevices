#ifdef PMMODULE
#ifdef HF_SERIAL
#include "SensorHLKLD2420.h"
#include <Arduino.h>
#include <Wire.h>
#include <OpenKNX.h>

SensorHLKLD2420::SensorHLKLD2420(uint16_t iMeasureTypes, TwoWire *iWire)
    : SensorHLKLD2420(iMeasureTypes, &Wire, 0){};

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

    if (mRangeGateMax > mRangeGateMin)
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
            if (minDistance > NO_NUM)
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
            if (delayTime > NO_NUM)
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
    // #ToDo: discuss how to get triggered externally

    sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE_LENGTH);
    delay(100);

    calibrationCompleted = false;
    pSensorState = Calibrate;
    mHfSensorStartupState = START_CALIBRATING;

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

double SensorHLKLD2420::rawToDb(int rawValue)
{
    return 10 * log10(rawValue);
}

int SensorHLKLD2420::dBToRaw(double dbValue)
{
    return int(pow(10, dbValue / 10));
}

void SensorHLKLD2420::resetRawDataRecording()
{
    for (uint8_t i = 0; i < 16; i++)
        rawDataRangeAverage[i] = 0;

    rawDataLastRecordingReceived = millis();
    rawDataRecordingCount = 0;

    logDebugP("Start sensor calibration");
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
                newDetectedRange = stoi(rangeString) / (float)10;

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

                        minDistance = bytesToInt(mBuffer[0], mBuffer[1], mBuffer[2], mBuffer[3]);
                        logDebugP("minDistance: %d", minDistance);

                        maxDistance = bytesToInt(mBuffer[4], mBuffer[5], mBuffer[6], mBuffer[7]);
                        logDebugP("maxDistance: %d", maxDistance);

                        logDebugP("triggerThreshold:");
                        logIndentUp();
                        int triggerThresholdTemp;
                        for (uint8_t i = 0; i < 16; i++)
                        {
                            triggerThresholdTemp = bytesToInt(mBuffer[i * 4 + 8], mBuffer[i * 4 + 9], mBuffer[i * 4 + 10], mBuffer[i * 4 + 11]);
                            logDebugP("Gate %i: %.2f", i, rawToDb(triggerThresholdTemp));
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

                        delayTime = bytesToInt(mBuffer[0], mBuffer[1], mBuffer[2], mBuffer[3]);
                        logDebugP("delayTime: %d", delayTime);

                        logDebugP("holdThreshold:");
                        logIndentUp();
                        int holdThresholdTemp;
                        for (uint8_t i = 0; i < 16; i++)
                        {
                            holdThresholdTemp = bytesToInt(mBuffer[i * 4 + 4], mBuffer[i * 4 + 5], mBuffer[i * 4 + 6], mBuffer[i * 4 + 7]);
                            logDebugP("Gate %i: %.2f", i, rawToDb(holdThresholdTemp));
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
                resetRawDataRecording();
            }
            else if (rawDataRecordingCount < CALIBRATION_VALUE_COUNT)
            {
                for (uint8_t i = 0; i < 16; i++)
                {
                    // calculate rolling average
                    rawDataRangeAverage[i] -= rawDataRangeAverage[i] / CALIBRATION_VALUE_COUNT;
                    rawDataRangeAverage[i] += (float)rangeMax[i] / CALIBRATION_VALUE_COUNT;
                }
                rawDataRecordingCount++;

                rawDataLastRecordingReceived = millis();
            }

            if (rawDataRecordingCount >= CALIBRATION_VALUE_COUNT)
            {
                // enter command mode to stop raw data transfer
                sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE_LENGTH);

                // reboot module to return to normal (not raw data) operation
                sendCommand(CMD_REBOOT_MODULE);
                delay(1000);

                sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE_LENGTH);
                delay(500);

                sendCalibrationData();

                logIndentDown();

                logDebugP("Sensor calibration finished");
                calibrationCompleted = true;
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

    logTraceP("rawDataRangeAverage:");
    logIndentUp();

    int8_t lSensitivity = mSensitivity > 0 && mSensitivity <= 10 ? mSensitivity : SENSITIVITY_DEFAULT;
    triggerOffsetDb = CALIBRATION_TRIGGER_OFFSET_DB - SENSITIVITY_TRIGGER_RANGE * (lSensitivity / 10.0);

    // convert to dB values and add trigger offset
    double triggerThresholdDb[16];
    for (uint8_t i = 0; i < 16; i++)
    {
        triggerThresholdDb[i] = rawToDb(rawDataRangeAverage[i]) + triggerOffsetDb;
        logTraceP("Gate %i:  %.2f", i, rawToDb(rawDataRangeAverage[i]));
    }

    logIndentDown();

    holdOffsetDb = CALIBRATION_HOLD_OFFSET_DB - SENSITIVITY_HOLD_RANGE * (lSensitivity / 10.0);

    // substract hold offset
    double holdThresholdDb[16];
    for (uint8_t i = 0; i < 16; i++)
    {
        holdThresholdDb[i] = triggerThresholdDb[i] - holdOffsetDb;
    }

    logDebugP("Sensitivity used:");
    logIndentUp();
    logDebugP("User setting: %d", mSensitivity);
    logDebugP("Calculated trigger offset : %.2f", triggerOffsetDb);
    logDebugP("Calculated hold offset: %.2f", holdOffsetDb);
    logIndentDown();

    logDebugP("Write config to sensor:");
    logIndentUp();
    logDebugP("Range gate min.: %d", mRangeGateMin);
    logDebugP("Range gate max.: %d", mRangeGateMax);
    logDebugP("Delay time: %d", mDelayTime);

    uint8_t param[48];

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
        triggerThreshold[i] = rawValue;

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
        triggerThreshold[i] = rawValue;

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
        holdThreshold[i] = rawValue;

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
        holdThreshold[i] = rawValue;

        logDebugP("Gate %i:  %.2f", i, holdThresholdDb[i]);
    }
    sendCommand(CMD_WRITE_MODULE_CONFIG, param, 48);
    delay(500);
    logIndentDown();

    logIndentDown();
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
        rawDataRangeAverage[i] = openknx.flash.readFloat();
    
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
        openknx.flash.writeFloat(rawDataRangeAverage[i]);

    logDebugP("Calibration data written to flash");
}

uint16_t SensorHLKLD2420::sensorFlashSize()
{
    return HLKLD2420_FLASH_SIZE;
}

void SensorHLKLD2420::showHelp()
{
    openknx.console.printHelpLine("hlk ver", "Print firmware version of HLK-LD2420 scanner");
    openknx.console.printHelpLine("hlk sens", "Print sensitivity defined by ETS app");
    openknx.console.printHelpLine("hlk offt", "Print calculated trigger offset based on sensitivity");
    openknx.console.printHelpLine("hlk offh", "Print calculated hold offset based on sensitivity");
    openknx.console.printHelpLine("hlk rmin", "Print min. range defined by ETS app");
    openknx.console.printHelpLine("hlk rmax", "Print max. range defined by ETS app");
    openknx.console.printHelpLine("hlk delay", "Print delay time defined by ETS app");
    openknx.console.printHelpLine("hlk ct read", "Print all 16 calibration trigger thresholds in dB");
    openknx.console.printHelpLine("hlk cNNt read", "Print calibration trigger threshold in dB at index NN (00-15)");
    //openknx.console.printHelpLine("hlk cNNt 00.00", "Set calibration hold threshold in dB at index NN (00-15)");
    openknx.console.printHelpLine("hlk ch read", "Print all 16 calibration hold thresholds in dB");
    openknx.console.printHelpLine("hlk cNNh read", "Print calibration hold threshold in dB at index NN (00-15)");
    //openknx.console.printHelpLine("hlk cNNh 00.00", "Set calibration hold threshold in dB at index NN (00-15)");
    openknx.console.printHelpLine("hlk calibrate", "Force new sensor calibration run");
}

bool SensorHLKLD2420::processCommand(const std::string iCmd, bool iDebugKo)
{
    bool lResult = false;
    if (iCmd.length() == 7 && iCmd.substr(4, 3) == "ver")
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
    else if (iCmd.length() == 8 && iCmd.substr(4, 4) == "offt")
    {
        logInfoP("triggerOffsetDb: %.2f", triggerOffsetDb);
        if (iDebugKo)
            openknx.console.writeDiagenoseKo("HLK offt %.2f", triggerOffsetDb);
        lResult = true;
    }
    else if (iCmd.length() == 8 && iCmd.substr(4, 4) == "offh")
    {
        logInfoP("holdOffsetDb: %.2f", holdOffsetDb);
        if (iDebugKo)
            openknx.console.writeDiagenoseKo("HLK offh %.2f", holdOffsetDb);
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
    else if (iCmd.length() == 13 && iCmd.substr(4, 9) == "calibrate")
    {
        forceCalibration();
        lResult = true;
    }
    else if (iCmd.length() == 11)
    {
        if (iCmd.substr(4, 7) == "ct read")
        {
            // calibration trigger thresholds: read all
            for (uint8_t i = 0; i < 16; i++)
            {
                logInfoP("triggerThreshold, gate %u: %.2f", i, rawToDb(triggerThreshold[i]));
                if (iDebugKo)
                {
                    openknx.console.writeDiagenoseKo("HLK t%02ut %.2f", i, rawToDb(triggerThreshold[i]));
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
                logInfoP("holdThreshold, gate %u: %.2f", i, rawToDb(holdThreshold[i]));
                if (iDebugKo)
                {
                    openknx.console.writeDiagenoseKo("HLK t%02uh %.2f", i, rawToDb(holdThreshold[i]));
                    openknx.console.writeDiagenoseKo("");
                }
            }
            lResult = true;
        }
    }
    else if (iCmd.length() == 13 && iCmd.substr(4, 1) == "c")
    {
        // read or set single calibration data value
        uint8_t valueIndex = stoi(iCmd.substr(5, 2));

        if (iCmd.substr(7, 1) == "t")
        {
            // trigger thresholds
            if (iCmd.substr(9, 4) == "read")
            {
                // read value
                logInfoP("triggerThreshold, gate %u: %.2f", valueIndex, rawToDb(triggerThreshold[valueIndex]));
                if (iDebugKo)
                {
                    openknx.console.writeDiagenoseKo("HLK t%02ut %.2f", valueIndex, rawToDb(triggerThreshold[valueIndex]));
                    openknx.console.writeDiagenoseKo("");
                }
                lResult = true;
            }
        }
        else if (iCmd.substr(7, 1) == "h")
        {
            // hold thresholds
            if (iCmd.substr(9, 4) == "read")
            {
                // read value
                logInfoP("holdThreshold, gate %u: %.2f", valueIndex, rawToDb(holdThreshold[valueIndex]));
                if (iDebugKo)
                {
                    openknx.console.writeDiagenoseKo("HLK t%02uh %.2f", valueIndex, rawToDb(holdThreshold[valueIndex]));
                    openknx.console.writeDiagenoseKo("");
                }
                lResult = true;
            }
        }
    }

    return lResult;
}

#endif
#endif