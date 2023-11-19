// #include "IncludeManager.h"
#ifdef PMMODULE
#ifdef SERIAL_HF
#include <Arduino.h>
#include <Wire.h>
#include "SensorHLKLD2420.h"

SensorHLKLD2420::SensorHLKLD2420(uint16_t iMeasureTypes)
    : SensorHLKLD2420(iMeasureTypes, 0){};

SensorHLKLD2420::SensorHLKLD2420(uint16_t iMeasureTypes, uint8_t iAddress)
    : Sensor(iMeasureTypes, iAddress)
{
    gMeasureTypes |= Pres | Sensitivity | Distance;
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

    // restart calibration with new sensitivity value
    mHfSensorStartupStates = START_READ2_DONE;
}

uint8_t SensorHLKLD2420::getSensorClass()
{
    return SENS_HLKLD2420;
}

void SensorHLKLD2420::sensorLoopInternal()
{
    switch (gSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
        case Calibrate:
            if (delayCheck(pSensorStateDelay, 50))
            {
                gSensorState = Finalize;
                pSensorStateDelay = millis();
            }
            break;
        case Finalize:
            if (delayCheck(pSensorStateDelay, 50))
            {
                gSensorState = Running;
                pSensorStateDelay = millis();
            }
            break;
        case Running:
            uartGetPacket();
            sendDefaultSensorValues();
            break;
        default:
            pSensorStateDelay = millis();
            break;
    }
}

// this state engine calculates startup behaviour of HF sensor
// and sends default values as soon as the sensor can consume them
void SensorHLKLD2420::sendDefaultSensorValues()
{
    // logDebugP("sendDefaultSensorValues: %d, %.2f", mHfSensorStartupStates, lastDetectedRange);
    switch (mHfSensorStartupStates)
    {
        case START_INIT:
            if (lastDetectedRange > NO_NUM)
            {
                pSensorStateDelay = millis();
                mHfSensorStartupStates = START_SENSOR_ACTIVE;

                sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE);
            }
            break;
        case START_SENSOR_ACTIVE:
            // Communication is established, we wait for version info from Sensor
            if (delayCheck(pSensorStateDelay, 1000))
            {
                pSensorStateDelay = millis();
                if (!moduleVersion.empty())
                {
                    mHfSensorStartupStates = START_VERSION_RECEIVED;
                }
                else
                {
                    sendCommand(CMD_READ_VERSION);
                }
            }
            break;
        case START_VERSION_RECEIVED:
            // We got version, we wait for read 1 done
            if (delayCheck(pSensorStateDelay, 2000))
            {
                pSensorStateDelay = millis();
                if (minDistance > NO_NUM)
                {
                    mHfSensorStartupStates = START_READ1_DONE;
                }
                else
                {
                    sendCommand(CMD_READ_MODULE_CONFIG, PARAM_READ_DISTANCE_TRIGGER);
                }
            }
            break;
        case START_READ1_DONE:
            // Read 1 is done, we wait for read 2 done
            if (delayCheck(pSensorStateDelay, 2000))
            {
                pSensorStateDelay = millis();
                if (delayTime > NO_NUM)
                {
                    mHfSensorStartupStates = START_READ2_DONE;
                }
                else
                {
                    sendCommand(CMD_READ_MODULE_CONFIG, PARAM_READ_DELAY_MAINTAIN);
                }
            }
            break;
        case START_READ2_DONE:
            // All done, close command mode again
            if (delayCheck(pSensorStateDelay, 1000))
            {
                pSensorStateDelay = millis();
                mHfSensorStartupStates = START_FINISHED;

                resetRawDataRecording();
                sendCommand(CMD_RAW_DATA_MODE, PARAM_RAW_DATA_MODE);

                sendCommand(CMD_CLOSE_COMMAND_MODE);
            }
            break;
        default:
            mHfSensorStartupStates = START_FINISHED;
            break;
    }
}

void SensorHLKLD2420::uartGetPacket()
{
    uint8_t rxByte;
    switch (mPacketState)
    {
        case GET_SYNC_STATE:
            // wait for a valid header
            while (mPacketState == GET_SYNC_STATE)
            {
                if (SERIAL_HF.available() > 0 && SERIAL_HF.readBytes(&rxByte, 1) == 1)
                {
                    mBuffer.push_back((byte)rxByte);
                    if (mBuffer.size() == HEADER_FOOTER_SIZE)
                    {
                        /*logTraceP("Header:");
                        logIndentUp();
                        logHexTraceP(mBuffer.data(), mBuffer.size());
                        logIndentDown();*/

                        if (mBuffer == HEADER_ON)
                        {
                            mPacketType = ON;
                            mPacketState = GET_PACKET_DATA;
                            break;
                        }
                        if (mBuffer == HEADER_OFF)
                        {
                            mPacketType = OFF;
                            mPacketState = GET_PACKET_DATA;
                            break;
                        }
                        if (mBuffer == HEADER_COMMAND)
                        {
                            mPacketType = COMMAND_RESPONSE;
                            mPacketState = GET_PACKET_DATA;
                            break;
                        }
                        if (mBuffer == HEADER_RAW_DATA)
                        {
                            mPacketType = RAW_DATA;
                            mPacketState = GET_PACKET_DATA;
                            break;
                        }
                    }

                    // if the current buffer is the expected size of the header,
                    // but we did not find a valid header yet, we remove the first element
                    if (mBuffer.size() == HEADER_FOOTER_SIZE)
                    {
                        mBuffer.erase(mBuffer.begin());
                    }
                }
                else if (delayCheck(pSensorStateDelay, 1000))
                {
                    // no data received for 1 sec., cancel header sync
                    break;
                }
            }
            break;
        case GET_PACKET_DATA:
            // add data till valid footer received
            while (mPacketState == GET_PACKET_DATA)
            {
                if (SERIAL_HF.available() > 0 && SERIAL_HF.readBytes(&rxByte, 1) == 1)
                {
                    mBuffer.push_back((byte)rxByte);
                    if (mPacketType == COMMAND_RESPONSE && equal(mBuffer.end() - HEADER_FOOTER_SIZE, mBuffer.end(), FOOTER.begin()) ||
                        mPacketType == RAW_DATA && equal(mBuffer.end() - HEADER_FOOTER_SIZE, mBuffer.end(), FOOTER_RAW_DATA.begin()))
                    {
                        mPacketState = PROCESS_PACKET_STATE;
                        break;
                    }

                    if ((mPacketType == OFF || mPacketType == ON && mBuffer.size() > 10) &&
                        mBuffer.end()[-2] == 13 && mBuffer.end()[-1] == 10)
                    { // new line
                        mPacketState = PROCESS_PACKET_STATE;
                        break;
                    }
                }
                else if (delayCheck(pSensorStateDelay, 1000))
                {
                    // no data received for 1 sec., cancel get packet data
                    break;
                }
            }
            break;
        case PROCESS_PACKET_STATE:
            getSensorData();
            mBuffer.clear();
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

std::vector<byte> SensorHLKLD2420::intToBytes(int intValue)
{
    std::vector<byte> bytes;

    bytes.push_back((byte)(intValue & 0xFF));
    bytes.push_back((byte)((intValue >> 8) & 0xFF));
    bytes.push_back((byte)((intValue >> 16) & 0xFF));
    bytes.push_back((byte)((intValue >> 24) & 0xFF));
    return bytes;
}

std::vector<byte> SensorHLKLD2420::shortToBytes(short shortValue)
{
    std::vector<byte> bytes;

    bytes.push_back((byte)(shortValue & 0xFF));
    bytes.push_back((byte)((shortValue >> 8) & 0xFF));
    return bytes;
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
    for (int i = 0; i < 16; i++)
    {
        rawDataRangeAverage[i] = 0;
    }

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

    int payloadSize;

    int rangeMax[16] = {};
    int dopplerOffset;
    int rangeOffset;
    int rangeValue;

    switch (mPacketType)
    {
        case ON:
            /*logTraceP("Content ON:");
            logIndentUp();
            logHexTraceP(mBuffer.data(), mBuffer.size());
            logIndentDown();*/

            // cut 10 bytes at the beginning: 4F 4E 0D 0A 52 61 6E 67 65 20 ("ON  RANGE ")
            // and 2 bytes at the end
            mBuffer.erase(mBuffer.begin(), mBuffer.begin() + 10);
            mBuffer.erase(mBuffer.end() - 2, mBuffer.end());

            // mBuffer now holds the detection range decimal value as string
            rangeString = std::string(reinterpret_cast<const char *>(&mBuffer[0]), mBuffer.size());
            newDetectedRange = stoi(rangeString) / (float)10;

            if (lastDetectedRange != newDetectedRange)
            {
                lastDetectedRange = newDetectedRange;
                logDebugP("Presence detected, range: %.1f m", lastDetectedRange);
            }

            result = true;
            break;

        case OFF:
            /*logTraceP("Content OFF:");
            logIndentUp();
            logHexTraceP(mBuffer.data(), mBuffer.size());
            logIndentDown();*/

            if (lastDetectedRange != -1)
            {
                lastDetectedRange = -1;
                logDebugP("No presence detected");
            }

            result = true;
            break;

        case COMMAND_RESPONSE:
            payloadSize = int(mBuffer[4]);

            // cut 6 bytes (4 header + 2 size) at the beginning and 4 bytes footer at the end
            mBuffer.erase(mBuffer.begin(), mBuffer.begin() + 6);
            mBuffer.erase(mBuffer.end() - 4, mBuffer.end());

            if (mBuffer.size() != payloadSize)
            {
                logDebugP("Invalid command reponse packet size: %d", mBuffer.size());
                break;
            }

            // we expect "01 00 00" after the command code for success
            success = mBuffer[1] == (byte)1 && mBuffer[2] == (byte)0 && mBuffer[3] == (byte)0;
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
                    mBuffer.erase(mBuffer.begin(), mBuffer.begin() + 6);

                    // mBuffer now holds the version value as string
                    moduleVersion = std::string(reinterpret_cast<const char *>(&mBuffer[0]), mBuffer.size());
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
                    mBuffer.erase(mBuffer.begin(), mBuffer.begin() + 4);

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
                        for (int i = 0; i < 16; i++)
                        {
                            triggerThreshold[i] = bytesToInt(mBuffer[i * 4 + 8], mBuffer[i * 4 + 9], mBuffer[i * 4 + 10], mBuffer[i * 4 + 11]);
                            logDebugP("Gate %i: %.2f", i, rawToDb(triggerThreshold[i]));
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
                        for (int i = 0; i < 16; i++)
                        {
                            holdThreshold[i] = bytesToInt(mBuffer[i * 4 + 4], mBuffer[i * 4 + 5], mBuffer[i * 4 + 6], mBuffer[i * 4 + 7]);
                            logDebugP("Gate %i: %.2f", i, rawToDb(holdThreshold[i]));
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
            mBuffer.erase(mBuffer.begin(), mBuffer.begin() + 4);
            mBuffer.erase(mBuffer.end() - 4, mBuffer.end());

            for (int i = 0; i < 20; i++)
            {
                dopplerOffset = i * 64;

                for (int j = 0; j < 16; j++)
                {
                    rangeOffset = dopplerOffset + j * 4;
                    rangeValue = bytesToInt(mBuffer[rangeOffset], mBuffer[rangeOffset + 1], mBuffer[rangeOffset + 2], mBuffer[rangeOffset + 3]);
                    rangeMax[j] = max(rangeMax[j], rangeValue);
                }
            }

            logTraceP("Range values received (%d):", rawDataRecordingCount);
            logIndentUp();
            for (int i = 0; i < 16; i++)
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
                for (int i = 0; i < 16; i++)
                {
                    // calculate rolling average
                    rawDataRangeAverage[i] -= rawDataRangeAverage[i] / CALIBRATION_VALUE_COUNT;
                    rawDataRangeAverage[i] += (double)rangeMax[i] / CALIBRATION_VALUE_COUNT;
                }
                rawDataRecordingCount++;

                rawDataLastRecordingReceived = millis();
            }

            if (rawDataRecordingCount >= CALIBRATION_VALUE_COUNT)
            {
                // enter command mode to stop raw data transfer
                sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE);

                logTraceP("rawDataRangeAverage:");
                logIndentUp();

                int8_t lSensitivity = mSensitivity > 0 && mSensitivity <= 10 ? mSensitivity : SENSITIVITY_DEFAULT;
                double triggerOffsetDb = CALIBRATION_TRIGGER_OFFSET_DB - SENSITIVITY_TRIGGER_RANGE * (lSensitivity / 10.0);

                // convert to dB values and add trigger offset
                double triggerThresholdDb[16];
                for (int i = 0; i < 16; i++)
                {
                    triggerThresholdDb[i] = rawToDb(rawDataRangeAverage[i]) + triggerOffsetDb;
                    logTraceP("Gate %i:  %.2f", i, rawToDb(rawDataRangeAverage[i]));
                }

                logIndentDown();

                double holdOffsetDb = CALIBRATION_HOLD_OFFSET_DB - SENSITIVITY_HOLD_RANGE * (lSensitivity / 10.0);

                // substract hold offset
                double holdThresholdDb[16];
                for (int i = 0; i < 16; i++)
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

                std::vector<byte> param;
                std::vector<byte> bytes;

                // write range gate min./max. and delay time, for each:
                // first 2 bytes parameter offset, then 4 bytes value

                param.push_back(OFFSET_PARAM_RANGE_GATE_MIN);
                param.push_back(0);
                param.push_back(mRangeGateMin);
                param.push_back(0);
                param.push_back(0);
                param.push_back(0);
                param.push_back(OFFSET_PARAM_RANGE_GATE_MAX);
                param.push_back(0);
                param.push_back(mRangeGateMax);
                param.push_back(0);
                param.push_back(0);
                param.push_back(0);

                param.push_back(OFFSET_PARAM_DELAY_TIME);
                param.push_back(0);
                bytes = shortToBytes(mDelayTime);
                param.insert(param.end(), bytes.cbegin(), bytes.cend());
                param.push_back(0);
                param.push_back(0);

                sendCommand(CMD_WRITE_MODULE_CONFIG, param);

                logDebugP("triggerThreshold:");
                logIndentUp();

                // write back trigger thresholds, for each:
                // first 2 bytes parameter offset, then 4 bytes value
                param.clear();
                for (int i = 0; i < 16; i++)
                {
                    param.push_back(OFFSET_PARAM_TRIGGERS + (byte)i);
                    param.push_back(0);

                    bytes = intToBytes(dBToRaw(triggerThresholdDb[i]));
                    param.insert(param.end(), bytes.cbegin(), bytes.cend());

                    logDebugP("Gate %i:  %.2f", i, triggerThresholdDb[i]);
                }
                sendCommand(CMD_WRITE_MODULE_CONFIG, param);
                logIndentDown();

                logDebugP("holdThreshold:");
                logIndentUp();

                // write back hold thresholds, for each:
                // first 2 bytes parameter offset, then 4 bytes value
                param.clear();
                for (int i = 0; i < 16; i++)
                {
                    param.push_back(OFFSET_PARAM_HOLDS + (byte)i);
                    param.push_back(0);

                    bytes = intToBytes(dBToRaw(holdThresholdDb[i]));
                    param.insert(param.end(), bytes.cbegin(), bytes.cend());

                    logDebugP("Gate %i:  %.2f", i, holdThresholdDb[i]);
                }
                sendCommand(CMD_WRITE_MODULE_CONFIG, param);
                logIndentDown();

                logIndentDown();

                logDebugP("Sensor calibration finished");

                // reboot module to return to normal (not raw data) operation
                sendCommand(CMD_REBOOT_MODULE);
            }

            result = true;
            break;

        default:
            logDebugP("Unknown packet type: %d", mPacketType);
            break;
    }

    return result;
}

void SensorHLKLD2420::sendCommand(byte command, std::vector<byte> paramter)
{
    std::vector<byte> cmdData = HEADER_COMMAND;

    byte payloadSize = 2 + paramter.size();
    cmdData.push_back(payloadSize);
    cmdData.push_back(0x00);

    cmdData.push_back(command);
    cmdData.push_back(0x00);

    cmdData.insert(cmdData.end(), paramter.cbegin(), paramter.cend());
    cmdData.insert(cmdData.end(), FOOTER.cbegin(), FOOTER.cend());

    SERIAL_HF.write(cmdData.data(), cmdData.size());

    logTraceP("Sending to sensor:");
    logIndentUp();
    logHexTraceP(cmdData.data(), cmdData.size());
    logIndentDown();
}

float SensorHLKLD2420::measureValue(MeasureType iMeasureType)
{
    if (mHfSensorStartupStates < START_FINISHED)
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
    #endif
#endif