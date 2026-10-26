#ifdef PMMODULE
    #ifdef HF_SERIAL
        #include "SensorHLKLD2410S.h"
        #include <Arduino.h>
        #include <OpenKNX.h>
        #include <Wire.h>

SensorHLKLD2410S::SensorHLKLD2410S(uint16_t iMeasureTypes, TwoWire *iWire)
    : SensorHLKLD2410S(iMeasureTypes, &Wire, 0) {};

SensorHLKLD2410S::SensorHLKLD2410S(uint16_t iMeasureTypes, TwoWire *iWire, uint8_t iAddress)
    : Sensor(iMeasureTypes, &Wire, iAddress)
{
    pMeasureTypes |= Pres | Sensitivity | Distance;
};

std::string SensorHLKLD2410S::logPrefix()
{
    return "Sensor<HLKLD2410S>";
}

#ifdef HF_USE_SERIALPIO 
    SerialPIO HF_SERIAL(HF_UART_TX_PIN, HF_UART_RX_PIN, 1300);
#endif



void SensorHLKLD2410S::defaultSensorParameters(uint8_t iSensitivity, uint16_t iDelayTime, uint8_t iRangeGateMin, uint8_t iRangeGateMax, uint8_t iStatusReportFrequency, uint8_t iDistanceReportFrequency, uint8_t iResponseSpeed)
{
    mSensitivity = iSensitivity;
    mDelayTime = iDelayTime;

    if (iRangeGateMin <= 15)
        mRangeGateMin = iRangeGateMin;

    if (iRangeGateMax > iRangeGateMin &&
        iRangeGateMax <= 15)
        mRangeGateMax = iRangeGateMax;

    if (iStatusReportFrequency <= 80 &&
        iStatusReportFrequency % 5 == 0)
        mStatusReportFrequency = iStatusReportFrequency;

    if (iDistanceReportFrequency <= 80 &&
        iDistanceReportFrequency % 5 == 0)
        mDistanceReportFrequency = iDistanceReportFrequency;
    
    if (iResponseSpeed == 5 ||
        iResponseSpeed == 10)
        mResponseSpeed = iResponseSpeed;
}

void SensorHLKLD2410S::writeSensitivity(int8_t iSensitivity)
{
    mSensitivity = iSensitivity;

    sendCalibrationData();
}

uint8_t SensorHLKLD2410S::getSensorClass()
{
    return SENS_HLKLD2410S;
}

void SensorHLKLD2410S::sensorLoopInternal()
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
void SensorHLKLD2410S::startupLoop()
{
    switch (mHfSensorStartupState)
    {
        case START_INIT:
            if (lastDetectedRange > NO_NUM)
            {
                pSensorStateDelay = millis();
                mHfSensorStartupState = START_SENSOR_ACTIVE;

                sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE_EXT, PARAM_OPEN_COMMAND_MODE_LENGTH);
            }
            break;
        case START_SENSOR_ACTIVE:
            // Communication is established, we wait for version info from Sensor
            if (moduleVersionMajor > 0)
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
            // We got version, we wait for general read
            if (storedDelayTime > NO_NUM)
            {
                pSensorStateDelay = millis();
                mHfSensorStartupState = START_READ_GENEAL_DONE;
            }
            else if (delayCheck(pSensorStateDelay, 2000))
            {
                pSensorStateDelay = millis();
                sendCommand(CMD_READ_GENERAL_CONFIG, PARAM_READ_GENERAL_CONFIG, PARAM_READ_GENERAL_CONFIG_LENGTH);
            }
            break;
        case START_READ_GENEAL_DONE:
            // Read general is done, we wait for read trigger done
            if (storedTriggerThreshold[0] > 0)
            {
                pSensorStateDelay = millis();
                mHfSensorStartupState = START_READ_TRIGGER_DONE;
            }
            else if (delayCheck(pSensorStateDelay, 2000))
            {
                pSensorStateDelay = millis();
                sendCommand(CMD_READ_TRIGGER_CONFIG, PARAM_READ_TRIGGER_HOLD, PARAM_READ_TRIGGER_HOLD_LENGTH);
            }
            break;
        case START_READ_TRIGGER_DONE:
            // Read general is done, we wait for read trigger done
            if (storedHoldThreshold[0] > 0)
            {
                pSensorStateDelay = millis();
                mHfSensorStartupState = START_READ_HOLD_DONE;
            }
            else if (delayCheck(pSensorStateDelay, 2000))
            {
                pSensorStateDelay = millis();
                sendCommand(CMD_READ_HOLD_CONFIG, PARAM_READ_TRIGGER_HOLD, PARAM_READ_TRIGGER_HOLD_LENGTH);
            }
            break;
        case START_READ_HOLD_DONE:
            // All done, close command mode again
            if (delayCheck(pSensorStateDelay, 50))
            {
                pSensorStateDelay = millis();

                if (calibrationCompleted)
                {
                    mHfSensorStartupState = START_FINISHED;

                    if (mJustAutoCalCompleted)
                    {
                        mJustAutoCalCompleted = false;
                        openknx.flash.save(true);
                        logInfoP("Native auto-calibration complete, thresholds saved");
                        openknx.console.writeDiagnoseKo("HLK cal done");
                        calibrationOnOffTimer = delayTimerInit();
                    }
                    else
                    {
                        sendCalibrationData();
                    }
                }
                else
                {
                    mHfSensorStartupState = START_CALIBRATING;
                    startAutoCalibration();
                }

                sendCommand(CMD_CLOSE_COMMAND_MODE);
            }
            break;
        case START_CALIBRATING:
            if (calibrationCompleted)
            {
                mHfSensorStartupState = START_FINISHED;
            }
            else if (delayCheck(pSensorStateDelay, 300000))
            {
                // if not complete after 5 min., restart calibration
                mHfSensorStartupState = START_READ_HOLD_DONE;
            }
            break;
    }
}

void SensorHLKLD2410S::forceCalibration()
{
    logDebugP("Force sensor calibration");

    pSensorState = Calibrate;
    mHfSensorStartupState = START_CALIBRATING;
    pSensorStateDelay = millis();

    startAutoCalibration();
}

void SensorHLKLD2410S::uartGetPacket()
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

                // logTraceP("Header:");
                // logIndentUp();
                // logHexTraceP(mBuffer, BUFFER_LENGTH);
                // logIndentDown();

                if (BUFFER_LENGTH == 1)
                {
                    // logTraceP("Header:");
                    // logIndentUp();
                    // logHexTraceP(mBuffer, BUFFER_LENGTH);
                    // logIndentDown();

                    if (mBuffer[0] == HEADER_DATA_MINIMAL[0])
                    {
                        mPacketType = DATA_MINIMAL;
                        mPacketState = GET_PACKET_DATA;
                        break;
                    }
                }

                if (BUFFER_LENGTH == HEADER_FOOTER_SIZE)
                {
                    // logTraceP("Header:");
                    // logIndentUp();
                    // logHexTraceP(mBuffer, BUFFER_LENGTH);
                    // logIndentDown();

                    if (memcmp(mBuffer, HEADER_DATA_STANDARD, HEADER_FOOTER_SIZE) == 0)
                    {
                        mPacketType = DATA_STANDARD;
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

                if (mPacketType == DATA_MINIMAL && memcmp(mBuffer + mBufferIndex - sizeof(FOOTER_DATA_MINIMAL), FOOTER_DATA_MINIMAL, sizeof(FOOTER_DATA_MINIMAL)) == 0 ||
                    mPacketType == DATA_STANDARD && memcmp(mBuffer + mBufferIndex - sizeof(FOOTER_DATA_STANDARD), FOOTER_DATA_STANDARD, sizeof(FOOTER_DATA_STANDARD)) == 0 ||
                    mPacketType == COMMAND_RESPONSE && memcmp(mBuffer + mBufferIndex - HEADER_FOOTER_SIZE, FOOTER_COMMAND, HEADER_FOOTER_SIZE) == 0 ||
                    mPacketType == RAW_DATA && memcmp(mBuffer + mBufferIndex - HEADER_FOOTER_SIZE, FOOTER_RAW_DATA, HEADER_FOOTER_SIZE) == 0)
                {
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

int SensorHLKLD2410S::bytesToInt(byte byte0, byte byte1, byte byte2, byte byte3)
{
    return int((unsigned char)(byte3) << 24 |
               (unsigned char)(byte2) << 16 |
               (unsigned char)(byte1) << 8 |
               (unsigned char)(byte0));
}

short SensorHLKLD2410S::bytesToShort(byte byte0, byte byte1)
{
    return short((unsigned char)(byte1) << 8 |
                 (unsigned char)(byte0));
}

float SensorHLKLD2410S::rawToDb(int rawValue)
{
    return float(10 * log10(rawValue));
}

// void SensorHLKLD2410S::rebootSensorSoft()
// {
//     // enter command mode to stop raw data transfer
//     sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE_EXT, PARAM_OPEN_COMMAND_MODE_LENGTH);

//     // reboot module to return to normal (not raw data) operation
//     sendCommand(CMD_REBOOT_MODULE);
//     delay(1000);

//     restartStartupLoop();
// }

void SensorHLKLD2410S::rebootSensorHard()
{
    switchPower(false);
    delay(1000);

    switchPower(true);
    openknx.console.writeDiagnoseKo("HLK reb hard");
}

void SensorHLKLD2410S::switchPower(bool on)
{
    logDebugP("Switch power on: %u", on);
    digitalWrite(HF_POWER_PIN, on ? HIGH : LOW);

    if (on)
    {
        delay(1000);
        restartStartupLoop();
    }
}

void SensorHLKLD2410S::restartStartupLoop()
{
    moduleVersionMajor = 0;
    moduleVersionMinor = 0;
    moduleVersionRevision = 0;
    storedDistanceMin = NO_NUM;
    storedDistanceMax = NO_NUM;
    storedDelayTime = NO_NUM;
    for (uint8_t i = 0; i < 16; i++)
    {
        storedTriggerThreshold[i] = 0;
        storedHoldThreshold[i] = 0;
    }
    pSensorState = Calibrate;
    mHfSensorStartupState = START_INIT;
}

void SensorHLKLD2410S::startAutoCalibration()
{
    calibrationCompleted = false;
    mAutoCalProgress = 0;
    logInfoP("Starting native auto-calibration (120s)...");
    openknx.console.writeDiagnoseKo("HLK cal start");

    uint8_t param[6] = {0x02, 0x00, 0x01, 0x00, 0x78, 0x00}; // trigger=2, hold=1, scan=120s
    sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE_EXT, PARAM_OPEN_COMMAND_MODE_LENGTH);
    delay(100);
    sendCommand(CMD_AUTO_THRESHOLD, param, 6);
    delay(100);
    sendCommand(CMD_CLOSE_COMMAND_MODE);
}

bool SensorHLKLD2410S::getSensorData()
{
    bool result = false;

    std::string rangeString;
    float newDetectedRange;

    bool success;
    std::string successMessage;

    uint8_t payloadSize;

    switch (mPacketType)
    {
        case DATA_MINIMAL:
        {
            // cut 2 bytes (1 header) at the beginning and 1 byte footer at the end
            memmove(mBuffer, mBuffer + 1, BUFFER_LENGTH - 1);
            mBufferIndex -= 2;

            if (BUFFER_LENGTH != 3)
            {
                logDebugP("Invalid data packet size: %d", BUFFER_LENGTH);
                break;
            }

            bool targetDetected = mBuffer[0] > 1;
            if (targetDetected)
            {
                newDetectedRange = bytesToShort(mBuffer[1], mBuffer[2]) / (float)100;

                if (lastDetectedRange != newDetectedRange)
                {
                    lastDetectedRange = newDetectedRange;
                    logDebugP("Presence detected, range: %.1f m", lastDetectedRange);
                }
            }
            else
            {
                if (lastDetectedRange != -1)
                {
                    lastDetectedRange = -1;
                    logDebugP("No presence detected");
                }
            }

            result = true;
            break;
        }

        case DATA_STANDARD:
        {
            // we use this for calibration

                //logHexTraceP(mBuffer, BUFFER_LENGTH);

            payloadSize = mBuffer[4];

            // cut 6 bytes (4 header + 2 size) at the beginning and 4 bytes footer at the end
            memmove(mBuffer, mBuffer + 6, BUFFER_LENGTH - 6);
            mBufferIndex -= 10;

            if (BUFFER_LENGTH != payloadSize)
            {
                logDebugP("Invalid command reponse packet size: %d", BUFFER_LENGTH);
                break;
            }

            // check data type: 0x01 = standard data, 0x03 = auto-threshold progress
            uint8_t dataType = mBuffer[0];
            if (dataType == 0x03)
            {
                mAutoCalProgress = bytesToShort(mBuffer[1], mBuffer[2]);
                logDebugP("Auto-threshold progress: %u%%", mAutoCalProgress / 100);
                if (mAutoCalProgress >= 10000)
                {
                    logInfoP("Auto-calibration reached 100%%, reading back thresholds");
                    mJustAutoCalCompleted = true;
                    calibrationCompleted = true;
                    restartStartupLoop();
                }
                result = true;
                break;
            }
            if (dataType != 0x01)
            {
                logDebugP("Unknown standard frame data type: 0x%02X", dataType);
                break;
            }

            // cut 6 bytes: 1 byte data type, 1 byte target status, 2 bytes target distance, 2 bytes reserved
            memmove(mBuffer, mBuffer + 6, BUFFER_LENGTH - 6);
            mBufferIndex -= 6;

            logTraceP("Standard data frame (gate energy):");
            logIndentUp();
            for (uint8_t i = 0; i < 16; i++)
            {
                int gateVal = bytesToInt(mBuffer[i * 4], mBuffer[i * 4 + 1], mBuffer[i * 4 + 2], mBuffer[i * 4 + 3]);
                if (gateVal > 0)
                    logTraceP("Gate %i: %.2f dB", i, rawToDb(gateVal));
            }
            logIndentDown();

            result = true;
            break;
        }

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

                    // cut 4 bytes at the beginning: 00 01 00 00
                    memmove(mBuffer, mBuffer + 4, BUFFER_LENGTH - 4);
                    mBufferIndex -= 4;

                    moduleVersionMajor = bytesToShort(mBuffer[0], mBuffer[1]);
                    moduleVersionMinor = bytesToShort(mBuffer[2], mBuffer[3]);
                    moduleVersionRevision = bytesToShort(mBuffer[4], mBuffer[5]);
                    logDebugP("Module version: %u.%u.%u", moduleVersionMajor, moduleVersionMinor, moduleVersionRevision);

                    result = true;
                    break;
                case CMD_READ_GENERAL_CONFIG:
                    logDebugP("Received response: CMD_READ_GENERAL_CONFIG (%s)", successMessage.c_str());

                    if (!success)
                        break;

                    // cut 4 bytes at the beginning: 71 01 00 00
                    memmove(mBuffer, mBuffer + 4, BUFFER_LENGTH - 4);
                    mBufferIndex -= 4;
                    
                    logDebugP("Read general config from sensor:");
                    logIndentUp();

                    storedDistanceMax = bytesToInt(mBuffer[0], mBuffer[1], mBuffer[2], mBuffer[3]);
                    logDebugP("distanceMax: %u", storedDistanceMax);

                    storedDistanceMin = bytesToInt(mBuffer[4], mBuffer[5], mBuffer[6], mBuffer[7]);
                    logDebugP("distanceMin: %u", storedDistanceMin);

                    storedDelayTime = bytesToInt(mBuffer[8], mBuffer[9], mBuffer[10], mBuffer[11]);
                    logDebugP("delayTime: %u", storedDelayTime);

                    storedStatusReportFrequency = bytesToInt(mBuffer[12], mBuffer[13], mBuffer[14], mBuffer[15]);
                    logDebugP("statusReportFrequency: %.1f", storedStatusReportFrequency / (float)10);

                    storedDistanceReportFrequency = bytesToInt(mBuffer[16], mBuffer[17], mBuffer[18], mBuffer[19]);
                    logDebugP("distanceReportFrequency: %.1f", storedDistanceReportFrequency / (float)10);

                    storedResponseSpeed = bytesToInt(mBuffer[20], mBuffer[21], mBuffer[22], mBuffer[23]);
                    logDebugP("responseSpeed: %u", storedResponseSpeed);

                    logIndentDown();
                    result = true;
                    break;
                case CMD_WRITE_GENERAL_CONFIG:
                    logDebugP("Received response: CMD_WRITE_GENERAL_CONFIG (%s)", successMessage.c_str());
                    result = true;
                    break;
                case CMD_READ_TRIGGER_CONFIG:
                    logDebugP("Received response: CMD_READ_TRIGGER_CONFIG (%s)", successMessage.c_str());

                    if (!success)
                        break;

                    // cut 4 bytes at the beginning: 73 01 00 00
                    memmove(mBuffer, mBuffer + 4, BUFFER_LENGTH - 4);
                    mBufferIndex -= 4;

                    logDebugP("Read trigger config from sensor:");
                    logIndentUp();

                    logDebugP("triggerThreshold:");
                    logIndentUp();
                    for (uint8_t i = 0; i < 16; i++)
                    {
                        storedTriggerThreshold[i] = bytesToInt(mBuffer[i * 4], mBuffer[i * 4 + 1], mBuffer[i * 4 + 2], mBuffer[i * 4 + 3]);
                        logDebugP("Gate %u: %u", i, storedTriggerThreshold[i]);
                    }
                    logIndentDown();

                    logIndentDown();
                    result = true;
                    break;
                case CMD_WRITE_TRIGGER_CONFIG:
                    logDebugP("Received response: CMD_WRITE_TRIGGER_CONFIG (%s)", successMessage.c_str());
                    result = true;
                    break;
                case CMD_READ_HOLD_CONFIG:
                    logDebugP("Received response: CMD_READ_HOLD_CONFIG (%s)", successMessage.c_str());

                    if (!success)
                        break;

                    // cut 4 bytes at the beginning: 77 01 00 00
                    memmove(mBuffer, mBuffer + 4, BUFFER_LENGTH - 4);
                    mBufferIndex -= 4;

                    logDebugP("Read hold config from sensor:");
                    logIndentUp();

                    logDebugP("holdThreshold:");
                    logIndentUp();
                    for (uint8_t i = 0; i < 16; i++)
                    {
                        storedHoldThreshold[i] = bytesToInt(mBuffer[i * 4], mBuffer[i * 4 + 1], mBuffer[i * 4 + 2], mBuffer[i * 4 + 3]);
                        logDebugP("Gate %u: %u", i, storedHoldThreshold[i]);
                    }
                    logIndentDown();

                    logIndentDown();
                    result = true;
                    break;
                case CMD_WRITE_HOLD_CONFIG:
                    logDebugP("Received response: CMD_WRITE_HOLD_CONFIG (%s)", successMessage.c_str());
                    result = true;
                    break;
                case CMD_DATA_MODE:
                    logDebugP("Received response: CMD_DATA_MODE (%s)", successMessage.c_str());
                    result = true;
                    break;
                default:
                    logDebugP("Unknown response code: %d (%s)", mBuffer[0], successMessage.c_str());
                    break;
            }

            break;

        default:
            logDebugP("Unknown packet type: %d", mPacketType);
            break;
    }

    return result;
}

void SensorHLKLD2410S::sendCalibrationData()
{
    logDebugP("Sending calibration data to sensor");
    logIndentUp();

    uint8_t triggerDb[16];
    uint8_t holdDb[16];

    if (useFactoryDefaultThresholds)
    {
        triggerDb[0] = 48; triggerDb[1] = 42; triggerDb[2] = 36; triggerDb[3] = 34;
        triggerDb[4] = 32; triggerDb[5] = 31; triggerDb[6] = 31; triggerDb[7] = 31;
        triggerDb[8] = 31; triggerDb[9] = 31; triggerDb[10] = 31; triggerDb[11] = 31;
        triggerDb[12] = 31; triggerDb[13] = 31; triggerDb[14] = 31; triggerDb[15] = 31;

        holdDb[0] = 45; holdDb[1] = 42; holdDb[2] = 36; holdDb[3] = 33;
        holdDb[4] = 32; holdDb[5] = 28; holdDb[6] = 28; holdDb[7] = 28;
        holdDb[8] = 28; holdDb[9] = 28; holdDb[10] = 28; holdDb[11] = 28;
        holdDb[12] = 28; holdDb[13] = 28; holdDb[14] = 28; holdDb[15] = 28;

        logDebugP("Factory default thresholds used");
    }
    else
    {
        for (uint8_t i = 0; i < 16; i++)
        {
            triggerDb[i] = mSavedTrigger[i];
            holdDb[i] = mSavedHold[i];
        }
    }

    bool storedCurrentGeneral =
        (storedDistanceMin == mRangeGateMin) &&
        (storedDistanceMax == mRangeGateMax) &&
        (storedDelayTime == mDelayTime) &&
        (storedStatusReportFrequency == mStatusReportFrequency) &&
        (storedDistanceReportFrequency == mDistanceReportFrequency) &&
        (storedResponseSpeed == mResponseSpeed);

    bool storedCurrentCalibration = true;
    for (uint8_t i = 0; i < 16; i++)
    {
        if ((storedTriggerThreshold[i] != triggerDb[i]) ||
            (storedHoldThreshold[i] != holdDb[i]))
        {
            storedCurrentCalibration = false;
            break;
        }
    }

    if (!storedCurrentGeneral || !storedCurrentCalibration)
    {
        sendCommand(CMD_OPEN_COMMAND_MODE, PARAM_OPEN_COMMAND_MODE_EXT, PARAM_OPEN_COMMAND_MODE_LENGTH);
        delay(100);
    }

    if (storedCurrentGeneral)
        logDebugP("Skip writing distance/time to sensor as data is current");
    else
    {
        sendCalibrationDataGeneral(mRangeGateMin, mRangeGateMax, mDelayTime, mStatusReportFrequency, mDistanceReportFrequency, mResponseSpeed);
        openknx.console.writeDiagnoseKo("HLK par send");
    }

    if (storedCurrentCalibration)
        logDebugP("Skip writing calibration to sensor as data is current");
    else
    {
        uint8_t param[96];
        uint8_t offset;

        logDebugP("Writing trigger thresholds:");
        logIndentUp();
        for (uint8_t i = 0; i < 16; i++)
        {
            offset = i * 6;
            param[offset] = i;     param[offset + 1] = 0;
            param[offset + 2] = triggerDb[i];
            param[offset + 3] = 0; param[offset + 4] = 0; param[offset + 5] = 0;
            logDebugP("Gate %u: %u", i, triggerDb[i]);
        }
        sendCommand(CMD_WRITE_TRIGGER_CONFIG, param, 96);
        delay(500);
        logIndentDown();

        logDebugP("Writing hold thresholds:");
        logIndentUp();
        for (uint8_t i = 0; i < 16; i++)
        {
            offset = i * 6;
            param[offset] = i;     param[offset + 1] = 0;
            param[offset + 2] = holdDb[i];
            param[offset + 3] = 0; param[offset + 4] = 0; param[offset + 5] = 0;
            logDebugP("Gate %u: %u", i, holdDb[i]);
        }
        sendCommand(CMD_WRITE_HOLD_CONFIG, param, 96);
        delay(500);
        logIndentDown();

        if (!storedCurrentGeneral)
            openknx.console.writeDiagnoseKo("");
        openknx.console.writeDiagnoseKo("HLK cal send");
        openknx.console.writeDiagnoseKo("");
        rebootSensorHard();
    }

    logIndentDown();

    if (!storedCurrentGeneral || !storedCurrentCalibration)
    {
        restartStartupLoop();
        calibrationOnOffTimer = delayTimerInit();
    }
}

void SensorHLKLD2410S::sendCalibrationDataGeneral(uint8_t rangeGateMin, uint8_t rangeGateMax, uint16_t delayTime, uint8_t statusReportFrequency, uint8_t distanceReportFrequency, uint8_t responseSpeed)
{
    logDebugP("Range gate min.: %d", rangeGateMin);
    logDebugP("Range gate max.: %d", rangeGateMax);
    logDebugP("Delay time: %d", delayTime);
    logDebugP("Status report frequency: %.1f", statusReportFrequency / (float)10);
    logDebugP("Distance report frequency: %.1f", distanceReportFrequency / (float)10);
    logDebugP("Response speed: %d", responseSpeed);

    uint8_t param[36];
    param[0] = OFFSET_PARAM_RANGE_GATE_MAX;
    param[1] = 0;
    param[2] = rangeGateMax;
    param[3] = 0;
    param[4] = 0;
    param[5] = 0;
    param[6] = OFFSET_PARAM_RANGE_GATE_MIN;
    param[7] = 0;
    param[8] = rangeGateMin;
    param[9] = 0;
    param[10] = 0;
    param[11] = 0;

    param[12] = OFFSET_PARAM_DELAY_TIME;
    param[13] = 0;
    param[14] = (uint8_t)(delayTime & 0xFF);
    param[15] = (uint8_t)((delayTime >> 8) & 0xFF);
    param[16] = 0;
    param[17] = 0;

    param[18] = OFFSET_PARAM_STATUS_REPORT_FREQUENCY;
    param[19] = 0;
    param[20] = statusReportFrequency;
    param[21] = 0;
    param[22] = 0;
    param[23] = 0;

    param[24] = OFFSET_PARAM_DISTANCE_REPORT_FREQUENCY;
    param[25] = 0;
    param[26] = distanceReportFrequency;
    param[27] = 0;
    param[28] = 0;
    param[29] = 0;

    param[30] = OFFSET_PARAM_RESPONSE_SPEED;
    param[31] = 0;
    param[32] = responseSpeed;
    param[33] = 0;
    param[34] = 0;
    param[35] = 0;

    sendCommand(CMD_WRITE_GENERAL_CONFIG, param, 36);
    delay(500);
}

void SensorHLKLD2410S::sendCommand(uint8_t command, const uint8_t parameter[], uint8_t parameterLength)
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

    memcpy(cmdData + cmdDataIndex, FOOTER_COMMAND, HEADER_FOOTER_SIZE);

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

float SensorHLKLD2410S::measureValue(MeasureType iMeasureType)
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

bool SensorHLKLD2410S::checkSensorConnection()
{
    return true;
}

bool SensorHLKLD2410S::begin()
{
    logDebugP("Starting sensor HLK-LD2410S (Presence)... ");
    bool lResult = Sensor::begin();
    logResult(lResult);
    return lResult;
}

uint8_t SensorHLKLD2410S::getI2cSpeed()
{
    return 10; // n * 100kHz // no I2C, so we support "all" frequencies :-)
}

void SensorHLKLD2410S::sensorReadFlash(const uint8_t *buffer, const uint16_t size)
{
    if (size == 0)
        return;

    logDebugP("Reading state from flash");
    logIndentUp();

    uint8_t version = openknx.flash.readByte();
    if (version != HLKLD2410S_FLASH_VERSION)
    {
        logDebugP("Flash version %u not supported (expected %u), ignoring", version, HLKLD2410S_FLASH_VERSION);
        return;
    }

    uint32_t magicWord = openknx.flash.readInt();
    if (magicWord != HLKLD2410S_FLASH_MAGIC_WORD)
    {
        logDebugP("Flash content invalid (magic word mismatch)");
        return;
    }

    for (uint8_t i = 0; i < 16; i++)
        mSavedTrigger[i] = openknx.flash.readByte();
    for (uint8_t i = 0; i < 16; i++)
        mSavedHold[i] = openknx.flash.readByte();

    calibrationCompleted = true;

    logDebugP("Calibration data read from flash");
    logIndentDown();
}

void SensorHLKLD2410S::sensorWriteFlash()
{
    if (!calibrationCompleted)
    {
        logDebugP("No data written to flash as calibration not completed");
        return;
    }

    openknx.flash.writeByte(HLKLD2410S_FLASH_VERSION);
    openknx.flash.writeInt(HLKLD2410S_FLASH_MAGIC_WORD);

    for (uint8_t i = 0; i < 16; i++)
        openknx.flash.writeByte((uint8_t)storedTriggerThreshold[i]);
    for (uint8_t i = 0; i < 16; i++)
        openknx.flash.writeByte((uint8_t)storedHoldThreshold[i]);

    logDebugP("Calibration data written to flash");
}

uint16_t SensorHLKLD2410S::sensorFlashSize()
{
    return HLKLD2410S_FLASH_SIZE;
}

void SensorHLKLD2410S::sensorSavePower()
{
    // calibration data is already persisted via openknx.flash.save() after calibration
}

bool SensorHLKLD2410S::sensorRestorePower()
{
    restartStartupLoop();
    return true;
}

void SensorHLKLD2410S::sensorShowHelp()
{
    openknx.console.printHelpLine("hlk ver", "Print firmware version of HLK-LD2410S sensor.");
    openknx.console.printHelpLine("hlk sens", "Print sensitivity defined by ETS app.");
    openknx.console.printHelpLine("hlk rmin", "Print min. range defined by ETS app.");
    openknx.console.printHelpLine("hlk rmax", "Print max. range defined by ETS app.");
    openknx.console.printHelpLine("hlk delay", "Print delay time defined by ETS app.");
    openknx.console.printHelpLine("hlk cal run", "Start native auto-calibration (120s).");
    openknx.console.printHelpLine("hlk cal prog", "Print current auto-calibration progress (0-10000).");
    openknx.console.printHelpLine("hlk cal send", "Re-apply stored calibration thresholds to sensor.");
    openknx.console.printHelpLine("hlk cal def", "Send factory default calibration data to sensor.");
    openknx.console.printHelpLine("hlk reb hard", "Reboot sensor via hardware (cut power).");
}

bool SensorHLKLD2410S::sensorProcessCommand(const std::string iCmd, bool iDebugKo)
{
    bool lResult = false;
    if (iCmd.length() == 5 && iCmd.substr(4, 1) == "h")
    {
        // Command help
        if (iDebugKo)
        {
            openknx.console.writeDiagnoseKo("-> ver");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> sens");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> rmin");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> rmax");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> delay");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> cal run");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> cal prog");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> cal send");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> cal def");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> reb hard");
        }
    }
    else if (iCmd.length() == 7 && iCmd.substr(4, 3) == "ver")
    {
        logDebugP("Module version: %u.%u.%u", moduleVersionMajor, moduleVersionMinor, moduleVersionRevision);
        if (iDebugKo)
            openknx.console.writeDiagnoseKo("HLK ver %u.%u.%u", moduleVersionMajor, moduleVersionMinor, moduleVersionRevision);
        lResult = true;
    }
    else if (iCmd.length() == 8 && iCmd.substr(4, 4) == "sens")
    {
        logInfoP("Sensitivity: %u", mSensitivity);
        if (iDebugKo)
            openknx.console.writeDiagnoseKo("HLK sens %u", mSensitivity);
        lResult = true;
    }
    else if (iCmd.length() == 8 && iCmd.substr(4, 4) == "rmin")
    {
        logInfoP("Range gate min.: %u", mRangeGateMin);
        if (iDebugKo)
            openknx.console.writeDiagnoseKo("HLK rmin %u", mRangeGateMin);
        lResult = true;
    }
    else if (iCmd.length() == 8 && iCmd.substr(4, 4) == "rmax")
    {
        logInfoP("Range gate max.: %u", mRangeGateMax);
        if (iDebugKo)
            openknx.console.writeDiagnoseKo("HLK rmax %u", mRangeGateMax);
        lResult = true;
    }
    else if (iCmd.length() == 9 && iCmd.substr(4, 5) == "delay")
    {
        logInfoP("Delay time: %u", mDelayTime);
        if (iDebugKo)
            openknx.console.writeDiagnoseKo("HLK delay %u", mDelayTime);
        lResult = true;
    }
    else if (iCmd.length() == 12 && iCmd.substr(4, 8) == "cal prog")
    {
        logInfoP("Auto-calibration progress: %u / 10000", mAutoCalProgress);
        if (iDebugKo)
            openknx.console.writeDiagnoseKo("HLK cal %u%%", mAutoCalProgress / 100);
        lResult = true;
    }
    else if (iCmd.length() == 11 && iCmd.substr(4, 7) == "cal run")
    {
        forceCalibration();
        lResult = true;
    }
    else if (iCmd.length() == 12 && iCmd.substr(4, 8) == "cal send")
    {
        sendCalibrationData();
        lResult = true;
    }
    else if (iCmd.length() == 11 && iCmd.substr(4, 7) == "cal def")
    {
        useFactoryDefaultThresholds = true;
        sendCalibrationData();
        lResult = true;
    }
    else if (iCmd.length() == 12 && iCmd.substr(4, 8) == "reb hard")
    {
        rebootSensorHard();
        lResult = true;
    }
    return lResult;
}

bool SensorHLKLD2410S::handleFunctionProperty(uint8_t *iData, uint8_t *eResultData, uint8_t &eResultLength)
{
    // dispatch ets online commands
    switch (iData[0])
    {
        case 1:
            return getCalibrationData(iData, eResultData, eResultLength);
        case 2:
            return doCalibration(iData, eResultData, eResultLength);
        case 3:
            return setCalibrationData(iData, eResultData, eResultLength);
        default:
            return false;
    }
}

bool SensorHLKLD2410S::getCalibrationData(uint8_t *iData, uint8_t *eResultData, uint8_t &eResultLength)
{
    uint8_t lRequestedData = iData[1];
    eResultData[0] = 0;

    switch (lRequestedData)
    {
        case 1: // trigger thresholds
            for (uint8_t i = 0; i < 16; i++)
            {
                uint16_t lValue = mSavedTrigger[i] * 100;
                eResultData[i * 2 + 1] = lValue >> 8;
                eResultData[i * 2 + 2] = lValue & 0xFF;
            }
            eResultLength = 33;
            return true;
        case 2: // hold thresholds
            for (uint8_t i = 0; i < 16; i++)
            {
                uint16_t lValue = mSavedHold[i] * 100;
                eResultData[i * 2 + 1] = lValue >> 8;
                eResultData[i * 2 + 2] = lValue & 0xFF;
            }
            eResultLength = 33;
            return true;
        default:
            return false;
    }
}

bool SensorHLKLD2410S::setCalibrationData(uint8_t *iData, uint8_t *eResultData, uint8_t &eResultLength)
{
    eResultData[0] = 1;
    eResultLength = 1;
    return false;
}

bool SensorHLKLD2410S::doCalibration(uint8_t *iData, uint8_t *eResultData, uint8_t &eResultLength)
{
    eResultData[0] = 0;
    eResultData[1] = 0;
    eResultLength = 2;
    switch (iData[1])
    {
        case 1:
        case 4:
            forceCalibration();
            return true;
        case 2:
            // return progress (0-10000) as big-endian uint16
            eResultData[1] = mAutoCalProgress >> 8;
            eResultData[2] = mAutoCalProgress & 0xFF;
            eResultLength = 3;
            return true;
        default:
            eResultData[0] = 1;
            return false;
    }
}

    #endif
#endif