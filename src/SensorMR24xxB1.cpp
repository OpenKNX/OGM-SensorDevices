// #include "IncludeManager.h"
#ifdef PMMODULE
#ifdef SERIAL_HF
#include <Arduino.h>
#include <Wire.h>
#include "SensorMR24xxB1.h"

#define BUFFER_POS_LENGTH 1  // position of packet length info in buffer
#define BUFFER_POS_FCODE 3   // position of function code in buffer
#define BUFFER_POS_AD1 4     // position of address code 1 in packet
#define BUFFER_POS_AD2 5     // position of address code 2 in packet
#define BUFFER_POS_DATA 6    // position of data start in packet

#define MESSAGE_HEAD 0x55

// function codes
#define F_READ_COMMANDS 0x01   // read command
#define F_WRITE_COMMANDS 0x02  // copy order
#define F_ANSWER_COMMANDS 0x03 // passive reporting of orders
#define F_ACTIVE_COMMANDS 0x04 // proactive reporting of commands
#define F_FALL_COMMANDS 0x06   // fall radar data reporting

// address code 1
#define AD1_REPORT_IDENTIFICATION 0x01  //
#define AD1_REPORT_RADAR_INFO 0x03
#define AD1_REPORT_SYSTEM_INFO 0x04
#define AD1_REPORT_OTHER_INFO 0x05
#define AD1_REPORT_FALL_ALARM 0x01

// address code 2
#define AD2_DEVICE_ID 0x01
#define AD2_SOFTWARE_VERSION 0x02
#define AD2_HARDWARE_VERSION 0x03
#define AD2_PROTOCOL_VERSION 0x04
#define AD2_PRESENCE_STATUS 0x05  // environment status
#define AD2_SIGNS_PARAMETERS 0x06 // what is this?
#define AD2_APPROACH_AWAY 0x07
#define AD2_THRESHOLD_GEAR 0x0C   // values 1-10 (0x01-0x0A)
#define AD2_SCENE_SETTING 0x10
#define AD2_FEEDBACK_OTA_START 0x08
#define AD2_FEEDBACK_OTA_RUNNING 0x09
#define AD2_FEEDBACK_OTA_START 0x08
#define AD2_FALL_FUNCTION_SWITCH 0x0B
#define AD2_FALL_ALARM_TIME 0x0C
#define AD2_FALL_SENSIVITY 0x0E
#define AD2_HEARTBEAT 0x01
#define AD2_ABNORMAL_RESET 0x02
#define AD2_FALL_ALARM 0x01
#define AD2_FALL_DWELL_ALARM 0x02

#define AD3_SCENE_DEFAULT 0          // Default mode
#define AD3_SCENE_AREA_TILT 1        // Area detection (top loading)
#define AD3_SCENE_BATH_TOP 2         // Bathroom (top mounted)
#define AD3_SCENE_BEDROOM_TILT 3     // Bedroom(top loading)
#define AD3_SCENE_LIVINGROOM_TOP 4   // Living room (top mounted)
#define AD3_SCENE_OFFICE_TILT 5      // Office (top loading)
#define AD3_SCENE_HOTEL_TILT 6       // Hotel (top loading)

typedef union
{
    uint8_t Byte[4];
    float Float;
} Float_Byte;

float toFloat(uint8_t *iData)
{
    Float_Byte fb;
    fb.Byte[0] = iData[0];
    fb.Byte[1] = iData[1];
    fb.Byte[2] = iData[2];
    fb.Byte[3] = iData[3];
    return fb.Float;
}

SensorMR24xxB1::SensorMR24xxB1(uint16_t iMeasureTypes)
    : SensorMR24xxB1(iMeasureTypes, 0){};

SensorMR24xxB1::SensorMR24xxB1(uint16_t iMeasureTypes, uint8_t iAddress)
    : Sensor(iMeasureTypes, iAddress)
    {
        gMeasureTypes |= Pres | Speed | Sensitivity | Scenario;
    };

void SensorMR24xxB1::defaultSensorParameters(int8_t iScenario, uint8_t iSensitivity)
{
    mDefaultScenario = iScenario; 
    mDefaultSensitivity = iSensitivity;
}

uint8_t SensorMR24xxB1::getSensorClass()
{
    return SENS_MR24xxB1;
}

void SensorMR24xxB1::sensorLoopInternal()
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
void SensorMR24xxB1::sendDefaultSensorValues()
{
    switch (mHfSensorStartupStates)
    {
        case START_INIT:
            if (mMoveSpeed > NO_NUM)
            {
                pSensorStateDelay = millis();
                mHfSensorStartupStates = START_SENSOR_ACTIVE;
            }
            break;
        case START_SENSOR_ACTIVE:
            // Communication is established, we wait for scenario info from Sensor
            if (delayCheck(pSensorStateDelay, 1000))
            {
                pSensorStateDelay = millis();
                if (mScenario >= 0)
                {
                    mHfSensorStartupStates = START_SCENARIO_RECEIVED;
                    SERIAL_DEBUG.print("Got Scenario at startup: ");
                    SERIAL_DEBUG.println(mScenario);
                } 
                else
                {
                    sendCommand(RadarCmd_ReadScene);
                }
            }
            break;
        case START_SCENARIO_RECEIVED:
            // We got Scenario, we wait for sensitivity info from Sensor
            if (delayCheck(pSensorStateDelay, 1000))
            {
                pSensorStateDelay = millis();
                if (mSensitivity >= 0)
                {
                    mHfSensorStartupStates = START_SENSITIVITY_RECEIVED;
                    SERIAL_DEBUG.print("Got Sensitivity at startup: ");
                    SERIAL_DEBUG.println(mSensitivity);
                } 
                else
                {
                    sendCommand(RadarCmd_ReadSensitivity);
                }
            }
            break;
        case START_SENSITIVITY_RECEIVED:
            // We got Scenario and sensitivity, we set now default scenario, if it differs from current
            if (delayCheck(pSensorStateDelay, 5000))
            {
                pSensorStateDelay = millis();
                mHfSensorStartupStates = START_SCENARIO_SET;
                if (mDefaultScenario >= 0 && mScenario != mDefaultScenario) 
                {
                    SERIAL_DEBUG.print("Setting Scenario: ");
                    SERIAL_DEBUG.println(mDefaultScenario);
                    sendCommand(RadarCmd_WriteScene, mDefaultScenario);
                }
            }
            break;
        case START_SCENARIO_SET:
            // We got sensitivity and we set a scenario, we set now default sensitivity, if it differs from current
            if (delayCheck(pSensorStateDelay, 1000))
            {
                pSensorStateDelay = millis();
                mHfSensorStartupStates = START_SENSITIVITY_SET;
                if (mDefaultSensitivity > 0 && mSensitivity != mDefaultSensitivity) 
                {
                    SERIAL_DEBUG.print("Setting Sensitivity: ");
                    SERIAL_DEBUG.println(mDefaultSensitivity);
                    sendCommand(RadarCmd_WriteSensitivity, mDefaultSensitivity);
                }
            }
        case START_SENSITIVITY_SET:
            // we finished default settings for hf sensor, lets check if everything is as expected
            if (delayCheck(pSensorStateDelay, 1000))
            {
                pSensorStateDelay = millis();
                if ((mDefaultScenario < 0 || mScenario == mDefaultScenario) && (mDefaultSensitivity <= 0 || mSensitivity == mDefaultSensitivity)) 
                {
                    SERIAL_DEBUG.println("Setting HF-Sensor defaults OK!");
                    mHfSensorStartupStates = START_FINISHED;
                }
                else
                {
                    SERIAL_DEBUG.println("Setting HF-Sensor defaults FAILED, retrying!!!!!!!!!");
                    mHfSensorStartupStates = START_SENSITIVITY_RECEIVED;
                }
            }
            break;
        default:
            mHfSensorStartupStates = START_FINISHED;
            break;
    }
}

// Packet: 
// 0x55 - Message head
// ll   - data length low
// hh   - data length high (currenty 0x00)
// cmd  - command
// adr0 - 
// adr1 - 
void SensorMR24xxB1::uartGetPacket()
{
    uint8_t lCRCLo;
    uint8_t lCRCHi;
    uint8_t lRxByte;
    switch (mPacketState)
    {
        // Waiting for packet sync byte 0x55
        case GET_SYNC_STATE:
            // SERIAL_DEBUG.println(u8RxByte);
            if (SERIAL_HF.available() > 0 && SERIAL_HF.readBytes(&lRxByte, 1) == 1)
                if (lRxByte == MESSAGE_HEAD)
                {
                    mPacketState = GET_PACKET_STATE;
                    mBuffer[0] = lRxByte;
                    mBufferIndex = 1;
                }
            break;
        case GET_PACKET_STATE:
            if (SERIAL_HF.available() > 0 && SERIAL_HF.readBytes(&lRxByte, 1) == 1)
            {
                mBuffer[mBufferIndex++] = lRxByte;
                if (mBufferIndex > mBuffer[BUFFER_POS_LENGTH])
                    mPacketState = CHECK_CRC16D_STATE;
            }
            else
                mPacketState = GET_SYNC_STATE;
            break;
        case CHECK_CRC16D_STATE:
            calculateCrcLoHi(mBuffer, mBufferIndex - 2, lCRCLo, lCRCHi);
            if (mBuffer[mBufferIndex - 2] == lCRCLo && mBuffer[mBufferIndex - 1] == lCRCHi)
                mPacketState = PROCESS_PACKET_STATE;
            else
                mPacketState = GET_SYNC_STATE;
            break;
        case PROCESS_PACKET_STATE:
            getSensorData();
            mPacketState = GET_SYNC_STATE;
            break;
    }
}

float SensorMR24xxB1::measureValue(MeasureType iMeasureType)
{
    if (mHfSensorStartupStates < START_FINISHED)
        return NO_NUM;
    switch (iMeasureType)
    {
    case Pres:
        // This is a compromize, return values of sensors should support different types.
        if (mPresence < 0xFF)
            return (float)mPresence;
        break;
    case Speed:
        return mMoveSpeed;
        break;
    case Sensitivity:
        if (mSensitivity >= 0)
            return mSensitivity;
        break;
    case Scenario:
        if (mScenario >= 0)
            return mScenario;
        break;
    default:
        break;
    }
    return NO_NUM;
}

bool SensorMR24xxB1::checkSensorConnection()
{
    return true;
}

bool SensorMR24xxB1::begin()
{
    printDebug("Starting sensor MR24xxB1 (Presence)... ");
    bool lResult = Sensor::begin();
    printResult(lResult);
    return lResult;
}

uint8_t SensorMR24xxB1::getI2cSpeed()
{
    return 10; // n * 100kHz // no I2C, so we support "all" frequencies :-)
}

void SensorMR24xxB1::getPresenceState(bool iHeartbeat)
{
    if (iHeartbeat)
        SERIAL_DEBUG.print("H-");
    if (mBuffer[BUFFER_POS_DATA] == 0x00 && mBuffer[BUFFER_POS_DATA + 1] == 0xFF && mBuffer[BUFFER_POS_DATA + 2] == 0xFF)
    {
        SERIAL_DEBUG.println("Unoccupied ");
        mPresence = RADAR_PresenceNo;
    }
    else if (mBuffer[BUFFER_POS_DATA] == 0x01 && mBuffer[BUFFER_POS_DATA + 1] == 0x00 && mBuffer[BUFFER_POS_DATA + 2] == 0xFF)
    {
        SERIAL_DEBUG.println("Someone is stationary");
        mPresence = (RADAR_MoveStay << RADAR_MoveOffset) | RADAR_Presence;
    }
    else if (mBuffer[BUFFER_POS_DATA] == 0x01 && mBuffer[BUFFER_POS_DATA + 1] == 0x01 && mBuffer[BUFFER_POS_DATA + 2] == 0x01)
    {
        SERIAL_DEBUG.println("Some people exercise");
        mPresence = (RADAR_MoveMove << RADAR_MoveOffset) | RADAR_Presence;
    }
}

void SensorMR24xxB1::getMoveState()
{
    if (mBuffer[BUFFER_POS_DATA] == 0x01 && mBuffer[BUFFER_POS_DATA + 1] == 0x01 && mBuffer[BUFFER_POS_DATA + 2] == 0x01)
    {
        SERIAL_DEBUG.println("Moving (same distance)");
        mPresence = (RADAR_MoveMove << RADAR_MoveOffset) | RADAR_Presence;
    }
    else if (mBuffer[BUFFER_POS_DATA] == 0x01 && mBuffer[BUFFER_POS_DATA + 1] == 0x01 && mBuffer[BUFFER_POS_DATA + 2] == 0x02)
    {
        SERIAL_DEBUG.println("Moving closer");
        mPresence = (RADAR_MoveCloser << RADAR_MoveOffset) | RADAR_Presence;
    }
    else if (mBuffer[BUFFER_POS_DATA] == 0x01 && mBuffer[BUFFER_POS_DATA + 1] == 0x01 && mBuffer[BUFFER_POS_DATA + 2] == 0x03)
    {
        SERIAL_DEBUG.println("Moving away");
        mPresence = (RADAR_MoveAway << RADAR_MoveOffset) | RADAR_Presence;
    }
}

void SensorMR24xxB1::getMoveSpeed()
{
    mMoveSpeed = toFloat(&mBuffer[BUFFER_POS_DATA]);
    SERIAL_DEBUG.print("Move speed: ");
    SERIAL_DEBUG.println(mMoveSpeed);
}

void SensorMR24xxB1::printDebugData(const char *iMessage, uint8_t iLength)
{
    SERIAL_DEBUG.print(iMessage);
    for (int i = 0; i < iLength; i++)
    {
        SERIAL_DEBUG.print(mBuffer[BUFFER_POS_DATA + i]);
        SERIAL_DEBUG.print((" "));
    }
    SERIAL_DEBUG.println(F(" "));
}

bool SensorMR24xxB1::getSensorData()
{
    bool lResult = false;
    uint8_t *data = &mBuffer[BUFFER_POS_DATA];
    switch (mBuffer[BUFFER_POS_FCODE])
    {
        case F_ANSWER_COMMANDS: // Passive reporting of commands
            switch (mBuffer[BUFFER_POS_AD1])
            {
                case AD1_REPORT_IDENTIFICATION: // Reporting module identification
                    switch (mBuffer[BUFFER_POS_AD2])
                    {
                        case AD2_DEVICE_ID: // Device ID 12 Bytes
                            printDebugData("Device ID: ", 12);
                            break;                 // ENDE 0x01: //Device ID 12Bytes
                        case AD2_SOFTWARE_VERSION: // SW Version 10Bytes
                            printDebugData("SW_Version: ", 10);
                            break;                 // ENDE 0x02: //SW Version 10Bytes
                        case AD2_HARDWARE_VERSION: // HW Version 8Bytes
                            printDebugData("HW-Version: ", 8);
                            break;                 // ENDE 0x03: //HW Version 8Bytes
                        case AD2_PROTOCOL_VERSION: // Protocol version 8Bytes
                            printDebugData("Prot Version: ", 8);
                            break; // ENDE 0x04: //Protocol version 8Bytes
                    }
                    break;                  // ENDE 0x01: //Reporting module identification
                case AD1_REPORT_RADAR_INFO: // Report radar information
                    switch (mBuffer[BUFFER_POS_AD2])
                    {
                        case AD2_PRESENCE_STATUS: // Environment status
                            getPresenceState(false);
                            break;                 // ENDE 0x05: //Environment status
                        case AD2_SIGNS_PARAMETERS: // Signs parameters
                            getMoveSpeed();
                            break; // ENDE 0x06: //Signs parameters
                    }
                    break;                   // ENDE 0x03: //Report radar information
                case AD1_REPORT_SYSTEM_INFO: // Reporting system information
                    switch (mBuffer[BUFFER_POS_AD2])
                    {
                        case AD2_THRESHOLD_GEAR: // Threshold gear
                            mSensitivity = data[0];
                            SERIAL_DEBUG.print("Sensitivity: ");
                            SERIAL_DEBUG.println(mSensitivity);
                            break;              // ENDE 0x0C: //Threshold gear
                        case AD2_SCENE_SETTING: // Scene setting
                            mScenario = data[0];
                            SERIAL_DEBUG.print("Scenario ");
                            SERIAL_DEBUG.print(mScenario);
                            SERIAL_DEBUG.print(": ");
                            switch (mScenario)
                            {
                                case AD3_SCENE_DEFAULT: // Default mode
                                    SERIAL_DEBUG.println("Default mode");
                                    break;
                                case AD3_SCENE_AREA_TILT: // Area detection (top loading)
                                    SERIAL_DEBUG.println("Area detection (top loading)");
                                    break;
                                case AD3_SCENE_BATH_TOP: // Bathroom (top mounted)
                                    SERIAL_DEBUG.println("Bathroom (top mounted)");
                                    break;
                                case AD3_SCENE_BEDROOM_TILT: // Bedroom (top loading)
                                    SERIAL_DEBUG.println("Bedroom (top loading)");
                                    break;
                                case AD3_SCENE_LIVINGROOM_TOP: // Living room (top mounted)
                                    SERIAL_DEBUG.println("Living room (top mounted)");
                                    break;
                                case AD3_SCENE_OFFICE_TILT: // Office (top loading)
                                    SERIAL_DEBUG.println("Office (top loading)");
                                    break;
                                case AD3_SCENE_HOTEL_TILT: // Hotel (top loading)
                                    SERIAL_DEBUG.println("Hotel (top loading)");
                                    break;
                            }
                            break; // ENDE 0x10: //Scene setting
                    }
                    break;                  // ENDE 0x04: //Reporting system information
                case AD1_REPORT_OTHER_INFO: // Report additional information
                    switch (mBuffer[BUFFER_POS_AD2])
                    {
                        case AD2_FEEDBACK_OTA_START: // Feedback OTA Upgrade Start
                            SERIAL_DEBUG.print("OTA Upgrade: ");
                            if (data[0] == 0)
                                SERIAL_DEBUG.println("Failue");
                            else
                                SERIAL_DEBUG.println("Success");
                            break;                     // ENDE 0x08: //Feedback OTA Upgrade Start
                        case AD2_FEEDBACK_OTA_RUNNING: // Feedback OTA transmission
                            break;                     // ENDE 0x09: //Feedback OTA transmission
                            SERIAL_DEBUG.println(data[0], HEX);
                        case AD2_FALL_FUNCTION_SWITCH: // Fall function switch
                            SERIAL_DEBUG.print("Fall function: ");
                            if (data[0] == 0)
                                SERIAL_DEBUG.println("OFF");
                            else
                                SERIAL_DEBUG.println("ON");
                            break;                // ENDE 0x0B: //Fall function switch
                        case AD2_FALL_ALARM_TIME: // Fall alarm time
                            break;                // ENDE 0x0C: //Fall alarm time
                        case AD2_FALL_SENSIVITY:  // Response to fall sensitivity setting
                            break;                // ENDE 0x0E: //Response to fall sensitivity settingandre
                    }
                    break; // ENDE 0x05: //Report additional information
            }
            break; // ENDE 0x03 Report radar information
            break;
        case F_ACTIVE_COMMANDS: // Proactive reporting of commands
            switch (mBuffer[BUFFER_POS_AD1])
            {
                case AD1_REPORT_IDENTIFICATION: // Reporting module identification
                    switch (mBuffer[BUFFER_POS_AD2])
                    {
                        case AD2_SOFTWARE_VERSION: // SW Version 10Bytes
                            printDebugData("SW_Version: ", 10);
                            break; // ENDE 0x02: //SW Version 10Bytes
                    }
                    break;                  // ENDE 0x01: // Reporting module identification
                case AD1_REPORT_RADAR_INFO: // Report radar information
                    switch (mBuffer[BUFFER_POS_AD2])
                    {
                        case AD2_PRESENCE_STATUS: // Environment status
                            getPresenceState(false);
                            break;              // ENDE 0x05 Environment status
                        case AD2_APPROACH_AWAY: // Approaching away state
                            getMoveState();
                            break; // ENDE 0x07 Approaching away state
                        case AD2_SIGNS_PARAMETERS: // Signs parameters
                            getMoveSpeed();
                            break; // ENDE 0x06: //Signs parameters
                    }
                    break;                  // ENDE 0x03 Report radar information
                case AD1_REPORT_OTHER_INFO: // Report other information
                    switch (mBuffer[BUFFER_POS_AD2])
                    {
                        case AD2_HEARTBEAT: // Heartbeat Pack
                            getPresenceState(true);
                            break; // ENDE 0x01 Heartbeat Pack
                    }
                    break; // ENDE 0x05 Report other information
            }
            break; // ENDE CMD 0x04  Proactive reporting of commands
    }
    return lResult;
}

const uint8_t cCRCHi[256] = 
{
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

const uint8_t cCRCLo[256] = 
{
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};

// static
void SensorMR24xxB1::calculateCrcLoHi(uint8_t *iFrame, uint16_t iLen, uint8_t &eCRCLo, uint8_t &eCRCHi)
{
    eCRCHi = 0xFF;
    eCRCLo = 0xFF;
    int lIndex = 0;
    while (iLen--)
    {
        lIndex = eCRCLo ^ *(iFrame++);
        eCRCLo = (uint8_t)(eCRCHi ^ cCRCHi[lIndex]);
        eCRCHi = cCRCLo[lIndex];
    }
}

// commands sent to HF sensor
// if length longer than command data (in writes): Parameter must be added 
uint8_t cCommands[30] =
    {
         0x07, 0x00, 0x02, 0x05, 0x04, /* resetSensor */
         0x07, 0x00, 0x01, 0x04, 0x0C, /* readSensitivity */
         0x08, 0x00, 0x02, 0x04, 0x0C, /* writeSensitivity(value) */
         0x07, 0x00, 0x01, 0x04, 0x10, /* readScene */
         0x08, 0x00, 0x02, 0x04, 0x10  /* writeScene(value) */
    };

void SensorMR24xxB1::sendCommand(uint8_t iCommandId, int8_t iValue /* = -1 */)
{
    uint8_t lFrame[10] = {0};
    uint8_t *lFramePtr = lFrame;
    uint8_t lCRCHi;
    uint8_t lCRCLo;
    uint8_t *lCommandPtr = cCommands + iCommandId;
    // build frame, header first
    *(lFramePtr++) = MESSAGE_HEAD;
    // then command itself
    for (uint8_t lCount = 0; lCount < 5; lCount++)
        *lFramePtr++ = *lCommandPtr++; // yes, this syntax is intended
    // now parameter, if provided
    if (iValue >= 0)
        *lFramePtr++ = iValue;
    calculateCrcLoHi(lFrame, lFramePtr - lFrame, lCRCLo, lCRCHi);
    *lFramePtr++ = lCRCLo;
    *lFramePtr++ = lCRCHi;
    SERIAL_HF.write(lFrame, lFramePtr - lFrame);
#ifdef debugOutput
    SERIAL_DEBUG.println(F("Reboot Sensor!"));
    for (int i = 0; i < 8; i++)
    {
        SERIAL_DEBUG.print(sendData[i], HEX);
        SERIAL_DEBUG.print(" ");
    }
    SERIAL_DEBUG.println(" ");
#endif
}

bool SensorMR24xxB1::decodePresenceResult(uint8_t iResult, bool &ePresence, uint8_t &eMove, uint8_t &eFall, uint8_t &eAlarm) 
{
    ePresence = iResult & RADAR_PresenceMask;
    eMove = (iResult & RADAR_MoveMask) >> RADAR_MoveOffset;
    eFall = (iResult & RADAR_FallMask) >> RADAR_FallOffset;
    eAlarm = (iResult & RADAR_AlarmMask) >> RADAR_AlarmOffset;
    return (iResult < RADAR_NoValue);
}
#endif
#endif