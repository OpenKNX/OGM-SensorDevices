#pragma once
// #include "IncludeManager.h"
#ifdef PMMODULE
#ifdef SERIAL_HF
#include "Sensor.h"

// Radar commands, value is index to command table cCommands
#define RadarCmd_ResetSensor 0
#define RadarCmd_ReadSensitivity 5
#define RadarCmd_WriteSensitivity 10
#define RadarCmd_ReadScene 15
#define RadarCmd_WriteScene 20

// result of Presence is a bitfielt uint8_t, where bits are used as following:
// Presence - Bit   0:  0 - no presence, 1 - any presence
// Move     - Bit 1-2:  00 - stay, 01 - move, 10 - closer, 11 - away
// Fall     - Bit 3-4:  00 - no fall, 01 - suspected fall, 10 - real fall
// Alarm    - Bit 5-8:  000 - no alarm, 001-100 alarm level 1-4
// $FF means uninitialized/sensor not available
#define RADAR_PresenceMask 0x01
#define RADAR_PresenceOffset 0
#define RADAR_MoveMask 0x06
#define RADAR_MoveOffset 1
#define RADAR_FallMask 0x18
#define RADAR_FallOffset 3
#define RADAR_AlarmMask 0xE0
#define RADAR_AlarmOffset 5

#define RADAR_NoValue 0xFF

#define RADAR_PresenceNo 0
#define RADAR_Presence 1

#define RADAR_MoveStay 0
#define RADAR_MoveMove 1
#define RADAR_MoveCloser 2
#define RADAR_MoveAway 3

#define RADAR_FallNo 0
#define RADAR_FallSuspected 1
#define RADAR_FallReal 2

#define RADAR_AlarmNo 0
#define RADAR_AlarmLevel1 1
#define RADAR_AlarmLevel2 2
#define RADAR_AlarmLevel3 3
#define RADAR_AlarmLevel4 4

#define START_INIT 0
#define START_SENSOR_ACTIVE 1
#define START_SCENARIO_RECEIVED 2
#define START_SENSITIVITY_RECEIVED 3
#define START_SCENARIO_SET 4
#define START_SENSITIVITY_SET 5
#define START_FINISHED 255


class SensorMR24xxB1 : public Sensor
{
  private:
    //! uartGetPacket state machine states.
    enum PacketStates
    {
        //! Waiting for the synchronisation byte 0x55
        GET_SYNC_STATE = 0,
        //! Reads the whole sensor message in our buffer
        GET_PACKET_STATE,
        //! Checking the info CRC8 checksum.
        CHECK_CRC16D_STATE,
        //! Process the correctly received packet
        PROCESS_PACKET_STATE
    };

    static const uint8_t mBufferSize = 20;
    uint8_t mBuffer[mBufferSize] = {0}; // message buffer
    uint8_t mBufferIndex = 0;
    PacketStates mPacketState = GET_SYNC_STATE;
    
    int8_t mDefaultScenario = 0;
    int8_t mDefaultSensitivity = 8;
    uint8_t mHfSensorStartupStates = 0;

    static void calculateCrcLoHi(uint8_t *iFrame, uint16_t iLen, uint8_t &eCRCLo, uint8_t &eCRCHi);
    void uartGetPacket();
    void getPresenceState(bool iHeartbeat);
    void getMoveState();
    void getMoveSpeed();
    bool getSensorData();
    void printDebugData(const char *iMessage, uint8_t iLength);

  protected:
    uint8_t mPresence = RADAR_NoValue;
    float mMoveSpeed = NO_NUM;
    int8_t mSensitivity = -1;
    int8_t mScenario = -1;
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    bool checkSensorConnection() override;
    float measureValue(MeasureType iMeasureType) override;
    void sendDefaultSensorValues();

  public:
    SensorMR24xxB1(uint16_t iMeasureTypes);
    SensorMR24xxB1(uint16_t iMeasureTypes, uint8_t iAddress);
    virtual ~SensorMR24xxB1() {}

    static bool decodePresenceResult(uint8_t iResult, bool &ePresence, uint8_t &eMove, uint8_t &eFall, uint8_t &eAlarm);
    bool begin() override;
    uint8_t getI2cSpeed() override;
    void defaultSensorParameters(int8_t iScenario, uint8_t iSensitivity);
    // void resetSensor();
    // void writeSensitivity(uint8_t iValue);
    // void readSensitivity();
    // void writeScenario(uint8_t iValue);
    // void readScenario();
    void sendCommand(uint8_t iCommand, int8_t iValue = -1);
};
#endif
#endif
