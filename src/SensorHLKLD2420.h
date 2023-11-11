#pragma once
// #include "IncludeManager.h"
#ifdef PMMODULE
#ifdef SERIAL_HF
#include <string>
#include <vector>
#include "Sensor.h"

#define HEADER_FOOTER_SIZE 4

const std::vector<byte> HEADER_ON = {(byte)0x4F, (byte)0x4E, (byte)0x0D, (byte)0x0A};
const std::vector<byte> HEADER_OFF = {(byte)0x4F, (byte)0x46, (byte)0x46, (byte)0x0D};
const std::vector<byte> HEADER_COMMAND = {(byte)0xFD, (byte)0xFC, (byte)0xFB, (byte)0xFA};
const std::vector<byte> FOOTER = {(byte)0x04, (byte)0x03, (byte)0x02, (byte)0x01};

const std::vector<byte> HEADER_RAW_DATA = {(byte)0xAA, (byte)0xBF, (byte)0x10, (byte)0x14};
const std::vector<byte> FOOTER_RAW_DATA = {(byte)0xFD, (byte)0xFC, (byte)0xFB, (byte)0xFA};

#define CMD_OPEN_COMMAND_MODE 0xFF
#define CMD_CLOSE_COMMAND_MODE 0xFE
#define CMD_READ_VERSION 0x00
#define CMD_REBOOT_MODULE 0x68
#define CMD_READ_MODULE_CONFIG 0x08
#define CMD_WRITE_MODULE_CONFIG 0x07
#define CMD_RAW_DATA_MODE 0x12

#define OFFSET_PARAM_TRIGGERS 0x10
#define OFFSET_PARAM_HOLDS 0x20

const std::vector<byte> PARAM_OPEN_COMMAND_MODE = {(byte)0x01, (byte)0x00};
const std::vector<byte> PARAM_READ_DISTANCE_TRIGGER = {(byte)0x00, (byte)0x00, (byte)0x01, (byte)0x00, (byte)0x10, (byte)0x00, (byte)0x11, (byte)0x00, (byte)0x12, (byte)0x00, (byte)0x13, (byte)0x00, (byte)0x14, (byte)0x00, (byte)0x15, (byte)0x00, (byte)0x16, (byte)0x00, (byte)0x17, (byte)0x00, (byte)0x18, (byte)0x00, (byte)0x19, (byte)0x00, (byte)0x1A, (byte)0x00, (byte)0x1B, (byte)0x00, (byte)0x1C, (byte)0x00, (byte)0x1D, (byte)0x00, (byte)0x1E, (byte)0x00, (byte)0x1F, (byte)0x00};
const std::vector<byte> PARAM_READ_DELAY_MAINTAIN = {(byte)0x04, (byte)0x00, (byte)0x20, (byte)0x00, (byte)0x21, (byte)0x00, (byte)0x22, (byte)0x00, (byte)0x23, (byte)0x00, (byte)0x24, (byte)0x00, (byte)0x25, (byte)0x00, (byte)0x26, (byte)0x00, (byte)0x27, (byte)0x00, (byte)0x28, (byte)0x00, (byte)0x29, (byte)0x00, (byte)0x2A, (byte)0x00, (byte)0x2B, (byte)0x00, (byte)0x2C, (byte)0x00, (byte)0x2D, (byte)0x00, (byte)0x2E, (byte)0x00, (byte)0x2F, (byte)0x00};
const std::vector<byte> PARAM_RAW_DATA_MODE = {(byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00, (byte)0x00};

#define CALIBRATION_VALUE_COUNT 100
#define CALIBRATION_TRIGGER_OFFSET_DB 3
#define CALIBRATION_HOLD_OFFSET_DB 1.5


#define START_INIT 0
#define START_SENSOR_ACTIVE 1
#define START_VERSION_RECEIVED 2
#define START_READ1_DONE 3
#define START_READ2_DONE 4
#define START_FINISHED 255


class SensorHLKLD2420 : public Sensor
{
  private:
    //! uartGetPacket state machine states.
    enum PacketStates
    {
        //! Waiting for the synchronisation byte 0x55
        GET_SYNC_STATE = 0,
        //! Reads message data in our buffer
        GET_PACKET_DATA,
        //! Process the correctly received packet
        PROCESS_PACKET_STATE
    };

    enum PacketType
    {
        //! Presence detection on
        ON = 0,
        //! Presence detection off
        OFF,
        //! Command packet
        COMMAND_RESPONSE,
        //! Raw data packet
        RAW_DATA
    };

    std::vector<byte> mBuffer; // message buffer
    int mBufferIndex = 0;
    PacketStates mPacketState = GET_SYNC_STATE;
    PacketType mPacketType;
    
    std::string moduleVersion;
    float lastDetectedRange = NO_NUM;
    int minDistance = NO_NUM;
    int maxDistance = NO_NUM;
    int delayTime = NO_NUM;
    int triggerThreshold[16];
    int holdThreshold[16];

    uint32_t rawDataLastRecordingReceived = 0;
    double rawDataRangeAverage[16];
    int rawDataRecordingCount = 0;
    
    int8_t mDefaultScenario = 0;
    int8_t mDefaultSensitivity = 8;
    uint8_t mHfSensorStartupStates = 0;

    void uartGetPacket();
    int bytesToInt(byte byte1, byte byte2, byte byte3, byte byte4);
    std::vector<byte> intToBytes(int intValue);
    double rawToDb(int rawValue);
    int dBToRaw(double dbValue);
    void resetRawDataRecording();
    bool getSensorData();

  protected:
    uint8_t mPresence = -1;
    float mMoveSpeed = NO_NUM;
    int8_t mSensitivity = -1;
    int8_t mScenario = -1;
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    bool checkSensorConnection() override;
    float measureValue(MeasureType iMeasureType) override;
    void sendDefaultSensorValues();

  public:
    SensorHLKLD2420(uint16_t iMeasureTypes);
    SensorHLKLD2420(uint16_t iMeasureTypes, uint8_t iAddress);
    virtual ~SensorHLKLD2420() {}

    //static bool decodePresenceResult(uint8_t iResult, bool &ePresence, uint8_t &eMove, uint8_t &eFall, uint8_t &eAlarm);
    bool begin() override;
    uint8_t getI2cSpeed() override;
    void defaultSensorParameters(int8_t iScenario, uint8_t iSensitivity);
    // void resetSensor();
    // void writeSensitivity(uint8_t iValue);
    // void readSensitivity();
    // void writeScenario(uint8_t iValue);
    // void readScenario();
    void sendCommand(byte command, std::vector<byte> paramter = {});
    std::string logPrefix() override;
};
#endif
#endif
