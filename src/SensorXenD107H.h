#pragma once
#ifdef PMMODULE
    #ifdef HF_SERIAL
        #include "Sensor.h"
        #include <string>

        #define HEADER_FOOTER_SIZE 4

        #define CMD_OPEN_COMMAND_MODE 0xFF
        #define CMD_CLOSE_COMMAND_MODE 0xFE
        #define CMD_READ_VERSION 0x00
        #define CMD_REBOOT_MODULE 0x68
        #define CMD_READ_GENERAL_CONFIG 0x71
        #define CMD_WRITE_GENERAL_CONFIG 0x70
        #define CMD_READ_MOVING_TARGET_CONFIG 0x73
        #define CMD_WRITE_MOVING_TARGET_CONFIG 0x72
        #define CMD_READ_MOTIONLESS_TARGET_CONFIG 0x75
        #define CMD_WRITE_MOTIONLESS_TARGET_CONFIG 0x74
        #define CMD_RAW_DATA_MODE 0x12

        #define OFFSET_PARAM_RANGE_GATE_MIN 0x00
        #define OFFSET_PARAM_RANGE_GATE_MAX 0x01
        #define OFFSET_PARAM_DELAY_TIME 0x04
        #define OFFSET_PARAM_TRIGGERS 0x10
        #define OFFSET_PARAM_HOLDS 0x20

        #define PARAM_OPEN_COMMAND_MODE_LENGTH 2
        #define PARAM_READ_GENERAL_CONFIG_LENGTH 10
        #define PARAM_READ_MOVING_TARGET_CONFIG_LENGTH 24
        #define PARAM_READ_MOTIONLESS_TARGET_CONFIG_LENGTH 20
        #define PARAM_RAW_DATA_MODE_LENGTH 6

        #define CALIBRATION_VALUE_COUNT 100
        #define CALIBRATION_TRIGGER_OFFSET_DB 10

        // these will be deducted from trigger offset based on sensitivity percentage
        // lower offsets = higher sensitivity
        #define SENSITIVITY_TRIGGER_RANGE 8
        #define SENSITIVITY_DEFAULT 5 // in case user setting invalid

        #define START_INIT 0
        #define START_SENSOR_ACTIVE 1
        #define START_VERSION_RECEIVED 2
        #define START_READ1_DONE 3
        #define START_READ2_DONE 4
        #define START_READ3_DONE 5
        #define START_CALIBRATING 10
        #define START_FINISHED 255

        #define BUFFER_LENGTH mBufferIndex

        #define XEND107H_FLASH_VERSION 0
        #define XEND107H_FLASH_MAGIC_WORD 2274541778
        #define XEND107H_FLASH_SIZE 69

class SensorXenD107H : public Sensor
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
        //! Presence detection data
        DATA = 0,
        //! Command packet
        COMMAND_RESPONSE,
        //! Raw data packet
        RAW_DATA
    };

    uint8_t mBufferTemp[1024];
    int mBufferTempIndex = 0;

    uint8_t mBuffer[1288]; // message buffer
    int mBufferIndex = 0;
    PacketStates mPacketState = GET_SYNC_STATE;
    PacketType mPacketType;
    float lastDetectedRange = NO_NUM;
    float lastDetectedSpeed = NO_NUM;

    int moduleVersionMajor = 0;
    int moduleVersionMinor = 0;
    int moduleVersionRevision = 0;

    int storedMovingTargetRangeMax = NO_NUM;
    int storedMotionlessTargetRangeMax = NO_NUM;
    bool storedAiPoweredDetection = false;
    bool storedDataTransfer = false;

    int storedDistanceMin = NO_NUM;
    int storedDistanceMax = NO_NUM;
    int storedDelayTime = NO_NUM;
    int storedTriggerThreshold[16];
    int storedHoldThreshold[16];
    float triggerThresholdDb[16];
    float holdThresholdDb[16];
    float triggerOffsetDb;
    float holdOffsetDb;
    float customeTriggerOffsetDb[16] = {};
    float customHoldOffsetDb[16] = {};
    bool calibrationCompleted = false;
    bool useFactoryDefaultThresholds = false;
    // initially suppress ON/OFF signals form HF-Sensor
    uint32_t calibrationOnOffTimer = 1;

    uint32_t rawDataLastRecordingReceived = 0;
    int rawDataRecordingCount = 0;
    float rawDataRangeAverageTempDb[16];
    float rawDataRangeAverageDb[16] = {};
    float rawDataRangeDifferencesDb[16] = {};
    bool calibrationTestRunOnly = false;

    int8_t mDefaultSensitivity = 5;
    uint8_t mHfSensorStartupState = 0;

    uint8_t HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
    uint8_t FOOTER[4] = {0xF8, 0xF7, 0xF6, 0xF5};
    uint8_t HEADER_COMMAND[4] = {0xFD, 0xFC, 0xFB, 0xFA};
    uint8_t FOOTER_COMMAND[4] = {0x04, 0x03, 0x02, 0x01};

    uint8_t HEADER_RAW_DATA[4] = {0xAA, 0xBF, 0x10, 0x14};
    uint8_t FOOTER_RAW_DATA[4] = {0xFD, 0xFC, 0xFB, 0xFA};

    uint8_t PARAM_OPEN_COMMAND_MODE[PARAM_OPEN_COMMAND_MODE_LENGTH] = {0x01, 0x00};
    uint8_t PARAM_READ_GENERAL_CONFIG[PARAM_READ_GENERAL_CONFIG_LENGTH] = {0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04, 0x00};
    uint8_t PARAM_READ_MOVING_TARGET_CONFIG[PARAM_READ_MOVING_TARGET_CONFIG_LENGTH] = {0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04, 0x00, 0x05, 0x00, 0x06, 0x00, 0x07, 0x00, 0x08, 0x00, 0x09, 0x00, 0x0A, 0x00, 0x0B, 0x00};
    uint8_t PARAM_READ_MOTIONLESS_TARGET_CONFIG[PARAM_READ_MOTIONLESS_TARGET_CONFIG_LENGTH] = {0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04, 0x00, 0x05, 0x00, 0x06, 0x00, 0x07, 0x00, 0x08, 0x00, 0x09, 0x00};
    uint8_t PARAM_RAW_DATA_MODE[PARAM_RAW_DATA_MODE_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    float maxDbValue = log10(pow(2, 31) - 1) * 10;

    void rebootSensorSoft();
    void rebootSensorHard();
    void startupLoop();
    void uartGetPacket();
    int bytesToInt(byte byte1, byte byte2, byte byte3, byte byte4);
    short bytesToShort(byte byte0, byte byte1);
    float rawToDb(int rawValue);
    int dBToRaw(float dbValue);
    void restartStartupLoop();
    void resetRawDataRecording();
    void sendCalibrationData();
    bool useCustomOffsets();
    bool getSensorData();

  protected:
    uint8_t mPresence = -1;
    float mMoveSpeed = NO_NUM;
    int8_t mSensitivity = -1;
    uint16_t mDelayTime = 30;
    uint8_t mRangeGateMin = 0;
    uint8_t mRangeGateMax = 15;
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    bool checkSensorConnection() override;
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorXenD107H(uint16_t iMeasureTypes, TwoWire* iWire);
    SensorXenD107H(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress);
    virtual ~SensorXenD107H() {}

    void sensorReadFlash(const uint8_t* iBuffer, const uint16_t iSize) override;
    void sensorWriteFlash() override;
    uint16_t sensorFlashSize() override;

    void forceCalibration();

    bool begin() override;
    uint8_t getI2cSpeed() override;
    void defaultSensorParameters(uint8_t iSensitivity, uint16_t iDelayTime, uint8_t iRangeGateMin, uint8_t iRangeGateMax);
    // void resetSensor();
    void writeSensitivity(int8_t iSensitivity);
    // void readSensitivity();
    void sendCommand(uint8_t command, const uint8_t parameter[] = nullptr, uint8_t parameterLength = 0);
    void showHelp();
    bool processCommand(const std::string iCmd, bool iDebugKo);
    std::string logPrefix() override;
    void switchPower(bool on);
};
    #endif
#endif
