#pragma once
#ifdef PMMODULE
    #ifdef HLK_SERIAL
        #include "Sensor.h"
        #include <string>

        #define HEADER_FOOTER_SIZE 4

        #define CMD_OPEN_COMMAND_MODE 0xFF
        #define CMD_CLOSE_COMMAND_MODE 0xFE
        #define CMD_READ_VERSION 0x00
        //#define CMD_REBOOT_MODULE 0x68
        #define CMD_READ_GENERAL_CONFIG 0x71
        #define CMD_WRITE_GENERAL_CONFIG 0x70
        #define CMD_READ_TRIGGER_CONFIG 0x73
        #define CMD_WRITE_TRIGGER_CONFIG 0x72
        #define CMD_READ_HOLD_CONFIG 0x77
        #define CMD_WRITE_HOLD_CONFIG 0x76
        #define CMD_DATA_MODE 0x7A

        #define OFFSET_PARAM_RANGE_GATE_MIN 0x0A
        #define OFFSET_PARAM_RANGE_GATE_MAX 0x05
        #define OFFSET_PARAM_DELAY_TIME 0x06
        #define OFFSET_PARAM_STATUS_REPORT_FREQUENCY 0x02
        #define OFFSET_PARAM_DISTANCE_REPORT_FREQUENCY 0x0C
        #define OFFSET_PARAM_RESPONSE_SPEED 0x0B

        #define PARAM_OPEN_COMMAND_MODE_LENGTH 2
        #define PARAM_READ_GENERAL_CONFIG_LENGTH 12
        #define PARAM_READ_TRIGGER_HOLD_LENGTH 32
        #define PARAM_DATA_MODE_LENGTH 6

        #define CALIBRATION_VALUE_COUNT 100
        #define CALIBRATION_TRIGGER_OFFSET_DB 10

        // these will be deducted from trigger offset based on sensitivity percentage
        // lower offsets = higher sensitivity
        #define SENSITIVITY_TRIGGER_RANGE 8
        #define SENSITIVITY_DEFAULT 5 // in case user setting invalid

        #define START_INIT 0
        #define START_SENSOR_ACTIVE 1
        #define START_VERSION_RECEIVED 2
        #define START_READ_GENEAL_DONE 3
        #define START_READ_TRIGGER_DONE 4
        #define START_READ_HOLD_DONE 5
        #define START_CALIBRATING 6
        #define START_FINISHED 255

        #define BUFFER_LENGTH mBufferIndex

        #define HLKLD2420_FLASH_VERSION 1
        #define HLKLD2420_FLASH_MAGIC_WORD 2274541778
        #define HLKLD2420_FLASH_SIZE 325

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
        //! Presence detection minimal data
        DATA_MINIMAL = 0,
        //! Presence detection standard data
        DATA_STANDARD,
        //! Command packet
        COMMAND_RESPONSE,
        //! Raw data packet
        RAW_DATA
    };

    uint8_t mBuffer[1288]; // message buffer
    int mBufferIndex = 0;
    PacketStates mPacketState = GET_SYNC_STATE;
    PacketType mPacketType;
    float lastDetectedRange = NO_NUM;

    short moduleVersionMajor = 0;
    short moduleVersionMinor = 0;
    short moduleVersionRevision = 0;
    int storedDistanceMin = NO_NUM;
    int storedDistanceMax = NO_NUM;
    int storedDelayTime = NO_NUM;
    int storedStatusReportFrequency = NO_NUM;
    int storedDistanceReportFrequency = NO_NUM;
    int storedResponseSpeed = NO_NUM;
    int storedTriggerThreshold[16] = {};
    int storedHoldThreshold[16] = {};
    float triggerOffsetDb[16];
    float holdOffsetDb[16];
    bool calibrationCompleted = false;
    bool useFactoryDefaultThresholds = false;
    // initially suppress ON/OFF signals form HF-Sensor
    uint32_t calibrationOnOffTimer = 1;

    uint32_t rawDataLastRecordingReceived = 0;
    byte rawDataRecordingCount = 0;
    byte rawDataRecordingCountGate[16] = {};
    float rawDataRangeTempSumDb[16];
    double rawDataRangeTempSquareSumDb[16];
    float rawDataRangeTempMaxDb[16] = {};
    float rawDataRangeAverageDb[16] = {};
    float rawDataRangeDeviationDb[16] = {};
    float rawDataRangeMaxDb[16] = {};
    float rawDataRangeTestAverageDb[16] = {};
    float rawDataRangeTestDifferencesDb[16] = {};
    float rawDataRangeTestDeviationDb[16] = {};
    float rawDataRangeTestMaxDb[16] = {};
    bool calibrationTestRunOnly = false;

    int8_t mDefaultSensitivity = 5;
    uint8_t mHfSensorStartupState = 0;

    uint8_t HEADER_DATA_MINIMAL[1] = {0x6E};
    uint8_t FOOTER_DATA_MINIMAL[1] = {0x62};
    uint8_t HEADER_DATA_STANDARD[4] = {0xF4, 0xF3, 0xF2, 0xF1};
    uint8_t FOOTER_DATA_STANDARD[4] = {0xF8, 0xF7, 0xF6, 0xF5};
    uint8_t HEADER_COMMAND[4] = {0xFD, 0xFC, 0xFB, 0xFA};
    uint8_t FOOTER_COMMAND[4] = {0x04, 0x03, 0x02, 0x01};

    uint8_t HEADER_RAW_DATA[4] = {0xAA, 0xBF, 0x10, 0x14};
    uint8_t FOOTER_RAW_DATA[4] = {0xFD, 0xFC, 0xFB, 0xFA};

    uint8_t PARAM_OPEN_COMMAND_MODE[PARAM_OPEN_COMMAND_MODE_LENGTH] = {0x01, 0x00};
    uint8_t PARAM_OPEN_COMMAND_MODE_EXT[PARAM_OPEN_COMMAND_MODE_LENGTH] = {0x02, 0x00};
    uint8_t PARAM_READ_GENERAL_CONFIG[PARAM_READ_GENERAL_CONFIG_LENGTH] = {0x05, 0x00, 0x0A, 0x00, 0x06, 0x00, 0x02, 0x00, 0x0C, 0x00, 0x0B, 0x00};
    uint8_t PARAM_READ_TRIGGER_HOLD[PARAM_READ_TRIGGER_HOLD_LENGTH] = {0x00, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04, 0x00, 0x05, 0x00, 0x06, 0x00, 0x07, 0x00, 0x08, 0x00, 0x09, 0x00, 0x0A, 0x00, 0x0B, 0x00, 0x0C, 0x00, 0x0D, 0x00, 0x0E, 0x00, 0x0F, 0x00};
    uint8_t PARAM_DATA_MODE_STANDARD[PARAM_DATA_MODE_LENGTH] = {0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
    uint8_t PARAM_DATA_MODE_MINIMAL[PARAM_DATA_MODE_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    float maxDbValue = log10(pow(2, 31) - 1) * 10;

    // void rebootSensorSoft();
    void rebootSensorHard();
    void startupLoop();
    void uartGetPacket();
    int bytesToInt(byte byte1, byte byte2, byte byte3, byte byte4);
    short bytesToShort(byte byte0, byte byte1);
    float rawToDb(int rawValue);
    void restartStartupLoop();
    void resetRawDataRecording();
    void sendCalibrationData();
    void sendCalibrationDataGeneral(uint8_t rangeGateMin, uint8_t rangeGateMax, uint16_t delayTime, uint8_t statusReportFrequency, uint8_t distanceReportFrequency, uint8_t responseSpeed);
    bool getSensorData();

  protected:
    uint8_t mPresence = -1;
    float mMoveSpeed = NO_NUM;
    int8_t mSensitivity = -1;
    uint16_t mDelayTime = 30;
    uint8_t mRangeGateMin = 0;
    uint8_t mRangeGateMax = 15;
    uint8_t mStatusReportFrequency = 40; // value / 10 = sec. (resolution: 0.5 sec.)
    uint8_t mDistanceReportFrequency = 5; // value / 10 = sec. (resolution: 0.5 sec.)
    uint8_t mResponseSpeed = 5; // 5 = normal, 10 = fast (no other values possible)
    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    bool checkSensorConnection() override;
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorHLKLD2420(uint16_t iMeasureTypes, TwoWire *iWire);
    SensorHLKLD2420(uint16_t iMeasureTypes, TwoWire *iWire, uint8_t iAddress);
    virtual ~SensorHLKLD2420() {}

    void sensorReadFlash(const uint8_t *iBuffer, const uint16_t iSize) override;
    void sensorWriteFlash() override;
    uint16_t sensorFlashSize() override;

    void forceCalibration();

    bool begin() override;
    uint8_t getI2cSpeed() override;
    void defaultSensorParameters(uint8_t iSensitivity, uint16_t iDelayTime, uint8_t iRangeGateMin, uint8_t iRangeGateMax, uint8_t iStatusReportFrequency = 40, uint8_t iDistanceReportFrequency = 5, uint8_t iResponseSpeed = 5);
    // void resetSensor();
    void writeSensitivity(int8_t iSensitivity);
    // void readSensitivity();
    void sendCommand(uint8_t command, const uint8_t parameter[] = nullptr, uint8_t parameterLength = 0);
    void showHelp();
    bool processCommand(const std::string iCmd, bool iDebugKo);
    std::string logPrefix() override;
    void switchPower(bool on);
    bool handleFunctionProperty(uint8_t *iData, uint8_t *eResultData, uint8_t &eResultLength);
    bool getCalibrationData(uint8_t *iData, uint8_t *eResultData, uint8_t &eResultLength);
    bool setCalibrationData(uint8_t *iData, uint8_t *eResultData, uint8_t &eResultLength);
    bool doCalibration(uint8_t *iData, uint8_t *eResultData, uint8_t &eResultLength);
};

      #ifdef HF_USE_SERIALPIO
        extern SerialPIO HLK_SERIAL;
      #endif
    #endif
#endif
