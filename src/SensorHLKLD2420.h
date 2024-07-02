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
#define CMD_READ_MODULE_CONFIG 0x08
#define CMD_WRITE_MODULE_CONFIG 0x07
#define CMD_RAW_DATA_MODE 0x12

#define OFFSET_PARAM_RANGE_GATE_MIN 0x00
#define OFFSET_PARAM_RANGE_GATE_MAX 0x01
#define OFFSET_PARAM_DELAY_TIME 0x04
#define OFFSET_PARAM_TRIGGERS 0x10
#define OFFSET_PARAM_HOLDS 0x20

#define PARAM_OPEN_COMMAND_MODE_LENGTH 2
#define PARAM_READ_DISTANCE_TRIGGER_LENGTH 36
#define PARAM_READ_DELAY_MAINTAIN_LENGTH 34
#define PARAM_RAW_DATA_MODE_LENGTH 6

#define CALIBRATION_VALUE_COUNT 100
#define CALIBRATION_TRIGGER_OFFSET_DB 5 // = min. sensitivity, range 1 - 5
#define CALIBRATION_HOLD_OFFSET_DB 2.5  // = min. sensitivity, range 1.5 - 2.5

// these will be deducted from trigger/hold offset based on sensitivity percentage
// e. g. sensitivity 5 (= 50 %): trigger offset 5-2=3, hold offset 2.5-1=1.5
// lower offsets = higher sensitivity
#define SENSITIVITY_TRIGGER_RANGE 4
#define SENSITIVITY_HOLD_RANGE 2
#define SENSITIVITY_DEFAULT 5 // in case user setting invalid

#define START_INIT 0
#define START_SENSOR_ACTIVE 1
#define START_VERSION_RECEIVED 2
#define START_READ1_DONE 3
#define START_READ2_DONE 4
#define START_CALIBRATING 5
#define START_FINISHED 255

#define BUFFER_LENGTH mBufferIndex

#define HLKLD2420_FLASH_VERSION 0
#define HLKLD2420_FLASH_MAGIC_WORD 1234 // #ToDo
#define HLKLD2420_FLASH_SIZE 133

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

    uint8_t mBuffer[1288]; // message buffer
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
    bool calibrationCompleted = false;

    uint32_t rawDataLastRecordingReceived = 0;
    double rawDataRangeAverage[16];
    int rawDataRecordingCount = 0;

    int8_t mDefaultSensitivity = 5;
    uint8_t mHfSensorStartupState = 0;

    uint8_t HEADER_ON[4] = {0x4F, 0x4E, 0x0D, 0x0A};
    uint8_t HEADER_OFF[4] = {0x4F, 0x46, 0x46, 0x0D};
    uint8_t HEADER_COMMAND[4] = {0xFD, 0xFC, 0xFB, 0xFA};
    uint8_t FOOTER[4] = {0x04, 0x03, 0x02, 0x01};

    uint8_t HEADER_RAW_DATA[4] = {0xAA, 0xBF, 0x10, 0x14};
    uint8_t FOOTER_RAW_DATA[4] = {0xFD, 0xFC, 0xFB, 0xFA};

    uint8_t PARAM_OPEN_COMMAND_MODE[PARAM_OPEN_COMMAND_MODE_LENGTH] = {0x01, 0x00};
    uint8_t PARAM_READ_DISTANCE_TRIGGER[PARAM_READ_DISTANCE_TRIGGER_LENGTH] = {0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x11, 0x00, 0x12, 0x00, 0x13, 0x00, 0x14, 0x00, 0x15, 0x00, 0x16, 0x00, 0x17, 0x00, 0x18, 0x00, 0x19, 0x00, 0x1A, 0x00, 0x1B, 0x00, 0x1C, 0x00, 0x1D, 0x00, 0x1E, 0x00, 0x1F, 0x00};
    uint8_t PARAM_READ_DELAY_MAINTAIN[PARAM_READ_DELAY_MAINTAIN_LENGTH] = {0x04, 0x00, 0x20, 0x00, 0x21, 0x00, 0x22, 0x00, 0x23, 0x00, 0x24, 0x00, 0x25, 0x00, 0x26, 0x00, 0x27, 0x00, 0x28, 0x00, 0x29, 0x00, 0x2A, 0x00, 0x2B, 0x00, 0x2C, 0x00, 0x2D, 0x00, 0x2E, 0x00, 0x2F, 0x00};
    uint8_t PARAM_RAW_DATA_MODE[PARAM_RAW_DATA_MODE_LENGTH] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    void startupLoop();
    void uartGetPacket();
    int bytesToInt(byte byte1, byte byte2, byte byte3, byte byte4);
    double rawToDb(int rawValue);
    int dBToRaw(double dbValue);
    void resetRawDataRecording();
    void saveCalibrationData();
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
    SensorHLKLD2420(uint16_t iMeasureTypes, TwoWire* iWire);
    SensorHLKLD2420(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress);
    virtual ~SensorHLKLD2420() {}

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
    std::string logPrefix() override;
};
    #endif
#endif
