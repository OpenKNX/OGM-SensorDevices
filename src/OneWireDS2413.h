#pragma once

#include <inttypes.h>
#include "OneWire.h"
#ifdef COUNT_1WIRE_CHANNEL
#include "OneWireDS2482.h"

// Model Commands
#define DS2413_CHANNEL_READ_CMD  0xF5
#define DS2413_CHANNEL_WRITE_CMD 0x5A

// Control/Status Bits
#define PIOA_STATE_INPUT_BIT  0x1
#define PIOA_STATE_OUTPUT_BIT 0x2
#define PIOB_STATE_INPUT_BIT  0x4
#define PIOB_STATE_OUTPUT_BIT 0x8

#define PIOA_WRITE_BIT        0x01
#define PIOB_WRITE_BIT        0x02
#define PIO_WRITE_MASK        0xFC

class OneWireDS2413 : public OneWire
{
  public:
    enum StateSensorIO
    {
        Startup,
        GetInput,
        GetStatus,
        SendOutput,
        Idle,
        Error
    };

    OneWireDS2413(tIdRef iId);

    void init();
    void loop() override;

    uint8_t convertStateToValue(uint8_t iValue);
    uint8_t getState();
    bool setState(uint8_t iState);

    bool getValue(uint8_t &eValue, uint8_t iModelFunction) override;
    bool setValue(uint8_t iValue, uint8_t iModelFunction) override;
    bool setParameter(ModelParameter iModelParameter, uint8_t iValue, uint8_t iModelFunction) override;

  private:
    uint8_t mState = Startup;
    uint8_t mValue = 0;
    uint8_t mLastValue = 0;
    uint8_t mIoMask = 0; // no input, just ouput
    uint8_t mIoInvertMask = 0; // no invert
};
#endif
