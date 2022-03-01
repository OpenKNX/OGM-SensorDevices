#include <arduino.h>
#include <stdio.h>
#include <Wire.h>
#include "OneWire.h"
#ifdef COUNT_1WIRE_CHANNEL
#include "OneWireDS2413.h"

OneWireDS2413::OneWireDS2413(tIdRef iId) : OneWire(iId){
    pPrio = PrioNormal;
};

void OneWireDS2413::init()
{
}

void OneWireDS2413::loop() {
    bool lSuccess = false;
    switch (mState) {
        case Startup:
            mValue = getState();
            mLastValue = mValue;
            mState = Idle;
            break;
        case SendOutput:
            // first find out, if we have to write
            if (mValue != mLastValue && mIoMask < 3) {
                // mValue was changed by external call, we have to write this to the 1W bus
                lSuccess = setState(mValue);
                // status is automatically fetched after a write command, so mValue is now correct
            }
            mState = lSuccess ? SendOutput : GetInput;
            break;
        case GetStatus:
            // after an Ouput the current state shoud be read
            mValue = getState();
            mLastValue = mValue;
            mState = SendOutput;
        case GetInput:
            // normal read happens just if the IO is configured for Input 
            if (mIoMask) {
                mValue = getState();
                mLastValue = mValue;
            }
            mState = SendOutput;
            break;
        case Idle:
            // first find out, if we have to write
            if ((mIoMask < 3) && (mValue != mLastValue))
            {
                // mValue was changed by external call, we have to write this to the 1W bus
                setState(mValue);
                // afterwards mValue and mLastvalue are equal and mValue contains the state info
                // so we need not to read anymore from 1W-Bus
            }
            else if (mIoMask)
            {
                // we do not have to write, so we will read inputs from 1W bus if there are input pins
                mValue = getState();
                mLastValue = mValue;
            }
            break;
        default:
            break;
  }
}

uint8_t OneWireDS2413::convertStateToValue(uint8_t iValue){
    uint8_t lResult = ((iValue & PIOB_STATE_INPUT_BIT) >> 1) | (iValue & PIOA_STATE_INPUT_BIT);
    return lResult;
}

uint8_t OneWireDS2413::getState()
{
    wireSelectThisDevice();
    pBM->wireWriteByte(DS2413_CHANNEL_READ_CMD);
    return convertStateToValue(pBM->wireReadByte());
}

bool OneWireDS2413::setState(uint8_t iState) {
    wireSelectThisDevice();
    iState |= PIO_WRITE_MASK | mIoMask;
    pBM->wireWriteByte(DS2413_CHANNEL_WRITE_CMD);
    pBM->wireWriteByte(iState);
    pBM->wireWriteByte(~iState);
    uint8_t lByte1 = pBM->wireReadByte();
    // here the check for state gets complicated
    // the last fetched byte is not the sent byte, 
    // but the state byte (as in getState)
    uint8_t lByte2 = convertStateToValue(pBM->wireReadByte());
    if (lByte1 == 0xAA && lByte2 == (iState & ~PIO_WRITE_MASK))
    {
        mValue = lByte2;
        mLastValue = mValue;
        return true;
    }
    return false;
}

bool OneWireDS2413::setValue(uint8_t iValue, uint8_t iModelFunction)
{
    // we set here the output according to given model funtion as byte or bit
    bool lResult = (Mode() == Connected);
    if (lResult) {
        if (iModelFunction == ModelFunction_IoByte || iModelFunction == ModelFunction_Default) {
            mValue = (iValue & ~PIO_WRITE_MASK) ^ mIoInvertMask;
        } else if (iModelFunction >= ModelFunction_IoBit0 && iModelFunction <= ModelFunction_IoBit1) {
            uint8_t lBit = (1 << (iModelFunction - 1));
            if (iValue != ((mIoInvertMask & lBit) ? 1 : 0)) {
                mValue |= lBit;
            } else {
                mValue &= ~lBit;
            }
        } else {
            lResult = false;
        }
    }
    // ensure, that state machine is set to write this value next
    // if (lResult)
    //     mState = SendOutput;
    return lResult;
}

bool OneWireDS2413::getValue(uint8_t &eValue, uint8_t iModelFunction)
{
    // we get here the input according to given model funtion as byte or bit
    bool lResult = (Mode() == Connected);
    if (lResult)
    {
        if (iModelFunction == ModelFunction_IoByte || iModelFunction == ModelFunction_Default) {
            eValue = mValue ^ mIoInvertMask;
        } else if (iModelFunction >= ModelFunction_IoBit0 && iModelFunction <= ModelFunction_IoBit1) {
            uint8_t lBit = (1 << (iModelFunction - 1));
            eValue = ((mValue & lBit) ^ (mIoInvertMask & lBit)) ? 1 : 0;
        } else {
            lResult = false;
        }
    }
    return lResult;
}

bool OneWireDS2413::setParameter(OneWire::ModelParameter iModelParameter, uint8_t iValue, uint8_t iModelFunction)
{
    bool lResult = false;
    uint8_t lValue = iValue & ~PIO_WRITE_MASK;
    uint8_t lBit = (1 << (iModelFunction - 1));
    switch (iModelParameter)
    {
        case IoMask:
            // if an IOMask is set, we use it here, 1 is Input, 0 is Ouput;
            if (iModelFunction == ModelFunction_IoByte || iModelFunction == ModelFunction_Default)
            {
                mIoMask = lValue;
                lResult = true;
            }
            else if (iModelFunction >= ModelFunction_IoBit0 && iModelFunction <= ModelFunction_IoBit1)
            {
                mIoMask &= ~lBit;
                mIoMask |= (lValue & lBit);
                lResult = true;
            }
            break;
        case IoInvertMask:
            // if an IoInvertMask is set, we use it here, 1 is invert, 0 is normal;
            if (iModelFunction == ModelFunction_IoByte || iModelFunction == ModelFunction_Default)
            {
                mIoInvertMask = lValue;
                lResult = true;
            }
            else if (iModelFunction >= ModelFunction_IoBit0 && iModelFunction <= ModelFunction_IoBit1)
            {
                mIoInvertMask &= ~lBit;
                mIoInvertMask |= (lValue & lBit);
                lResult = true;
            }
            break;
        default:
            break;
    }
    return lResult;
}
#endif