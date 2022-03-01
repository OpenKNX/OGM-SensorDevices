#include <arduino.h>
#include <stdio.h>
#include <Wire.h>
#include "OneWire.h"
#ifdef COUNT_1WIRE_CHANNEL
#include "OneWireDS2408.h"

OneWireDS2408::OneWireDS2408(tIdRef iId) : OneWire(iId) {
    pPrio = PrioNormal;
};

void OneWireDS2408::init()
{
}

void OneWireDS2408::loop() {
    bool lSuccess = false;
    switch (mState)
    {
        case Startup:
            mValue = getState();
            mLastValue = mValue;
            mState = Idle;
            break;
        case SendOutput:
            // first find out, if we have to write
            if (mValue != mLastValue && mIoMask < 0xFF)
            {
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
            if (mIoMask)
            {
                mValue = getState();
                mLastValue = mValue;
            }
            mState = SendOutput;
            break;
        case Idle:
            // first find out, if we have to write
            if ((mIoMask < 0xFF) && (mValue != mLastValue))
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

uint8_t OneWireDS2408::getState() {
    wireSelectThisDevice();
    pBM->wireWriteByte(DS2408_CHANNEL_READ_CMD);
    return pBM->wireReadByte();
}

bool OneWireDS2408::setState(uint8_t iState) {
    wireSelectThisDevice();
    iState |= mIoMask;
    pBM->wireWriteByte(DS2408_CHANNEL_WRITE_CMD);
    pBM->wireWriteByte(iState);
    pBM->wireWriteByte(~iState);
    uint8_t lByte1 = pBM->wireReadByte();
    uint8_t lByte2 = pBM->wireReadByte();
    if (lByte1 == 0xAA && lByte2 == iState) {
        mValue = lByte2;
        mLastValue = mValue;
        return true;
    }
    return false;
}

bool OneWireDS2408::setValue(uint8_t iValue, uint8_t iModelFunction)
{
    // we set here the output according to given model funtion as byte or bit
    bool lResult = (Mode() == Connected);
    if (lResult) {
        if (iModelFunction == ModelFunction_IoByte || iModelFunction == ModelFunction_Default) {
            mValue = iValue ^ mIoInvertMask;
        } else if (iModelFunction >= ModelFunction_IoBit0 && iModelFunction <= ModelFunction_IoBit7) {
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

bool OneWireDS2408::getValue(uint8_t &eValue, uint8_t iModelFunction)
{
    // we get here the input according to given model funtion as byte or bit
    bool lResult = (Mode() == Connected);
    if (lResult) {
        if (iModelFunction == ModelFunction_IoByte || iModelFunction == ModelFunction_Default) {
            eValue = mValue ^ mIoInvertMask;
        } else if (iModelFunction >= ModelFunction_IoBit0 && iModelFunction <= ModelFunction_IoBit7) {
            uint8_t lBit = (1 << (iModelFunction - 1));
            eValue = ((mValue & lBit) ^ (mIoInvertMask & lBit)) ? 1 : 0;
        } else {
            lResult = false;
        }
    }
    return lResult;
}

bool OneWireDS2408::setParameter(OneWire::ModelParameter iModelParameter, uint8_t iValue, uint8_t iModelFunction)
{
    bool lResult = false;
    uint8_t lBit = (1 << (iModelFunction - 1));
    switch (iModelParameter)
    {
        case IoMask:
            // if an IOMask is set, we use it here, 1 is Input, 0 is Ouput;
            if (iModelFunction == ModelFunction_IoByte || iModelFunction == ModelFunction_Default)
            {
                mIoMask = iValue;
                lResult = true;
            }
            else if (iModelFunction >= ModelFunction_IoBit0 && iModelFunction <= ModelFunction_IoBit7)
            {
                mIoMask &= ~lBit;
                mIoMask |= (iValue & lBit);
                lResult = true;
            }
            break;
        case IoInvertMask:
            // if an IoInvertMask is set, we use it here, 1 is invert, 0 is normal;
            if (iModelFunction == ModelFunction_IoByte || iModelFunction == ModelFunction_Default)
            {
                mIoInvertMask = iValue;
                lResult = true;
            }
            else if (iModelFunction >= ModelFunction_IoBit0 && iModelFunction <= ModelFunction_IoBit7)
            {
                mIoInvertMask &= ~lBit;
                mIoInvertMask |= (iValue & lBit);
                lResult = true;
            }
            break;
        default:
            break;
    }
    return lResult;
}

uint8_t OneWireDS2408::getRegister(uint16_t iRegister)
{
    wireSelectThisDevice();
    pBM->wireWriteByte(DS2408_PIO_READ_CMD);
    pBM->wireWriteByte(REG_LO(iRegister));
    pBM->wireWriteByte(REG_HI(iRegister));
    return pBM->wireReadByte();
}

void OneWireDS2408::setRegister(uint16_t iRegister, uint8_t iValue)
{
    wireSelectThisDevice();
    pBM->wireWriteByte(DS2408_SEARCH_CMD);
    pBM->wireWriteByte(REG_LO(iRegister));
    pBM->wireWriteByte(REG_HI(iRegister));
    pBM->wireWriteByte(iValue);
}

void OneWireDS2408::setSearchMask(uint8_t iMask)
{
    wireSelectThisDevice();
    setRegister(DS2408_SEARCH_MASK_REG, iMask);
}

void OneWireDS2408::setSearchPolarity(uint8_t iPolarity)
{
    wireSelectThisDevice();
    setRegister(DS2408_SEARCH_SELECT_REG, iPolarity);
}

void OneWireDS2408::setMode(uint8_t iMode)
{
    wireSelectThisDevice();
    setRegister(DS2408_CONTROL_STATUS_REG, iMode);
}

uint8_t OneWireDS2408::getMode()
{
    wireSelectThisDevice();
    return getRegister(DS2408_CONTROL_STATUS_REG);
}

uint8_t OneWireDS2408::getCurrentState()
{
    wireSelectThisDevice();
    return getRegister(DS2408_PIO_LOGIC_REG);
}

uint8_t OneWireDS2408::getLastState()
{
    wireSelectThisDevice();
    return getRegister(DS2408_PIO_OUTPUT_REG);
}

uint8_t OneWireDS2408::getActivity()
{
    wireSelectThisDevice();
    return getRegister(DS2408_PIO_ACTIVITY_REG);
}

bool OneWireDS2408::resetActivity()
{
    wireSelectThisDevice();
    pBM->wireWriteByte(DS2408_RESET_CMD);
    return (pBM->wireReadByte() == 0xAA);
}
#endif
