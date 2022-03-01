#include "OneWire.h"
#ifdef COUNT_1WIRE_CHANNEL
#include "OneWireDS2482.h"
#include "OneWireDS18B20.h"
#include "OneWireDS1990.h"
#include "OneWireDS2408.h"
#include "OneWireDS2413.h"
#include "OneWireDS2438.h"

uint8_t OneWire::sSensorCount = 0;
OneWire* OneWire::sSensor[COUNT_1WIRE_CHANNEL] = {0};

bool equalId(const tIdRef iId1, const tIdRef iId2)
{
    for (uint8_t i = 1; i < 7; i++)
        if (iId1[i] != iId2[i])
            return false;
    return (iId1[0] == iId2[0]);
}

bool equalId(const tIdRef iId1, const int32_t* iId2)
{
    for (uint8_t i = 1; i < 7; i++)
        if (iId1[i] != iId2[i])
            return false;
    return (iId1[0] == iId2[0]);
}

void copyId(tIdRef iIdLeft, const tIdRef iIdRight)
{
    for (uint8_t i = 0; i < 7; i++)
        iIdLeft[i] = iIdRight[i];
}

// static Factory method, creates OneWireSensors just in case, they do not exist
// it returns also the information if a new sensor was created
OneWire* OneWire::factory(tIdRef iId, bool *eIsNew)
{
    OneWire* lSensor = NULL;

    // check if Sensor exists
    for (uint8_t i = 0; i < sSensorCount; i++)
    {
        if (equalId(sSensor[i]->Id(), iId))
        {
            lSensor = sSensor[i];
            *eIsNew = false;
            break;
        }
    }
    if (lSensor == NULL && sSensorCount < COUNT_1WIRE_CHANNEL)
    {
        // this is a new sensor
        switch (iId[0])
        {
            case MODEL_DS18B20:
            case MODEL_DS18S20:
                lSensor = (new OneWireDS18B20(iId));
                break;
            case MODEL_DS1990:
                lSensor = (new OneWireDS1990(iId));
                break;
            case MODEL_DS2408:
                lSensor = (new OneWireDS2408(iId));
                break;
            case MODEL_DS2413:
                lSensor = (new OneWireDS2413(iId));
                break;
            case MODEL_DS2438:
                lSensor = (new OneWireDS2438(iId));
                break;
            default:
                printHEX("Unsupported family found: ", iId, 7);
                // sensor family not supported
                break;
        }
        if (lSensor != NULL)
        {
            *eIsNew = true;
            sSensor[sSensorCount++] = lSensor;
        }
    }
    return lSensor;
}

OneWire::OneWire(tIdRef iId)
{
    copyId(pId, iId);
};

OneWire::~OneWire(){};

OneWire::SensorMode OneWire::Mode() {
    return pMode;
}

void OneWire::setModeConnected(bool iForce /* = false */)
{
    if (iForce || pSearchCount >= cDisappearCount) {
        pMode = Connected;
        pSearchCount = 0;
    }
}

void OneWire::setModeDisconnected(bool iForce /* = false */)
{
    if (iForce || pSearchCount >= cDisappearCount)
    {
        pMode = Disconnected;
        pSearchCount = 0;
    }
}

void OneWire::setBusmaster(OneWireDS2482* iBM)
{
    pBM = iBM;
}

void OneWire::clearSearchCount() {
    pSearchCount = 0;
}

void OneWire::incrementSearchCount(bool iForce /* = false */)
{
    if (iForce || pMode != New) pSearchCount++;
}

void OneWire::wireSelectThisDevice() {
    if (pBM) {
        pBM->wireReset();
        pBM->wireSelect(pId);
    }
}

bool OneWire::getValue(float& eValue, uint8_t iModelFunction)
{
    return false;
}

bool OneWire::getValue(uint8_t& eValue, uint8_t iModelFunction)
{
    return false;
}

bool OneWire::setValue(uint8_t iValue, uint8_t iModelFunction)
{
    // do nothing, should be overridden for devices with output capabilities
    return false;
}

bool OneWire::setParameter(OneWire::ModelParameter iModelParameter, uint8_t iValue, uint8_t iModelFunction) {
    // default implementation for devices without parameters
    return false;
}
#endif