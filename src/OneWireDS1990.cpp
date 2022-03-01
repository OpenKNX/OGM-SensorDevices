#include <arduino.h>
#include <stdio.h>
#include <Wire.h>
#include "OneWire.h"
#ifdef COUNT_1WIRE_CHANNEL
#include "OneWireDS1990.h"

OneWireDS1990::OneWireDS1990(tIdRef iId)
    : OneWire(iId) {
    pPrio = PrioHigh;
};

void OneWireDS1990::loop()
{
}

bool OneWireDS1990::getValue(uint8_t &eValue, uint8_t iModelFunction) {
    eValue = (pMode == Connected);
    return true;
}
#endif
