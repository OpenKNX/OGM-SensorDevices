#include "Helper.h"

// generic helper for formatted debug output
int printDebug(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    int lResult = vsnprintf(buffer, 256, format, args);
    va_end(args);
    SERIAL_DEBUG.print(buffer);
    return lResult;
}

void printHEX(const char* iPrefix, const uint8_t *iData, size_t iLength)
{
    SERIAL_DEBUG.print(iPrefix);
    for (size_t i = 0; i < iLength; i++) {
        if (iData[i] < 0x10) { SERIAL_DEBUG.print("0"); }
        SERIAL_DEBUG.print(iData[i], HEX);
        SERIAL_DEBUG.print(" ");
    }
    SERIAL_DEBUG.println();
}

void printResult(bool iResult)
{
    SERIAL_DEBUG.println(iResult ? "OK" : "FAIL");
}

// ensure correct time delta check
// cannot be used in interrupt handler
bool delayCheck(uint32_t iOldTimer, uint32_t iDuration)
{
    return millis() - iOldTimer >= iDuration;
}

uint32_t delayTimerInit()
{
    uint32_t lResult = millis();
    if (lResult == 0)
        lResult = 1;
    return lResult;
}

// check if a float is a number (false if Not-a-number)
bool isNum(float iNumber) {
    return (iNumber + 10.0) > NO_NUM;
}