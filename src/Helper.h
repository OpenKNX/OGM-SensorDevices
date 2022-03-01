#pragma once

#include <cstdint>
#include <stdio.h>
#include <stdarg.h>
#include <Arduino.h>

#define NO_NUM -987654321.0F // normal NAN-Handling does not work

/*********************
 * Helper for any module
 * *******************/

// generic helper for formatted debug output
int printDebug(const char *format, ...);
void printHEX(const char* iPrefix, const uint8_t *iData, size_t iLength);
void printResult(bool iResult);

bool delayCheck(uint32_t iOldTimer, uint32_t iDuration);
bool isNum(float iNumber);