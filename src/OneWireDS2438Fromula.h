#pragma once

#include "OneWireDS2438.h"
#ifdef COUNT_1WIRE_CHANNEL

#define FmlRaw_TemperatureOnChip 10
#define FmlRaw_VoltageOfChip 11
#define FmlRaw_VoltageOfADC1 12
#define FmlRaw_VoltageOfSens 13

#define FmlNative_TemperatureNTC 15
#define FmlNative_Humidity 16
#define FmlNative_Brightness 17
#define FmlNative_Future 18

#define FmlUser_Start 50
#define FmlUser_End 79

class OneWireDS2438Fromula
{
  friend OneWireDS2438;

  private:
    OneWireDS2438Fromula(/* args */);
    ~OneWireDS2438Fromula();

    static float (*userFunction[30])(float iTemp, float iVDD, float iVAD, float iVSens);

    // native functions for value conversions (already implemented)
    static float nativeTemperatureNTC(float iTemp, float iVDD, float iVAD, float iVSens);
    static float nativeHumidity(float iTemp, float iVDD, float iVAD, float iVSens);
    static float nativeBrightness(float iTemp, float iVDD, float iVAD, float iVSens);

    // user functions for value conversions (empty, implemented by user)
    static float userFunction1(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction2(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction3(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction4(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction5(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction6(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction7(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction8(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction9(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction10(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction11(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction12(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction13(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction14(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction15(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction16(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction17(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction18(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction19(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction20(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction21(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction22(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction23(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction24(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction25(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction26(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction27(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction28(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction29(float iTemp, float iVDD, float iVAD, float iVSens);
    static float userFunction30(float iTemp, float iVDD, float iVAD, float iVSens);
};
#endif
