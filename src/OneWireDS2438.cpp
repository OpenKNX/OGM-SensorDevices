#include <arduino.h>
#include <stdio.h>
#include <Wire.h>
#include "OneWire.h"
#ifdef COUNT_1WIRE_CHANNEL
#include "OneWireDS2438.h"
#include "OneWireDS2438Fromula.h"

OneWireDS2438::OneWireDS2438(tIdRef iId)
    : OneWire(iId){};

void OneWireDS2438::loop()
{
    switch (mState)
    {
        case Startup:
            init();
            mState = Idle;
            break;
        case StartMeasureTemp:
            if (delayCheck(pDelay, 1)) {
                mState = startConversionTemp() ? GetMeasureTemp : Error;
                pDelay = millis();
            }
            break;
        case GetMeasureTemp:
            if (delayCheck(pDelay, 20)) {
                mState = updateTemp() ? SetConfigVDD : Error;
                pDelay = millis();
            }
        case SetConfigVDD:
            if (delayCheck(pDelay, 1)) {
                mState = writeConfig(CONFIG_VDD) ? StartMeasureVDD : Error;
                pDelay = millis();
            }
        case StartMeasureVDD:
            if (delayCheck(pDelay, 20)) {
                mState = startConversionVolt() ? GetMeasureVDD : Error;
                pDelay = millis();
            }
        case GetMeasureVDD:
            if (delayCheck(pDelay, 10)) {
                mState = updateVDD() ? SetConfigVAD : Error;
                pDelay = millis();
            }
        case SetConfigVAD:
            if (delayCheck(pDelay, 1)) {
                mState = writeConfig(CONFIG_VAD) ? StartMeasureVAD : Error;
                pDelay = millis();
            }
        case StartMeasureVAD:
            if (delayCheck(pDelay, 20)) {
                mState = startConversionVolt() ? GetMeasureVAD : Error;
                pDelay = millis();
            }
        case GetMeasureVAD:
            if (delayCheck(pDelay, 10)) {
                mState = updateVAD() ? Idle : Error;
                pDelay = millis();
                if (mState == Idle) {
                    // here we ensure, that each measured value 
                    // consists of all values got a the same time
                    mTemp = mTmpTemp;
                    mVAD = mTmpVAD;
                    mVDD = mTmpVDD;
                    mVSens = mTmpVSens;
                }
            }
            break;
        case Idle:
            if (delayCheck(pDelay, 2000))
            {
                mState = StartMeasureTemp;
                pDelay = millis();
            }
            break;
        default:
            // error case, we stay here as long as there is a short on 1W-Line
            mState = pBM->readStatusShortDet() ? Error : Idle;
            break;
    }
}

void OneWireDS2438::init()
{
    mParasite = false;
}

bool OneWireDS2438::getValue(float& eValue, uint8_t iModelFunction)
{
    int8_t lFunctionIndex = iModelFunction - FmlUser_Start;
    // first we process predefined model functions
    switch (iModelFunction) {
        case FmlRaw_TemperatureOnChip:
            eValue = mTemp;
            break;
        case FmlRaw_VoltageOfChip:
            eValue = mVDD;
            break;
        case FmlRaw_VoltageOfADC1:
            eValue = mVAD;
            break;
        case FmlRaw_VoltageOfSens:
            eValue = mVSens;
            break;

        case FmlNative_TemperatureNTC:
            eValue = OneWireDS2438Fromula::nativeTemperatureNTC(mTemp, mVDD, mVAD, mVSens);
            break;
        case FmlNative_Humidity:
            eValue = OneWireDS2438Fromula::nativeHumidity(mTemp, mVDD, mVAD, mVSens);
            break;
        case FmlNative_Brightness:
            eValue = OneWireDS2438Fromula::nativeBrightness(mTemp, mVDD, mVAD, mVSens);
            break;

        default:
            if (lFunctionIndex >= 0 && lFunctionIndex <= FmlUser_End) {
                eValue = (*OneWireDS2438Fromula::userFunction)(mTemp, mVDD, mVAD, mVSens);
            } else {
                eValue = NO_NUM;
            }
            break;
    }
    return pValid && pMode == Connected && isNum(eValue);
}

bool OneWireDS2438::writeConfig(uint8_t config)
{
    //write config to scratchpad
    wireSelectThisDevice();
    pBM->wireWriteByte(WRITESCRATCH);
    pBM->wireWriteByte(0x00, 0); //write to first block
    pBM->wireWriteByte(config, 0);

    //confirm good write
    wireSelectThisDevice();
    pBM->wireWriteByte(READSCRATCH, 0);
    pBM->wireWriteByte(0x00, 0);

    uint8_t compare = pBM->wireReadByte();

    if (compare != config)
        return false;

    wireSelectThisDevice();
    pBM->wireWriteByte(COPYSCRATCH, 0);
    pBM->wireWriteByte(0x00, 0);
    // delay(20);

    return true;
}

uint8_t OneWireDS2438::readConfig()
{
    readScratchPad();
    return mScratchPad[0];
}

// float OneWireDS2438::readHum()
// {
//     // humidity can be calculated via two methods with the HIH-4010
//     //VOUT=(VSUPPLY)(0.0062(sensor RH) + 0.16), typical at 25 ºC
//     //((vout / vsupply) - 0.16)/0.0062 = RH @ 25 ºC
//     //or temp compensated:
//     //True RH = (Sensor RH)/(1.0546 – 0.00216T), T in ºC

//     float nowTemp = readTempC();

//     writeSetup(0x0F); // read source voltage for formula
//     float sourceVolt = readVolt();

//     writeSetup(0x00); // back to humidity voltage
//     float sensorVolt = readVolt();

//     float stdHum = ((sensorVolt / sourceVolt) - 0.16) / 0.0062;
//     float trueHum = stdHum / (1.0546 - (0.00216 * nowTemp));

//     return trueHum;
// }

bool OneWireDS2438::startConversionTemp()
{
    //request temp conversion
    wireSelectThisDevice();
    pBM->wireWriteByte(CONVERTT, mParasite);
    // delay(20);
    return true;
}

bool OneWireDS2438::updateTemp()
{

    //copy data from eeprom to scratchpad & read scratchpad
    readScratchPad();

    //return tempC (ignore 3 lsb as they are always 0);
    int16_t lTempRaw = (((int16_t)mScratchPad[TEMP_MSB]) << 5) | (mScratchPad[TEMP_LSB] >> 3);

    mTmpTemp = (float)lTempRaw * 0.03125;
    return true;
}

bool OneWireDS2438::startConversionVolt()
{
    //request volt conversion
    wireSelectThisDevice();
    pBM->wireWriteByte(CONVERTV, mParasite);
    // delay(10);
    return true;
}

bool OneWireDS2438::updateVDD()
{
    //copy data from eeprom to scratchpad & read scratchpad
    readScratchPad();

    int16_t lVoltRaw = (((int16_t)mScratchPad[VOLT_MSB]) << 8) | mScratchPad[VOLT_LSB];

    mTmpVDD = (float)lVoltRaw * 0.01;
    return true;
}

bool OneWireDS2438::updateVAD()
{
    bool lResult = false;

    readScratchPad();

    int16_t lVoltRaw = (((int16_t)mScratchPad[VOLT_MSB]) << 8) | mScratchPad[VOLT_LSB];
    mTmpVAD = (float)lVoltRaw * 0.01;

    lVoltRaw = (((int16_t)mScratchPad[CURR_MSB]) << 8) | mScratchPad[CURR_MSB];
    mTmpVSens = (double)lVoltRaw * 0.0002441;

    lResult = isNum(mTmpVAD);
    pValid = lResult;
    return lResult;
}

void OneWireDS2438::readScratchPad()
{
    wireSelectThisDevice();
    pBM->wireWriteByte(RECALLSCRATCH);
    pBM->wireWriteByte(0x00); // starting at address 0x00

    wireSelectThisDevice();
    pBM->wireWriteByte(READSCRATCH);
    pBM->wireWriteByte(0x00); // starting at address 0x00

    mScratchPad[STATUS] = pBM->wireReadByte();
    mScratchPad[TEMP_LSB] = pBM->wireReadByte();
    mScratchPad[TEMP_MSB] = pBM->wireReadByte();
    mScratchPad[VOLT_LSB] = pBM->wireReadByte();
    mScratchPad[VOLT_MSB] = pBM->wireReadByte();
    mScratchPad[CURR_LSB] = pBM->wireReadByte();
    mScratchPad[CURR_MSB] = pBM->wireReadByte();
    mScratchPad[THRESH] = pBM->wireReadByte();
}
#endif
