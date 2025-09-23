// #include "IncludeManager.h"
#ifdef SENSORMODULE
    #include "SensorSCD40.h"
    #include "knx.h"
    #include <Wire.h>

// init communication buffer
uint8_t SensorSCD40::communication_buffer[9] = {0};
int16_t SensorSCD40::sError = 0;
char SensorSCD40::sErrorMessage[64] = {0};

SensorSCD40::SensorSCD40(uint16_t iMeasureTypes, TwoWire* iWire)
    : Sensor(iMeasureTypes, iWire, SCD40_I2C_ADDR), SensirionI2cScd4x(){};

SensorSCD40::SensorSCD40(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress)
    : Sensor(iMeasureTypes, iWire, iAddress), SensirionI2cScd4x(){};

uint8_t SensorSCD40::getSensorClass()
{
    return SENS_SCD40;
}

std::string SensorSCD40::logPrefix()
{
    return "Sensor<SCD40>";
}

void SensorSCD40::sensorLoopInternal()
{
    switch (pSensorState)
    {
        case Wakeup:
            if (delayCheck(pSensorStateDelay, 100))
            {
                Sensor::sensorLoopInternal();
                mRetryCounter = 0;
            }
            break;
        case Calibrate:
            if (delayCheck(pSensorStateDelay, 1000)) //SCD needs this to startup
            {
                uint8_t lError = calibrate();
                if (lError == 0)
                    pSensorState = Finalize;
                else if (lError < 100 && mRetryCounter++ == 0)
                    logDebugP("SCD4x still measuring, skipping initial startup.");
                else if (lError < 200 && mRetryCounter++ < 40) // try 40 seconds 
                    logDebugP("SCD4x startup failed, retrying...");
                else {
                    logDebugP("SCD4x startup failed, giving up.");
                    pSensorState = Off; // give up
                }
                pSensorStateDelay = delayTimerInit();
            }
            break;
        case Finalize:
            // we ask for value until we get a valid value
            if (delayCheck(pSensorStateDelay, 1000))
            {
                if (getSensorData())
                    pSensorState = Running;
                pSensorStateDelay = delayTimerInit();
            }
            break;
        case Running:
            if (delayCheck(pSensorStateDelay, 30000))
            {
                getSensorData();
                pSensorStateDelay = delayTimerInit();
            }
            break;
        default:
            pSensorStateDelay = delayTimerInit();
            break;
    }
}

float SensorSCD40::measureValue(MeasureType iMeasureType)
{
    switch (iMeasureType)
    {
        case Temperature:
            return (float)mTemp;
            break;
        case Humidity:
            return (float)mHum;
            break;
        case Co2:
            return (float)mCo2;
            break;
        default:
            break;
    }
    return NO_NUM;
}

bool SensorSCD40::begin()
{
    logDebugP("Starting sensor SCD40... ");
    return beginInternal();
}

bool SensorSCD40::beginInternal() {
    SensirionI2cScd4x::begin(*pWire, SCD40_I2C_ADDR);
    bool lResult = Sensor::begin();
    logResult(lResult);
    return lResult;
}

// override stopPeriodicMeasurement to avoid blocking forever
int16_t SensorSCD40::stopPeriodicMeasurement(bool blocking) {
    int16_t localError = 0; //NO_ERROR;
    uint8_t* buffer_ptr = communication_buffer;
    SensirionI2CTxFrame txFrame =
        SensirionI2CTxFrame::createWithUInt16Command(0x3f86, buffer_ptr, 2);
    localError =
        SensirionI2CCommunication::sendFrame(SCD40_I2C_ADDR, txFrame, *Sensor::pWire);
    if (localError != 0) {
        return localError;
    }
    if (blocking) delay(500);
    return localError;
}

void SensorSCD40::logSensorError(int16_t iError, const char* iErrorMessage)
{
    logDebugP(iErrorMessage);
    errorToString(iError, sErrorMessage, sizeof sErrorMessage);
    logIndentUp();
    logDebugP(sErrorMessage);
    logIndentDown();
}


uint8_t SensorSCD40::calibrateExtended()
{
    sError = startLowPowerPeriodicMeasurement();
    if (sError) {
        logSensorError(sError, "Error trying to execute startLowPowerPeriodicMeasurement(): ");
        return 130;
    }
    logDebugP("Low power mode enabled!");
    return 0;    
}


uint8_t SensorSCD40::calibrate()
{
    sError = wakeUp();
    if (sError) {
        logSensorError(sError, "Error trying to execute wakeUp(): ");
        return 1;
    }
    sError = stopPeriodicMeasurement(false);
    if (sError) {
        logSensorError(sError, "Error trying to execute stopPeriodicMeasurement(): ");
        return 2;
    }
    sError = reinit();
    if (sError) {
        logSensorError(sError, "Error trying to execute reinit(): ");
        return 3;
    }
    uint64_t lSerialNumber;
    sError = getSerialNumber(lSerialNumber);
    if (sError) {
        logSensorError(sError, "Error trying to execute getSerialNumber(): ");
        return 100;
    }
    uint16_t lSensorVariant;
    sError = getSensorVariantRaw(lSensorVariant);
    if (sError) {
        logSensorError(sError, "Error trying to determine Sensor with getSensorVariantRaw(): ");
        return 110;
    }
    mSensorVariant = (lSensorVariant & 0x1000) >> 12;
    logDebugP("SCD4%u connected, serial number: ", mSensorVariant);
    logHexDebugP((uint8_t*)&lSerialNumber, 8);

    if (pTempOffset != 0) {
        sError = setTemperatureOffset(-pTempOffset);
        if (sError) {
            logSensorError(sError, "Error trying to call setTemperatureOffset(): ");
            return 120;
        } else
            logDebugP("TempOffset %.2f successfully set", pTempOffset);
    }
        
    return calibrateExtended();
}

uint8_t SensorSCD40::getI2cSpeed()
{
    return 1; // n * 100kHz
}

bool SensorSCD40::getSensorData()
{
    bool lDataReady;
    bool lResult = (SensirionI2cScd4x::getDataReadyStatus(lDataReady) == 0);

    if (lResult)
    {
        if (lDataReady)
        {
            uint16_t lTemp;
            uint16_t lHum;
            uint16_t lCo2;
            lResult = (SensirionI2cScd4x::readMeasurementRaw(lCo2, lTemp, lHum) == 0);
            lResult = (lCo2 > 0);
            if (lResult)
            {
                mTemp = lTemp * 175.0 / 65535.0 - 45.0;
                mHum = lHum * 100.0 / 65535.0;
                mCo2 = lCo2;
                processPressure();
            }
        }
    }
    return lResult;
}

bool SensorSCD40::prepareTemperatureOffset(float iTempOffset)
{
    pTempOffset = iTempOffset;
    return true;
}

// quick hack: make pressure available by KO
void SensorSCD40::processPressure()
{
    #ifdef KoSENS_Pre
    // pressure is in mBar
    uint32_t lPressure = KoSENS_Pre.value(DPT_Value_Pres);
    // hack for testing: We take pressure from according KO and try to take this value as pressure compensation
    if (KoSENS_Pre.initialized() && lPressure > 70000 && lPressure < 120000 && (lPressure > mPressure + 10 || lPressure < mPressure - 10))
    {
        setPressure(lPressure);
        mPressure = lPressure;
        logDebugP("Pressure set to %i", lPressure);
    }
    #endif
}

// pressure is in mBar
bool SensorSCD40::setPressure(uint32_t pressure)
{
    return setAmbientPressure(pressure);
}

void SensorSCD40::sensorShowHelp()
{
    openknx.console.printHelpLine("scd asc", "Get current auto self calibration state.");
    openknx.console.printHelpLine("scd asc NNNN", "Set auto calibration target to NNNN ppm.");
    openknx.console.printHelpLine("scd factory", "Reset sensor with factory settings.");
    openknx.console.printHelpLine("scd force NNNN", "Set current CO2 value to NNNN ppm.");
    openknx.console.printHelpLine("scd persist", "persist volatile settings to EEPROM. Be careful, EEPROM has just 2000 write cycles.");
    openknx.console.printHelpLine("scd start", "Leave stop state and continue measureing.");
    openknx.console.printHelpLine("scd stop", "Stop measuring to allow management commands.");
    openknx.console.printHelpLine("scd ver", "Get connected SCD4x sensor variant.");
}

bool SensorSCD40::isInt(const std::string& s) {
    for (char c : s) {
        if (!std::isdigit(c) && !std::isspace(c)) return false;
    }
    return true;
}

int16_t SensorSCD40::getIntArg(const std::string& s) {
    if (s.length() <= 4) return -1; 
    std::string arg=s.substr(s.length()-4); // Extract nuber argument at the end
    if (!isInt(arg)) return -1; 
    return std::stoi(arg);
}


bool SensorSCD40::sensorProcessCommand(const std::string iCmd, bool iDebugKo)
{
    bool lResult = false;
    if (iCmd.find("scd ") != 0)
        return lResult;
    std::string lStateString = getSensorStateAsString();
    if (iCmd.find("scd h") == 0)
    {
        // Command help
        if (iDebugKo)
        {
            openknx.console.writeDiagnoseKo("-> asc");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> asc NNNN");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> factory");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> force NNNN");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> persist");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> start");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> stop");
            openknx.console.writeDiagnoseKo("");
            openknx.console.writeDiagnoseKo("-> ver");
        }
        else
            sensorShowHelp();
        lResult = true;
    }
    else if (iCmd.find("scd v") == 0)
    {
        logInfoP("SCD4%u connected", mSensorVariant);
        if (iDebugKo)
            openknx.console.writeDiagnoseKo("SCD4%u", mSensorVariant);
        lResult = true;
    } 
    else if (iCmd.find("scd sto") == 0)
    {
        if (pSensorState == Running && !pSensorStateStopped) {
            sError = stopPeriodicMeasurement(true);
            if (sError) {
                logWarningP("Sensor could not be stopped, please retry");
                if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u retry", mSensorVariant);
            }
            else
            {
                logInfoP("SCD4%u stopped", mSensorVariant);
                if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u stopped", mSensorVariant);
            }
            pSensorStateStopped = true;
            pSensorState = Off;
        }
        else if (pSensorState == Off && pSensorStateStopped)
        {
            logInfoP("Sensor is already stopped");
            if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u stopped", mSensorVariant);
        } 
        else
        {
            logInfoP("Sensor is in state %s, no stop possible", lStateString.c_str());
            if (iDebugKo) openknx.console.writeDiagnoseKo("IS %s", lStateString.substr(0,11).c_str());
        }
        lResult = true;
    } 
    else if (iCmd.find("scd star") == 0)
    {
        if (pSensorState == Off && pSensorStateStopped) {
            pSensorStateStopped = false;
            pSensorState = Wakeup;
            logInfoP("SCD4%u starting sensor", mSensorVariant);
            if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u staring", mSensorVariant);
        }
        else
        {
            logInfoP("Sensor state is %s", lStateString.c_str());
            if (iDebugKo) openknx.console.writeDiagnoseKo("IS %s", lStateString.substr(0,11).c_str());
        }
        lResult = true;
    } 
    // following commands only possible if sensor is stopped
    else if (pSensorState == Off && pSensorStateStopped) 
    {
        if (iCmd.find("scd persist") == 0)
        {
            sError = persistSettings();
            if (sError) {
                logErrorP("Persisting settings failed");
                if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u failed", mSensorVariant);
            }
            else
            {
                logInfoP("SCD4%u persisting settings was successful", mSensorVariant);
                if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u OK", mSensorVariant);
            }
            lResult = true;
        }
        if (iCmd.find("scd factory") == 0)
        {
            sError = performFactoryReset();
            if (sError) {
                logErrorP("Factory reset failed");
                if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u failed", mSensorVariant);
            }
            else
            {
                logInfoP("SCD4%u factroy reset was successful", mSensorVariant);
                if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u OK", mSensorVariant);
            }
            lResult = true;
        }
        else if (iCmd.find("scd a") == 0)
        {
            int16_t lTarget = getIntArg(iCmd);
            if (lTarget < 0) 
            {
                // no valid number given, we just want to know the current state
                uint16_t ascEnabled = 9999; 
                uint16_t ascTarget = 9999; 
                sError = getAutomaticSelfCalibrationEnabled(ascEnabled);
                if (sError) 
                {
                    logErrorP("Failed to get ASC state");
                    if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u failed", mSensorVariant);
                }
                else
                {
                    sError = getAutomaticSelfCalibrationTarget(ascTarget);
                    if (sError) 
                    {
                        logErrorP("Failed to get ASC target");
                        if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u failed", mSensorVariant);
                    }
                    else
                    {
                        logInfoP("SCD4%u automatic self calibration %s, target is %u ppm", mSensorVariant, (ascEnabled == 1) ? "enabled" : "disabled", ascTarget);
                        if (iDebugKo) openknx.console.writeDiagnoseKo("ASC %s %4u ppm", (ascEnabled == 1) ? "X" : "-", ascTarget);
                    }
                }
            }
            else if (lTarget < 400 || lTarget > 5000) 
            {
                logInfoP("Invalid target value, must be between 400 and 5000 ppm");
                if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u invalid", mSensorVariant);
            }
            else
            {
                sError = setAutomaticSelfCalibrationTarget(lTarget);
                if (sError) 
                {
                    logErrorP("Failed to set ASC target");
                    if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u failed", mSensorVariant);
                }
                else
                {
                    logInfoP("SCD4%u automatic self calibration enabled, target is %i ppm", mSensorVariant, lTarget);
                    if (iDebugKo) openknx.console.writeDiagnoseKo("ASC X %4u ppm", lTarget);
                }
            }
            lResult = true;
        }
        else if (iCmd.find("scd force ") == 0)
        {
            int16_t lCo2 = getIntArg(iCmd);
            if (lCo2 < 400 || lCo2 > 5000) 
            {
                logInfoP("Invalid CO2 value, must be between 400 and 5000 ppm");
                if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u invalid", mSensorVariant);
                lResult = true;
            }
            else
            {
                uint16_t frcCorr = 0;
                sError = performForcedRecalibration(lCo2, frcCorr);
                if (sError || frcCorr == 0xFFFF) 
                {
                    logErrorP("Failed to force recalibration");
                    if (iDebugKo) openknx.console.writeDiagnoseKo("SCD4%u failed", mSensorVariant);
                }
                else
                {
                    logInfoP("SCD4%u forced recalibration to %i ppm successful", mSensorVariant, lCo2);
                    if (iDebugKo) openknx.console.writeDiagnoseKo("FORCE %4u ppm", lCo2);
                }
                lResult = true;
            }
        }    
    } 
    else
    {
        logInfoP("Sensor not stopped, stop first");
        if (iDebugKo) openknx.console.writeDiagnoseKo("Stop first");
        lResult = true;
    }
    return lResult;
}

#endif

