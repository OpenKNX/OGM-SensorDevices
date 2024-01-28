// #include "IncludeManager.h"
#if defined(SENSORMODULE) || defined(PMMODULE)
    #include "Sensor.h"
    // #include "OpenKNX/Hardware.h"
    #include "SensorBME280.h"
    #include "SensorBME680.h"
    #include "SensorDummy.h"
    #include "SensorIAQCore.h"
    #include "SensorMR24xxB1.h"
    #include "SensorOPT300x.h"
    #include "SensorSCD30.h"
    #include "SensorSCD40.h"
    #include "SensorSCD41.h"
    #include "SensorSGP30.h"
    #include "SensorSHT3x.h"
    #include "SensorVEML7700.h"
    #include "SensorHLKLD2420.h"
    #include "SensorVL53L1X.h"

Sensor* newSensor(uint8_t iSensorClass, MeasureType iMeasureType)
{
    Sensor* lSensor = nullptr;
    switch (iSensorClass)
    {
    #ifdef SENSORMODULE
        case SENS_SHT3X:
            lSensor = new SensorSHT3x(iMeasureType);
            break;

        case SENS_BME280:
            lSensor = new SensorBME280(iMeasureType);
            break;

        case SENS_BME680:
            lSensor = new SensorBME680(iMeasureType);
            break;
        #ifdef SENSOR_SCD30_SUPPORT
        case SENS_SCD30:
            lSensor = new SensorSCD30(iMeasureType);
            break;
        #endif
        case SENS_SCD40:
            lSensor = new SensorSCD40(iMeasureType);
            break;
        case SENS_SCD41:
            lSensor = new SensorSCD41(iMeasureType);
            break;

        case SENS_IAQCORE:
            lSensor = new SensorIAQCore(iMeasureType);
            break;

        case SENS_VL53L1X:
            lSensor = new SensorVL53L1X(iMeasureType);
            break;

        case SENS_SGP30:
            lSensor = new SensorSGP30(iMeasureType);
            break;
    #endif
    #if defined(SENSORMODULE) || defined(PMMODULE)
        case SENS_OPT300X:
            lSensor = new SensorOPT300x(iMeasureType);
            break;

        case SENS_VEML7700:
            lSensor = new SensorVEML7700(iMeasureType);
            break;

    #endif
    #ifdef PMMODULE
        #ifdef HF_POWER_PIN
        case SENS_MR24xxB1:
            lSensor = new SensorMR24xxB1(iMeasureType);
            break;
        case SENS_HLKLD2420:
            lSensor = new SensorHLKLD2420(iMeasureType);
            break;
        #endif
    #endif
        default:
            lSensor = nullptr;
            break;
    }
    if (lSensor == nullptr)
    {
        lSensor = new SensorDummy(iMeasureType);
    }
    if (lSensor == nullptr)
    {
        openknx.hardware.fatalError(FATAL_SENS_UNKNOWN, "Unknown Sensor or sensor creation error!");
    }
    return lSensor;
}
#endif
