# OGM-SensorDevices – Agent Instructions

## Project

**OGM-SensorDevices** is an **OpenKNX sensor library** that provides a
unified, non-blocking API for accessing I²C and 1-Wire sensors. It is
**not a standalone application** — it is pulled in as a dependency into
device projects (OAMs) together with OFMs such as OFM-SensorModule or
OFM-PresenceModule.

> **Not self-contained:** There is no `platformio.ini`. Compilation
> requires a concrete device project that defines `SENSORMODULE` and/or
> `PMMODULE`. All sensor implementation files are wrapped in
> `#ifdef SENSORMODULE` or `#if defined(SENSORMODULE) || defined(PMMODULE)`.

- **Language**: C++ (Arduino framework)
- **Experience level**: Advanced — assume knowledge of Arduino, I²C,
  RP2040/ESP32, and OpenKNX fundamentals.
- **Branch convention**: `v1` = stable, `v1dev` = active development.
- **All source files live in `src/`** — there are no headers in `include/`.

## Architecture

### Core Classes

#### `Sensor` (`src/Sensor.h`, `src/Sensor.cpp`)

Abstract base class for every sensor. Key members:

| Member | Type | Purpose |
|--------|------|---------|
| `pWire` | `TwoWire*` | I²C bus instance |
| `pI2CAddress` | `uint8_t` | I²C address (0 = auto-detect) |
| `pMeasureTypes` | `uint16_t` | Bitmask of registered `MeasureType` values |
| `pSensorState` | `SensorState` | Current state machine state |
| `pSensorStateDelay` | `uint32_t` | `millis()` timestamp for delays |
| `pTempOffset` | `float` | Temperature offset (default 0.0) |
| `pWakeupRetries` | `uint8_t` | Wakeup retry counter (max 10, then → Off) |

**Pure virtual (must override):**
- `float measureValue(MeasureType)` — return the measured float for the given type
- `uint8_t getSensorClass()` — return the `SENS_*` constant for this sensor

**Virtual (override as needed):**
- `bool begin()` — one-time init; base calls `checkSensorConnection()`
- `void sensorLoopInternal()` — state machine; call `Sensor::sensorLoopInternal()`
  from subclass to advance the state
- `uint8_t getI2cSpeed()` — bus speed in units of 100 kHz (default 1 = 100 kHz)
- `bool prepareTemperatureOffset(float)` — base returns `false`; override to
  store into `pTempOffset` and return `true`
- `uint16_t sensorFlashSize()`, `sensorReadFlash()`, `sensorWriteFlash()`,
  `sensorSavePower()` — flash persistence (see below)
- `std::string logPrefix()` — log tag, e.g. `"Sensor<SHT4x>"`

**Public non-virtual:** `bool measureValue(MeasureType, float& eValue)` —
safe public accessor: checks `pMeasureTypes`, confirms `pSensorState == Running`,
delegates to the protected pure virtual, and rejects `NO_NUM` results.

#### `SensorDevices` (`src/SensorDevices.h/.cpp`)

Extends `OpenKNX::Module`. Holds an array of `Sensor*` up to
`MAX_SUPPORTED_SENSORS` (default 10). Drives `begin()` and
`sensorLoopInternal()` for all registered sensors.

**Consumer entry point:**
```cpp
Sensor* openknxSensorDevicesModule.factory(uint8_t iSensorClass,
                                            MeasureType iMeasureType);
```
- If a sensor of `iSensorClass` already exists, returns the existing
  instance and adds `iMeasureType` to it (deduplication).
- Otherwise heap-allocates a new sensor via `newSensor()` in
  `SensorFactory.cpp`.
- Tracks the minimum I²C speed across all sensors to set the bus clock.

#### `SensorFactory.cpp`

Contains `SensorDevices::newSensor()` — a `switch(iSensorClass)` that
instantiates the concrete class. **This is the only file that `#include`s
all sensor headers.** When adding a new sensor: add the `#include` here
and a `case SENS_XXX:` inside the appropriate `#ifdef` guard.

### State Machine

Every sensor runs the same state machine via `sensorLoopInternal()`:

```
Nil → Wakeup → Calibrate → Finalize → Running
                                ↓ (begin() fails)
                               Off
```

- **Wakeup**: calls `Sensor::sensorLoopInternal()` which probes the I²C
  bus (`checkSensorConnection()`), calls `begin()` on success, and
  advances to `Calibrate`. If `begin()` fails, retries up to 10 times
  (1 s apart via `pWakeupRetries`) before transitioning to `Off`.
- **Calibrate**: sensor-specific init (e.g. restore flash baseline, send
  first measurement command). Call `Sensor::sensorLoopInternal()` to
  advance to `Finalize`.
- **Finalize**: wait for first valid data (sensor-specific delay). Then
  transition to `Running` manually.
- **Running**: continuous non-blocking measurement loop.
- **Off**: sensor not available; `Wakeup` retries periodically.

Timing is driven by `delayCheck(pSensorStateDelay, ms)`. Never use
`delay()`.

### `MeasureType` Enum and `SENS_*` Constants

**Sensor class IDs** (`SENS_*`, defined in `Sensor.h`):

| Constant | Value | Sensor |
|----------|-------|--------|
| `SENS_NO` | 0 | None |
| `SENS_SHT3X` | 1 | Temp/Hum |
| `SENS_BME280` | 2 | Temp/Hum/Pressure |
| `SENS_BME680` | 3 | Temp/Hum/Pressure/Voc |
| `SENS_SCD30` | 4 | Temp/Hum/Co2 |
| `SENS_IAQCORE` | 5 | Voc |
| `SENS_OPT300X` | 6 | Lux |
| `SENS_VL53L1X` | 7 | Tof |
| `SENS_SGP30` | 8 | Voc/Co2Calc |
| `SENS_SCD40` | 9 | Temp/Hum/Co2 |
| `SENS_MR24xxB1` | 10 | HF-Presence |
| `SENS_VEML7700` | 11 | Lux |
| `SENS_HLKLD2420` | 12 | HF-Presence |
| `SENS_SCD41` | 13 | Temp/Hum/Co2 |
| `SENS_MAX31865` | 14 | PT100/PT1000 |
| `SENS_SHT4X` | 15 | Temp/Hum |

**`MeasureType` values** (bitmask, same as `BIT_*` defines):

| Constant | Value |
|----------|-------|
| `Temperature` | 2 |
| `Humidity` | 4 |
| `Pressure` | 8 |
| `Voc` | 16 |
| `Co2` | 32 |
| `Co2Calc` | 64 (estimated CO₂, e.g. SGP30) |
| `Accuracy` | 128 |
| `Lux` | 256 |
| `Tof` | 512 |
| `Pres` | 1024 |

## Sensor Implementations

### Pattern: Direct Wire (no external library)

Used by: `SensorSHT3x`, `SensorSHT4x`, `SensorOPT300x`

- Write commands via `pWire->beginTransmission` / `write` / `endTransmission`
- Read data via `pWire->requestFrom` / `read`
- CRC validation inline
- SHT3x: 16-bit commands (2 bytes, MSB first). SHT4x: 8-bit commands (1 byte).

### Pattern: Multiple Inheritance with External Library

Used by: `SensorBME280`, `SensorBME680`, `SensorSCD40`, `SensorSGP30`

```cpp
class SensorXXX : public Sensor, protected AdafruitXXX { ... }
```

The constructor chains both bases:
```cpp
SensorXXX::SensorXXX(uint16_t iMeasureTypes, TwoWire* iWire)
    : Sensor(iMeasureTypes, iWire, ADDR), AdafruitXXX() {}
```

The Adafruit/Sensirion library is initialized inside `begin()` with the
already-probed `pWire`.

### Pattern: Composition (member object)

Used by: `SensorVEML7700`

The library instance is a protected member, initialized in `begin()`.

### I²C Address Auto-Detection

For sensors with multiple possible addresses (e.g. SHT4x: 0x44/0x45,
BME680: 0x76/0x77), use `pI2CAddress = 0` in the default constructor and
probe in `begin()`:

```cpp
if (pI2CAddress == 0)
{
    pI2CAddress = SENSOR_ADDR_PRIMARY;
    lResult = Sensor::begin();
    if (!lResult)
    {
        pI2CAddress = SENSOR_ADDR_SECONDARY;
        lResult = Sensor::begin();
    }
}
else
{
    lResult = Sensor::begin();
}
```

### Temperature Offset and Humidity Compensation

Sensors that measure Temperature and/or Humidity implement:

```cpp
bool prepareTemperatureOffset(float iTempOffset) override
{
    pTempOffset = iTempOffset;
    return true;
}
```

In `measureValue()`:
- **Temperature**: `return mTemp - pTempOffset;`
- **Humidity** (Magnus formula — mathematically equivalent to converting
  through absolute humidity):

```cpp
if (pTempOffset == 0.0f || mHumidity == NO_NUM || mTemp == NO_NUM)
    return mHumidity;
float tMeas = mTemp;
float tReal  = mTemp - pTempOffset;
float rh = mHumidity * expf(17.269f * (tMeas / (237.3f + tMeas)
                                      - tReal  / (237.3f + tReal)));
if (rh < 0.0f)   rh = 0.0f;
if (rh > 100.0f) rh = 100.0f;
return rh;
```

Sensors that do NOT measure T/H themselves but need T/H for internal
compensation read the values directly from KNX group objects — guarded
so the code compiles out when those KOs are not defined:

```cpp
void SensorXXX::processHumidity()
{
#if defined(KoSENS_Temp) && defined(KoSENS_Hum)
    if (!KoSENS_Temp.initialized() || !KoSENS_Hum.initialized()) return;
    float lTemp = KoSENS_Temp.value(DPT_Value_Temp);
    float lHum  = KoSENS_Hum.value(DPT_Value_Humidity);
    // values from the KO are already offset-corrected
    if (lTemp < -40.0f || lTemp > 80.0f || lHum < 0.0f || lHum > 100.0f) return;
    setHumidity(getAbsoluteHumidity(lTemp, lHum));
#endif
}
```

Call `processHumidity()` inside the `Running` case before each measurement
(see `SensorSGP30`, `SensorSCD40::processPressure()` as reference).

**SCD40/SCD41**: have native temperature offset support via
`setTemperatureOffset()` (Sensirion library) — the sensor chip corrects
both Temperature and Humidity output internally. No software Magnus
formula needed. `prepareTemperatureOffset()` stores the value and
`calibrate()` passes it to the library.

## Flash Persistence

For sensors with calibration state (e.g. gas baselines), override the four
flash methods following the BME680 pattern:

```cpp
uint16_t sensorFlashSize() override;          // total bytes: 1 + 4 + payload
void sensorReadFlash(const uint8_t* iBuffer,
                     const uint16_t iSize) override;
void sensorWriteFlash() override;
void sensorSavePower() override;              // snapshot volatile state
```

**Flash layout**: `[version(1)] [magic(4)] [payload(n)]`

- `sensorReadFlash`: validate version byte (`== 1`) and 4 magic bytes;
  if valid, store `mFlashBuffer = iBuffer + 5` and call
  `openknx.flash.read(sensorFlashSize() - 1)` to advance the stream cursor.
- `sensorWriteFlash`: write version + magic + payload via
  `openknx.flash.writeByte()` / `openknx.flash.write()`.
- `sensorSavePower`: snapshot current sensor state into `mWorkBuffer`.
- `sensorWriteFlash` calls `sensorSavePower()` lazily if
  `mWorkBufferInitialized == false`.
- Restore the saved state in the `Calibrate` case of `sensorLoopInternal()`
  using `mFlashBuffer` (non-null = valid data available).

## Adding a New Sensor

1. Create `src/SensorXXX.h` and `src/SensorXXX.cpp` inside `#ifdef SENSORMODULE`
2. Inherit from `Sensor` (and optionally from the library class)
3. Override `getSensorClass()`, `measureValue()`, `sensorLoopInternal()`,
   `logPrefix()`, `begin()`, `getI2cSpeed()`
4. Add `#define SENS_XXX <next_id>  // MeasureTypes` to `Sensor.h`
5. Add `#include "SensorXXX.h"` to `SensorFactory.cpp`
6. Add `case SENS_XXX: lSensor = new SensorXXX(iMeasureType, iWire); break;`
   inside the appropriate `#ifdef` guard in `SensorFactory.cpp`
7. If an external library is needed, add it to `library.json`

## External Libraries (`library.json`)

| Library | Sensor |
|---------|--------|
| `adafruit/Adafruit BME280 Library` | SensorBME280, SensorBME680 (via Adafruit) |
| `adafruit/Adafruit Unified Sensor` | Adafruit base |
| `adafruit/Adafruit BusIO` | Adafruit I²C/SPI base |
| `adafruit/Adafruit VEML7700 Library` | SensorVEML7700 |
| `adafruit/Adafruit SGP30 Sensor` @ ^2.0.3 | SensorSGP30 |
| `sparkfun/SparkFun SCD30 Arduino Library` | SensorSCD30 |
| `sensirion/Sensirion I2C SCD4x` | SensorSCD40/SCD41 |
| `sensirion/Sensirion Core` | Sensirion base |
| `pololu/VL53L1X` | SensorVL53L1X |

BME680 uses the BSEC2 binary library from Bosch (included in `src/bsec/`),
not a PlatformIO library entry. SHT3x, SHT4x, and OPT300x use no external
library (direct Wire communication).

## Compile Guards

All sensor code is guarded:
- `#ifdef SENSORMODULE` — full sensor set (all I²C sensors)
- `#if defined(SENSORMODULE) || defined(PMMODULE)` — shared sensors
  (OPT300x, VEML7700) used by both SensorModule and PresenceModule
- `#ifdef PMMODULE` — presence sensors only (MR24xxB1, HLKLD2420)

## Code Conventions

Identical to OGM-Common (see `../OGM-Common/AGENTS.md`):
- **No `delay()`** — all timing via `delayCheck(timestamp, ms)`
- **Allman-style braces** (always on their own line)
- **4-space indentation, no tabs**
- Comments in German or English (consistent per file)
- `NO_NUM` is the sentinel value for "no valid reading yet" — return it
  from `measureValue()` until the sensor has a real value

## References

- [README.md](README.md) — sensor overview
- [`../OGM-Common/AGENTS.md`](../OGM-Common/AGENTS.md) — base framework,
  logging API, flash API, code conventions
