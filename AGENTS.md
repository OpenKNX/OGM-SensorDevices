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

---

## OneWire Stack

The OneWire subsystem is **entirely separate** from the I²C sensor stack above.
It has its own compile guard, its own class hierarchy, and is consumed by
**OFM-OneWireModule** (not by OFM-SensorModule or OFM-PresenceModule).

### Compile Guard

```cpp
#ifdef WIREMODULE
```

All OneWire source files are wrapped in this guard. It is defined by the device
project when OFM-OneWireModule is included.

### Architecture

```
OneWireDS2482          (bus master — I²C ↔ 1-Wire bridge, DS2482-100/800 or DS2484)
  │
  ├── OneWireSearch    (ROM search algorithm — one instance per search type per BM)
  │
  └── OneWire[]        (up to 30 device slots per bus master)
        ├── OneWireDS18B20    (DS18B20 / DS18S20 / DS1822 / DS1825 — temperature)
        ├── OneWireDS1990     (DS1990A — iButton)
        ├── OneWireDS2438     (battery/multi-sensor — temp + VDD + VAD + VSens)
        │     └── OneWireDS2438Fromula  (NTC / HIH-4010 humidity / brightness formulas)
        ├── OneWireDS2408     (8-channel PIO expander)
        └── OneWireDS2413     (2-channel PIO expander)
```

### `OneWire` Base Class (`src/OneWire.h/.cpp`)

Abstract base for all 1-Wire devices.

| Member | Meaning |
|---|---|
| `pId[7]` | 7-byte ROM ID (CRC byte not stored) |
| `pMode` | `New` / `Connected` / `Disconnected` |
| `pPrio` | `PrioHigh` (iButton) / `PrioNormal` (IO) / `PrioLow` (sensors) |
| `pBM` | Pointer to owning `OneWireDS2482` — set by bus master on `addSensor()` |
| `pDelay` | `millis()` timestamp for non-blocking timing |
| `pValid` | Whether the last measurement is valid |

**Static factory**: `OneWire::factory(tIdRef iId, bool* eIsNew)` instantiates the
correct subclass by family byte (byte 0 of the ROM ID). Stores up to
`COUNT_1WIRE_CHANNEL` sensors in a static internal array — creates a new instance
only the first time a given ID is seen (`*eIsNew = true`), returns the existing one
on subsequent calls.

**Disappear tolerance**: `cDisappearCount = 2` — a device must be absent from 2
consecutive bus searches before transitioning from `Connected` to `Disconnected`.

**Family codes:**

| Macro | Hex | Device |
|---|---|---|
| `MODEL_DS18S20` | 0x10 | Temperature |
| `MODEL_DS18B20` | 0x28 | Temperature |
| `MODEL_DS1822` | 0x22 | Temperature |
| `MODEL_DS1825` | 0x3B | Temperature |
| `MODEL_DS2438` | 0x26 | Battery/multi-sensor |
| `MODEL_DS2413` | 0x3A | 2-channel IO |
| `MODEL_DS2408` | 0x29 | 8-channel IO |
| `MODEL_DS1990` | 0x01 | iButton |

### Bus Master: `OneWireDS2482` (`src/OneWireDS2482.h/.cpp`)

Drives the Maxim DS2482/DS2484 I²C-to-1-Wire bridge chip via Arduino `Wire`.

**State machine:** `Init → Startup → SearchIButton → ProcessIO → ProcessSensors →
SearchNewDevices → Idle → (back to SearchIButton)` / `Error` (retries every 1 s,
checks for short circuit).

**Priority dispatch:**
- `ProcessIButton()`: runs every state-machine cycle; uses a dedicated
  `mSearchPrio` (`OneWireSearch`) to do a targeted search for each known iButton.
  State assignment after this call is unconditional (Error state from inside the
  method is ignored — intentional for iButton robustness).
- `ProcessPriorityBusUse()`: iterates IO expanders (DS2408/DS2413), one per cycle.
- `ProcessNormalBusUse()`: iterates slow sensors (DS18B20, DS2438), one per cycle.
- `SearchNewDevices`: discovers unknown bus members via `mSearchNormal`; can be
  disabled at `setup()` time.

**Error handling:**
- `waitOnBusy()`: polls up to 1000 × 20 µs; sets `DS2482_ERROR_TIMEOUT` on timeout.
- `wireReset()`: sets `DS2482_ERROR_SHORT` if SD bit set after reset.
- `writeConfig()`: sets `DS2482_ERROR_CONFIG` on readback mismatch.
- `Error` state: rechecks I²C presence every 1 s; calls
  `openknx.unsupportedEtsModule()` if chip permanently absent.

**`setupTiming(tRSTL, tMSP, tW0L, tREC0, RWPU)`**: DS2484-only port parameter
adjustment — must be called after `setup()` and only if `checkI2cPresence()` returns
true. Currently only called for busmaster 0 in OFM-OneWireModule.

### DS18B20 / DS18S20 (`src/OneWireDS18B20.h/.cpp`)

State machine: `Startup → Idle ↔ StartMeasurement (750 ms) → GetMeasure → Idle`.
Supports 9–12 bit resolution (DS18B20) and fixed 9-bit (DS18S20).
Parasite power: detected via `READPOWERSUPPLY`; issues `STARTCONVO` with strong
pullup and delays `COPYSCRATCH` by 10 ms. CRC validated on every scratchpad read.

### DS2438 Multi-Sensor (`src/OneWireDS2438.h/.cpp`)

Measures: on-chip temperature + VDD (supply) + VAD (external ADC) + VSens (current
sense). State machine progresses through `StartMeasureTemp → GetMeasureTemp →
SetConfigVDD → StartMeasureVDD → GetMeasureVDD → SetConfigVAD → StartMeasureVAD →
GetMeasureVAD → Idle`. All values are committed atomically at `Idle`.

**Model function constants (consumed by OFM-OneWireModule via `getModelFunction()`):**

| Constant | Returns |
|---|---|
| `FmlRaw_TemperatureOnChip` (10) | On-chip temperature (°C) |
| `FmlRaw_VoltageOfChip` (11) | VDD (V) |
| `FmlRaw_VoltageOfADC1` (12) | VAD (V) |
| `FmlRaw_VoltageOfSens` (13) | VSens (V) |
| `FmlNative_TemperatureNTC` (15) | NTC temperature via Steinhart-Hart |
| `FmlNative_Humidity` (16) | Relative humidity via HIH-4010 formula |
| `FmlNative_Brightness` (17) | Lux via 47 Ω photodiode formula |
| `FmlUser_Start`..`FmlUser_End` (50..79) | 30 user-defined function slots |

### DS2438 Formula Layer (`src/OneWireDS2438Fromula.h/.cpp`)

> **Note:** The class name and filenames spell "Fromula" (missing 'r'). This typo is
> present throughout — in both file names, the class name, and all `#include` references.
> Do **not** silently rename it without a full codebase-wide search.

Static utility class; `OneWireDS2438` is declared `friend`. Holds a static array
`userFunction[30]` of `(float, float, float, float) → float` function pointers.

**Native formulas:**
- `nativeTemperatureNTC`: Steinhart-Hart, Rref = 100 kΩ, b = 3590, Tn = 298.15 K.
- `nativeHumidity`: HIH-4010 formula, temperature-compensated.
- `nativeBrightness`: `pow(10, VSens×1000/47)` — interprets VSens through 47 Ω as
  photodiode current.

### DS2408 / DS2413 (`src/OneWireDS2408.h/.cpp`, `src/OneWireDS2413.h/.cpp`)

IO expanders, `PrioNormal`. DS2408: 8 open-drain pins; DS2413: 2 open-drain pins.
Write handshake: send byte + complement, expect 0xAA + echo. `mIoMask` / `mIoInvertMask`
configurable per bit.

### Search Algorithm (`src/OneWireSearch.h/.cpp`)

Implements the Dallas ROM search algorithm via DS2482 triplet commands.
`wireSearchBlocking()` finds one device per call (~91 ms), using the 64-bit discrepancy
tracking variables. CRC8 validated at end of each found ID. Calls
`searchLoopCallback()` every 4 bits to yield to the main loop.

### CRC

Dow-CRC8 (X^8 + X^5 + X^4 + X^0). Two implementations selectable via
`ONEWIRE_CRC8_TABLE`: table-based (2 × 16 entries, `pgm_read_byte`) or bit-by-bit.
CRC is recomputed on every `wireSelect` (7 ID bytes → 1 CRC byte sent over the bus).

