#pragma once

#include <cstdint>
#include <Arduino.h>

#ifndef BOARD_ENDUSER
// Board specific definietions
// #define BOARD_MASIFI
// ################################################
// ### Board Configuration
// ################################################
#ifdef BOARD_DEVEL
#define PROG_LED_PIN 26
#define PROG_LED_PIN_ACTIVE_ON LOW
#define PROG_BUTTON_PIN 10
#define PROG_BUTTON_PIN_INTERRUPT_ON RISING
#define SAVE_INTERRUPT_PIN 17
#define BUZZER_PIN 18
#define I2C_EEPROM_DEVICE_ADDRESSS 0xFF // Address of 24LC256 eeprom chip
#endif
#ifdef BOARD_ENOCEAN
#ifndef PROG_LED_PIN
#define PROG_LED_PIN 10
#define PROG_LED_PIN_ACTIVE_ON HIGH
#endif
#ifndef PROG_BUTTON_PIN
#define PROG_BUTTON_PIN 8
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#endif
#define SAVE_INTERRUPT_PIN A9
// #define INFO_LED_PIN 38
// #define INFO_LED_PIN_ACTIVE_ON HIGH
// #define COUNT_LOG_CHANNEL 80
// Buzzer
// #define BUZZER_PIN 9
// #define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
#define NO_I2C
#endif
#ifdef BOARD_MASIFI_V1
#define PROG_LED_PIN 13
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN 11
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define SAVE_INTERRUPT_PIN 8
#define INFO_LED_PIN 38
#define INFO_LED_PIN_ACTIVE_ON HIGH
#define COUNT_1WIRE_BUSMASTER 1
#define COUNT_1WIRE_CHANNEL 30
// #define COUNT_PM_CHANNEL 6
// #define COUNT_LOG_CHANNEL 80
// Buzzer
#define BUZZER_PIN 9
#define I2C_1WIRE_DEVICE_ADDRESSS 0x1A  // Address of DS2482 1-Wire-Busmaster chip
#define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
#define I2C_RGBLED_DEVICE_ADDRESS 0x60  // Address of PCA9632 RBGW-LED-Driver
#endif
#ifdef BOARD_MASIFI_V2
#define PROG_LED_PIN 13
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN 11
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define SAVE_INTERRUPT_PIN 8
#define INFO_LED_PIN 38
#define INFO_LED_PIN_ACTIVE_ON HIGH
#define COUNT_1WIRE_BUSMASTER 1
#define COUNT_1WIRE_CHANNEL 30
// #define COUNT_PM_CHANNEL 6
// #define COUNT_LOG_CHANNEL 99
// Buzzer
#define BUZZER_PIN 9
#define I2C_1WIRE_DEVICE_ADDRESSS 0x18 // Address of DS2484 1-Wire-Busmaster chip
#define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
#define I2C_RGBLED_DEVICE_ADDRESS 0x60  // Address of PCA9632 RBGW-LED-Driver
#endif
#ifdef BOARD_MASIFI_V3
#define PROG_LED_PIN 13
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN 11
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define SAVE_INTERRUPT_PIN A2 // 8
#define INFO_LED_PIN 38
#define INFO_LED_PIN_ACTIVE_ON HIGH
#define COUNT_1WIRE_BUSMASTER 1
#define COUNT_1WIRE_CHANNEL 30
// #define COUNT_PM_CHANNEL 6
// #define COUNT_LOG_CHANNEL 80
// Buzzer
#define BUZZER_PIN 9
#define I2C_1WIRE_DEVICE_ADDRESSS 0x18  // Address of DS2484 1-Wire-Busmaster chip
#define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
#define I2C_RGBLED_DEVICE_ADDRESS 0x60  // Address of PCA9632 RBGW-LED-Driver
#endif
#ifdef BOARD_MASIFI_V31
#define PROG_LED_PIN 13
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN 12
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define SAVE_INTERRUPT_PIN A2 // 8
#define INFO_LED_PIN 38
#define INFO_LED_PIN_ACTIVE_ON HIGH
#define COUNT_1WIRE_BUSMASTER 1
#define COUNT_1WIRE_CHANNEL 30
// #define COUNT_PM_CHANNEL 6
// #define COUNT_LOG_CHANNEL 80
// Buzzer
#define BUZZER_PIN 9
#define I2C_1WIRE_DEVICE_ADDRESSS 0x18  // Address of DS2484 1-Wire-Busmaster chip
#define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
#define I2C_RGBLED_DEVICE_ADDRESS 0x60  // Address of PCA9632 RBGW-LED-Driver
#endif
#ifdef BOARD_MASIFI_AUSSEN_V13
#define PROG_LED_PIN 13
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN 12
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define SAVE_INTERRUPT_PIN A2 // 8
#define INFO_LED_PIN 38
#define INFO_LED_PIN_ACTIVE_ON HIGH
#define COUNT_1WIRE_BUSMASTER 1
#define COUNT_1WIRE_CHANNEL 30
// #define COUNT_LOG_CHANNEL 80
// Buzzer
#define BUZZER_PIN 9
#define I2C_1WIRE_DEVICE_ADDRESSS 0x18  // Address of DS2484 1-Wire-Busmaster chip
#define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
#define I2C_RGBLED_DEVICE_ADDRESS 0x60  // Address of PCA9632 RBGW-LED-Driver
#endif
// Sensormodul auf RP2040 Basis
#ifdef BOARD_MASIFI_PICO
#define PROG_LED_PIN 12
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN 28
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define SAVE_INTERRUPT_PIN D29 // 8
// #define INFO_LED_PIN 38
// #define INFO_LED_PIN_ACTIVE_ON HIGH
#define COUNT_1WIRE_BUSMASTER 1
#define COUNT_1WIRE_CHANNEL 30
// #define COUNT_PM_CHANNEL 6
// #define COUNT_LOG_CHANNEL 99
#define KNX_UART_RX_PIN 17
#define KNX_UART_TX_PIN 16
#define KNX_I2C_SDA_PIN 20
#define KNX_I2C_SCL_PIN 21
// Buzzer
#define BUZZER_PIN 27
// #define I2C_1WIRE_DEVICE_ADDRESSS 0x18  // Address of DS2484 1-Wire-Busmaster chip
// #define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
// #define I2C_RGBLED_DEVICE_ADDRESS 0x60  // Address of PCA9632 RBGW-LED-Driver
#endif
// HF-Firmware-Test auf RP2040-Sensormodul Basis (reiner Test)
#ifdef BOARD_MASIFI_PICO_SEN_PM_TEST
#define PROG_LED_PIN 12
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN 28
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define SAVE_INTERRUPT_PIN D29 // 8
// #define INFO_LED_PIN 38
// #define INFO_LED_PIN_ACTIVE_ON HIGH
// #define COUNT_1WIRE_BUSMASTER 1
// #define COUNT_1WIRE_CHANNEL 30
// #define COUNT_PM_CHANNEL 6
// #define COUNT_LOG_CHANNEL 99
#define KNX_UART_RX_PIN 17
#define KNX_UART_TX_PIN 16
#define KNX_I2C_SDA_PIN 20
#define KNX_I2C_SCL_PIN 21
// Buzzer
#define BUZZER_PIN 27
#define PRESENCE_LED_PIN 9
#define PRESENCE_LED_PIN_ACTIVE_ON HIGH
#define MOVE_LED_PIN 8
#define MOVE_LED_PIN_ACTIVE_ON HIGH
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
#define HF_UART_TX_PIN 4
#define HF_UART_RX_PIN 5
#define HF_S1_PIN 6
#define HF_S2_PIN 7
#define HF_POWER_PIN 14
// #define I2C_1WIRE_DEVICE_ADDRESSS 0x18  // Address of DS2484 1-Wire-Busmaster chip
// #define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
// #define I2C_RGBLED_DEVICE_ADDRESS 0x60  // Address of PCA9632 RBGW-LED-Driver
#endif
#ifdef BOARD_MASIFI_HFPM_DEVEL
#define PROG_LED_PIN 16
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN 17
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define INFO_LED_PIN 10
#define INFO_LED_PIN_ACTIVE_ON HIGH
#define PRESENCE_LED_PIN 9
#define PRESENCE_LED_PIN_ACTIVE_ON HIGH
#define MOVE_LED_PIN 8
#define MOVE_LED_PIN_ACTIVE_ON HIGH
#define KNX_UART_TX_PIN 12
#define KNX_UART_RX_PIN 13
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 3
#define HF_UART_TX_PIN 4
#define HF_UART_RX_PIN 5
#define HF_S1_PIN 6
#define HF_S2_PIN 7
#define HF_POWER_PIN 14
#define SAVE_INTERRUPT_PIN 15
// #define PIR_PIN 14
// #define COUNT_PM_CHANNEL 15
// #define COUNT_LOG_CHANNEL 80
// Buzzer
// #define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
#endif
#ifdef BOARD_MASIFI_HFPM_DEVEL2
#define PROG_LED_PIN 18
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN 17
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
// #define INFO_LED_PIN 9
// #define INFO_LED_PIN_ACTIVE_ON HIGH
#define PRESENCE_LED_PIN 0
#define PRESENCE_LED_PIN_ACTIVE_ON HIGH
#define MOVE_LED_PIN 1
#define MOVE_LED_PIN_ACTIVE_ON HIGH
#define KNX_UART_TX_PIN 12
#define KNX_UART_RX_PIN 13
#define I2C_SDA_PIN 26 //I2C1
#define I2C_SCL_PIN 27 //I2C1
#define HF_UART_TX_PIN 4
#define HF_UART_RX_PIN 5
#define HF_S1_PIN 2
#define HF_S2_PIN 3
#define HF_POWER_PIN 10
#define SAVE_INTERRUPT_PIN 11
#endif
#ifdef BOARD_MASIFI_PM
#define PROG_LED_PIN 13
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN 11
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define SAVE_INTERRUPT_PIN 0
#define INFO_LED_PIN 26
#define INFO_LED_PIN_ACTIVE_ON HIGH
#define PRESENCE_LED_PIN 8
#define PRESENCE_LED_PIN_ACTIVE_ON HIGH
#define PIR_PIN 14
// #define COUNT_PM_CHANNEL 15
// #define COUNT_LOG_CHANNEL 80
// Buzzer
#define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
#endif
#ifdef BOARD_PM_V3
#define PROG_LED_PIN 13
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN 11
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define SAVE_INTERRUPT_PIN A2 // 8
#define INFO_LED_PIN 38
#define INFO_LED_PIN_ACTIVE_ON HIGH
#define COUNT_1WIRE_BUSMASTER 1
#define COUNT_1WIRE_CHANNEL 30
// #define COUNT_PM_CHANNEL 20
// #define COUNT_LOG_CHANNEL 80
// Buzzer
#define BUZZER_PIN 9
#define I2C_1WIRE_DEVICE_ADDRESSS 0x18  // Address of DS2484 1-Wire-Busmaster chip
#define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
#define I2C_RGBLED_DEVICE_ADDRESS 0x60  // Address of PCA9632 RBGW-LED-Driver
#endif
#ifdef BOARD_PM_V31
#define PROG_LED_PIN 13
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN 12
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define SAVE_INTERRUPT_PIN A2 // 8
#define INFO_LED_PIN 38
#define INFO_LED_PIN_ACTIVE_ON HIGH
#define COUNT_1WIRE_BUSMASTER 1
#define COUNT_1WIRE_CHANNEL 30
// #define COUNT_PM_CHANNEL 20
// #define COUNT_LOG_CHANNEL 80
// Buzzer
#define BUZZER_PIN 9
#define I2C_1WIRE_DEVICE_ADDRESSS 0x18  // Address of DS2484 1-Wire-Busmaster chip
#define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
#define I2C_RGBLED_DEVICE_ADDRESS 0x60  // Address of PCA9632 RBGW-LED-Driver
#endif
#ifdef BOARD_MASIFI_ONEWIRE
#define PROG_LED_PIN 26
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN A1
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define INFO_LED_PIN 25
#define INFO_LED_PIN_ACTIVE_ON HIGH
#define COUNT_1WIRE_BUSMASTER 3
#define COUNT_1WIRE_CHANNEL 90
// #define COUNT_LOG_CHANNEL 80
#define I2C_1WIRE_DEVICE_ADDRESSS 0x18 // Address of first DS2482 1-Wire-Busmaster chip, used are 0x19, 0x1A, 0x1B
#define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
#endif
#ifdef BOARD_MASIFI_ONEWIRE_ITSYBITSY_M0
#define PROG_LED_PIN 9
#define PROG_LED_PIN_ACTIVE_ON HIGH
#define PROG_BUTTON_PIN A1
#define PROG_BUTTON_PIN_INTERRUPT_ON FALLING
#define INFO_LED_PIN 13
#define INFO_LED_PIN_ACTIVE_ON HIGH
#define COUNT_1WIRE_BUSMASTER 3
#define COUNT_1WIRE_CHANNEL 90
// #define COUNT_LOG_CHANNEL 80
#define I2C_1WIRE_DEVICE_ADDRESSS 0x18 // Address of first DS2482 1-Wire-Busmaster chip, used are 0x19, 0x1A, 0x1B
#define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
#endif
#endif
// board independent definitions

// fatal error codes
#define FATAL_I2C_BUSY                3  // I2C busy during startup
#define FATAL_LOG_WRONG_CHANNEL_COUNT 4  // knxprod contains more channels than logic supports
#define FATAL_SENS_UNKNOWN            5  // unknown or unsupported sensor

// // EEPROM Support
// #define I2C_EEPROM_DEVICE_ADDRESSS 0x50 // Address of 24LC256 eeprom chip
// #define SAVE_BUFFER_START_PAGE     0    // All stored KO data begin at this page and takes 37 pages, 
//                                         // allow 3 pages boundary, so next store should start at page 40

// NCN5130: internal commands
#define U_RESET_REQ          0x01
#define U_STATE_REQ          0x02
#define U_SET_BUSY_REQ       0x03
#define U_QUIT_BUSY_REQ      0x04
#define U_BUSMON_REQ         0x05
#define U_SET_ADDRESS_REQ    0xF1 // different on TP-UART
#define U_SET_REPETITION_REQ 0xF2
#define U_L_DATA_OFFSET_REQ  0x08 //-0x0C
#define U_SYSTEM_STATE       0x0D
#define U_STOP_MODE_REQ      0x0E
#define U_EXIT_STOP_MODE_REQ 0x0F
#define U_ACK_REQ            0x10 //-0x17
#define U_CONFIGURE_REQ      0x18
#define U_INT_REG_WR_REQ     0x28 //-0x2B
#define U_INT_REG_RD_REQ     0x38 //-0x3B
#define U_POLLING_STATE_REQ  0xE0

// NCN5130: control services
#define U_RESET_IND           0x03
#define U_STATE_IND           0x07
#define SLAVE_COLLISION       0x80
#define RECEIVE_ERROR         0x40
#define TRANSMIT_ERROR        0x20
#define PROTOCOL_ERROR        0x10
#define TEMPERATURE_WARNING   0x08
#define U_FRAME_STATE_IND     0x13
#define U_FRAME_STATE_MASK    0x17
#define PARITY_BIT_ERROR      0x80
#define CHECKSUM_LENGTH_ERROR 0x40
#define TIMING_ERROR          0x20
#define U_CONFIGURE_IND       0x01
#define U_CONFIGURE_MASK      0x83
#define AUTO_ACKNOWLEDGE      0x20
#define AUTO_POLLING          0x10
#define CRC_CCITT             0x80
#define FRAME_END_WITH_MARKER 0x40
#define U_FRAME_END_IND       0xCB
#define U_STOP_MODE_IND       0x2B
#define U_SYSTEM_STAT_IND     0x4B

// NCN5130 write internal registers
#define U_INT_REG_WR_REQ_WD 0x28
#define U_INT_REG_WR_REQ_ACR0 0x29
#define U_INT_REG_WR_REQ_ACR1 0x2A
#define U_INT_REG_WR_REQ_ASR0 0x2B

// NCN5130 read internal registers
#define U_INT_REG_RD_REQ_WD   0x38
#define U_INT_REG_RD_REQ_ACR0 0x39
#define U_INT_REG_RD_REQ_ACR1 0x3A
#define U_INT_REG_RD_REQ_ASR0 0x3B

// NCN5130: Analog Control Register 0 - Bit values
#define ACR0_FLAG_V20VEN           0x40
#define ACR0_FLAG_DC2EN            0x20
#define ACR0_FLAG_XCLKEN           0x10
#define ACR0_FLAG_TRIGEN           0x08
#define ACR0_FLAG_V20VCLIMIT       0x04

#define BOARD_HW_EEPROM  0x01
#define BOARD_HW_LED     0x02
#define BOARD_HW_ONEWIRE 0x04
#define BOARD_HW_NCN5130 0x08

// call this BEFORE Wire.begin()
// it clears I2C Bus, calls Wire.begin() and checks which board hardware is available
bool boardCheck();
bool checkUartExistence();
bool initUart();
uint8_t sendUartCommand(const char* iInfo, uint8_t iCmd, uint8_t iResp, uint8_t iLen = 0);

// just work after boardCheck
bool boardWithOneWire();
bool boardWithLed();
bool boardWithEEPROM();
bool boardWithNCN5130();

void ledInfo(bool iOn);
void ledProg(bool iOn);
// Turn off 5V rail from NCN5130 to save power for EEPROM write during knx save operation
void savePower();
// Turn on 5V rail from NCN5130 in case SAVE-Interrupt was false positive
void restorePower();

void fatalError(uint8_t iErrorCode, const char *iErrorText = 0);

uint8_t clearI2cBus();
