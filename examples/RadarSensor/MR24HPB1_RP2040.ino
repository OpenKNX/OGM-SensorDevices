#include "RadarSensor.h"
#include <Wire.h>
//#include "wiring_private.h" // pinPeripheral() function

#define SER_SYNCH_CODE 0x55
#define SER_HEADER_NR_BYTES 5
#define CRC_BYTES 2

#define enable5V 14

#define MESSAGE_HEAD 0x55
//int data[14] = {0};
int i = 0;
int Msg;
int Situation_action;

uint8_t cmd = 0;
uint8_t data[20] = {0};
uint8_t addr[2] = {0};

unsigned char memo[20];



//! uart_getPacket state machine states.
enum STATES_GET_PACKET
{
  //! Waiting for the synchronisation byte 0x55
  GET_SYNC_STATE = 0,
  //! Copying the 4 after sync byte: raw data length (2 bytes), optional data length (1), type (1).
  GET_HEADER_STATE,
  //! Checking the header CRC8 checksum. Resynchronisation test is also done here
  CHECK_DATA_LENGTH,
  //! Copying the data and optional data bytes to the paquet buffer
  GET_DATA_STATE,
  //! Checking the info CRC8 checksum.
  CHECK_CRC16D_STATE,
};

uint8_t u8RxByte;
//! Checksum calculation
uint8_t u16CRC = 0;
//! Nr. of bytes received
uint16_t u16Count = 0;
uint8_t u8Count = 0;
// Data lenhth
uint8_t data_length = 0;
uint8_t u8Raw[20];
STATES_GET_PACKET u8State;


uint32_t DelayTimer = 0;

uint32_t DelayTimer2 = 0;

bool S1 = false;
bool S2 = false;

//radar RADAR;




void setup()
{
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(enable5V, OUTPUT);
  digitalWrite(enable5V, HIGH); // 5V = OFF

  Serial2.setRX(5);
  Serial2.setTX(4);
  Serial2.begin(9600);

  Wire1.setSDA(2); //SDA
  Wire1.setSCL(3); //SCL
  Wire1.begin();

  Serial.begin(115200);
  setResetCMD();
  delay(5000);
  Serial.println("Readly");

  u8State = GET_SYNC_STATE;

  setEmpfindlichkeit(3);
  delay(1000);
  //setScene(5);
}


bool delayCheck(uint32_t iOldTimer, uint32_t iDuration)
{
  return millis() - iOldTimer >= iDuration;
}




void loop()
{
  if (delayCheck(DelayTimer2, 63000))
  {
    //setEmpfindlichkeit(1);
    //readEmpfindlichkeit();
    readScene();
    DelayTimer2 = millis();
  }

  if (delayCheck(DelayTimer, 60000))
  {
    //setEmpfindlichkeit(1);
    readEmpfindlichkeit();
    DelayTimer = millis();
  }

  S1 = digitalRead(6);
  S2 = digitalRead(7);
  digitalWrite(9, S1);
  digitalWrite(8, S2);

  if (Serial2.available() > 0)
  {
    //u8RxByte = Serial2.read();
    //Serial.println(u8RxByte, HEX);

    while (Serial2.readBytes(&u8RxByte, 1) == 1)
    {

      S1 = digitalRead(6);
      S2 = digitalRead(7);
      digitalWrite(9, S1);
      digitalWrite(8, S2);

      switch (u8State)
      {
        // Waiting for packet sync byte 0x55
        case GET_SYNC_STATE:
          //Serial.println(u8RxByte);
          if (u8RxByte == 0x55)
          {
            u8State = GET_HEADER_STATE;
            u16Count = 0;
            u8Count = 0;
            u16CRC = 0;
          }
          break;
        case GET_HEADER_STATE:
          u8Raw[u16Count++] = u8RxByte;
          if (u16Count == SER_HEADER_NR_BYTES)
          {
            u8State = CHECK_DATA_LENGTH;
            //Serial.print("CMD: ");
            //Serial.println(u8Raw[2], HEX);
            cmd = u8Raw[2];
            //Serial.println("daten Länge: ");
            //Serial.println(u8Raw[0], HEX);
            //Serial.println(u8Raw[1], HEX);
            data_length = u8Raw[0];
            //Serial.println("Addr: ");
            //Serial.println(u8Raw[3], HEX);
            addr[0] = u8Raw[3];
            //Serial.println(u8Raw[4], HEX);
            addr[1] = u8Raw[4];
          }
          break;
        case CHECK_DATA_LENGTH:
          u8Raw[u16Count++] = u8RxByte;
          u8Count++;
          if (u8Count == data_length - 7 )
          {
            for (int i = 0; i < u8Count ; i++)
            {
              //Serial.print("Data: ");
              //Serial.println(u8Raw[ 5 + i], HEX);
              data[i] = u8Raw[ 5 + i];
            }
            u8State = CHECK_CRC16D_STATE;
          }
          break;
        case CHECK_CRC16D_STATE:

          u16CRC++;
          u8Raw[u16Count++] = u8RxByte;
          if (u16CRC == CRC_BYTES)
          {
            //Serial.println("CRC: ");
            //Serial.println(u8Raw[u16Count - 1], HEX);
            //Serial.println(u8Raw[u16Count], HEX);
            checkCMD();
            u8State = GET_SYNC_STATE;
          }
          break;
      }
    }
  }
}

void checkCMD()
{
  switch (cmd)
  {
    case 0x03: // Passive reporting of commands
      switch (addr[0])
      {
        case 0x01: //Reporting module identification
          switch (addr[1])
          {
            case 0x01: //Device ID 12 Bytes
              Serial.print(F("Device ID: "));
              for (int i = 0; i < 12; i++)
              {
                Serial.print(data[i]);
                Serial.print((" "));
              }
              Serial.println(F(" "));
              break; // ENDE 0x01: //Device ID 12Bytes
            case 0x02: //SW Version 10Bytes
              Serial.print(("SW_Version: "));
              for (int i = 0; i < 10; i++)
              {
                Serial.print(data[i]);
                Serial.print((" "));
              }
              Serial.println(F(""));
              break; // ENDE 0x02: //SW Version 10Bytes
            case 0x03: //HW Version 8Bytes
              Serial.print(("HW-Version: "));
              for (int i = 0; i < 8; i++)
              {
                Serial.print(data[i]);
                Serial.print((" "));
              }
              Serial.println(F(""));
              break; // ENDE 0x03: //HW Version 8Bytes
            case 0x04: //Protocol version 8Bytes
              Serial.print(("Prot Version: "));
              for (int i = 0; i < 8; i++)
              {
                Serial.print(data[i]);
                Serial.print((" "));
              }
              Serial.println(F(""));
              break; // ENDE 0x04: //Protocol version 8Bytes
          }
          break; // ENDE 0x01: //Reporting module identification
        case 0x03: //Report radar information
          switch (addr[1])
          {
            case 0x05: //Environment status
              if (data[0] == 0x00 && data[1] == 0xFF && data[2] == 0xFF)
                Serial.println("Unoccupied ");
              else if (data[0] == 0x01 && data[1] == 0x00 && data[2] == 0xFF)
                Serial.println("Someone is stationary");
              else if (data[0] == 0x01 && data[1] == 0x01 && data[2] == 0x01)
                Serial.println("Some people exercise");
              break; // ENDE 0x05: //Environment status
            case 0x06: //Signs parameters
              break; // ENDE 0x06: //Signs parameters
          }
          break; // ENDE 0x03: //Report radar information
        case 0x04: //Reporting system information
          switch (addr[1])
          {
            case 0x0C: //Threshold gear
              Serial.print("Empfindlichkeit: ");
              Serial.println(data[0]);
              break; // ENDE 0x0C: //Threshold gear
            case 0x10: //Scene setting
              Serial.print("Mode: ");
              switch (data[0])
              {
                case 0x00: //Default mode
                  Serial.println("Default mode");
                  break;
                case 0x01: //Area detection (top loading)
                  Serial.println("Area detection (top loading)");
                  break;
                case 0x02: //Bathroom (top mounted)
                  Serial.println("Bathroom (top mounted)");
                  break;
                case 0x03: //Bedroom (top loading)
                  Serial.println("Bedroom (top loading)");
                  break;
                case 0x04: //Living room (top mounted)
                  Serial.println("Living room (top mounted)");
                  break;
                case 0x05: //Office (top loading)
                  Serial.println("Office (top loading)");
                  break;
                case 0x06: //Hotel (top loading)
                  Serial.println("Hotel (top loading)");
                  break;
              }
              break; // ENDE 0x10: //Scene setting
          }
          break; // ENDE 0x04: //Reporting system information
        case 0x05: //Report additional information
          switch (addr[1])
          {
            case 0x08: //Feedback OTA Upgrade Start
              Serial.print("OTA Upgrade: ");
              if (data[0] == 0)
                Serial.println("Failue");
              else
                Serial.println("Success");
              break; // ENDE 0x08: //Feedback OTA Upgrade Start
            case 0x09: //Feedback OTA transmission
              break; // ENDE 0x09: //Feedback OTA transmission
              Serial.println(data[0], HEX);
            case 0x0B: //Fall function switch
              Serial.print("Fall function: ");
              if (data[0] == 0)
                Serial.println("OFF");
              else
                Serial.println("ON");
              break; // ENDE 0x0B: //Fall function switch
            case 0x0C: //Fall alarm time
              break; // ENDE 0x0C: //Fall alarm time
            case 0x0E: //Response to fall sensitivity setting
              break; // ENDE 0x0E: //Response to fall sensitivity settingandre
          }
          break; // ENDE 0x05: //Report additional information
      }
      break; // ENDE 0x03 Report radar information
      break;
    case 0x04: // Proactive reporting of commands
      switch (addr[0])
      {
        case 0x01: // Reporting module identification

          break; // ENDE 0x01: // Reporting module identification
        case 0x03: //Report radar information
          switch (addr[1])
          {
            case 0x05: //Environment status
              if (data[0] == 0x00 && data[1] == 0xFF && data[2] == 0xFF)
                Serial.println("Unoccupied ");
              else if (data[0] == 0x01 && data[1] == 0x00 && data[2] == 0xFF)
                Serial.println("Someone is stationary");
              else if (data[0] == 0x01 && data[1] == 0x01 && data[2] == 0x01)
                Serial.println("Some people exercise");
              break; //ENDE 0x05 Environment status
            case 0x07: //Approaching away state
              if (data[0] == 0x01 && data[1] == 0x01 && data[2] == 0x01)
                Serial.println("Fixed character None");
              else if (data[0] == 0x01 && data[1] == 0x01 && data[2] == 0x02)
                Serial.println("Fixed character Close");
              else if (data[0] == 0x01 && data[1] == 0x01 && data[2] == 0x03)
                Serial.println("Fixed character Stay away");
              break; // ENDE 0x07 Approaching away state
          }
          break; // ENDE 0x03 Report radar information
        case 0x05: //Report other information
          switch (addr[1])
          {
            case 0x01: //Heartbeat Pack
              if (data[0] == 0x00 && data[1] == 0xFF && data[2] == 0xFF)
                Serial.println("Unoccupied H");
              else if (data[0] == 0x01 && data[1] == 0x00 && data[2] == 0xFF)
                Serial.println("Someone is stationary H");
              else if (data[0] == 0x01 && data[1] == 0x01 && data[2] == 0x01)
                Serial.println("Some people exercise H");
              break; //ENDE 0x01 Heartbeat Pack
          }
          break; //ENDE 0x05 Report other information
      }
      break; // ENDE CMD 0x04  Proactive reporting of commands
  }
}
