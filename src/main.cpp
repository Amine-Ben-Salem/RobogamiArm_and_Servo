#include <Arduino.h>
#include "ServoBase/main_base.h"
#include "Robogami/main_robogami.h"
#include "main.h"

// ---------- INIT ----------
// Old code. Don't know what it does exactly, but doesn't matter for now.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL Serial1    // Hardware UART for the Bluetooth module
#else
  #define DEBUG_SERIAL Serial
#endif

DynamixelShield dxl; // Serial 
const float DXL_PROTOCOL_VERSION = 2.0;

// Shared state across modules
State state = STATE_IDLE;

// https://stackoverflow.com/questions/9072320/split-string-into-string-array
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//LED 
const int ledPin = 13; //As a visualizer on Arduino due.


void setup() {
  delay(100);
  bool py_script_running= false;

  //LED
  pinMode(ledPin, OUTPUT);
  bool ledState = false;

  //ARDUINO AND DYNAMIXEL SERIAL COMMUNICATION
  Serial.begin(115200);

  //BLUETOOTH HC-06 AND ARDUINO SERIAL COMMUNICATION
  DEBUG_SERIAL.begin(115200);

  // This while basically waits for the py script to get started
  while (!py_script_running)
  {
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
      if (DEBUG_SERIAL.available())
      {
          String msg = DEBUG_SERIAL.readStringUntil('\n');
          msg.trim();

          if (msg == "START")
          {
              DEBUG_SERIAL.println("Finishing the arduino setup...");
              py_script_running = true;
          }
          else
          {
              DEBUG_SERIAL.println("<Unexpected:" + msg + ">");
          }
      }
      delay(500);
  }
  // Before running the python script make sure to put the shield's switch on DYNAMIXEL
  //DYNAMIXEL XM430
  dxl.begin(115200); 
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  init_base(dxl, DEBUG_SERIAL);
  init_robogami(dxl, DEBUG_SERIAL);
}

void loop() {
    switch (state)
    {
    case STATE_BASE:
        loop_base(dxl, DEBUG_SERIAL);
        break;
    case STATE_ROBOGAMI:
        loop_robogami(dxl, DEBUG_SERIAL);
        break;
    default:
        if (DEBUG_SERIAL.available() > 0) {
            String cmd = DEBUG_SERIAL.readStringUntil('\n');
            if (cmd == "BASE") {
                DEBUG_SERIAL.println("Entering Base control mode...");
                state = STATE_BASE;
            } else if (cmd == "ROBOGAMI") {
                DEBUG_SERIAL.println("Entering Robogami control mode...");
                state = STATE_ROBOGAMI;
            }
        }
        break;
    }

    
    
}
