#include <Arduino.h>
#include "ServoBase/main_base.h"
#include "Robogami/main_robogami.h"
#include "main.h"

// ---------- INIT ----------

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
    if (DEBUG_SERIAL.available() > 0) {
        String cmd = DEBUG_SERIAL.readStringUntil('\n');

        static int state = -1; // 0 = base / 1 = robogami

        cmd.trim();

        if (cmd == "BASE") {
            DEBUG_SERIAL.println("Entering Base control mode...");
            state = 0;
        } else if (cmd == "ROBOGAMI") {
            DEBUG_SERIAL.println("Entering Robogami control mode...");
            state = 1;
        }
        switch (state) {
            case 0:
                loop_base(dxl, DEBUG_SERIAL);
                break;
            case 1:
                loop_robogami(dxl, DEBUG_SERIAL);
                break;
            default:
                DEBUG_SERIAL.print("<Waiting for command: 'BASE' or 'ROBOGAMI'>");
        }
    }
}
