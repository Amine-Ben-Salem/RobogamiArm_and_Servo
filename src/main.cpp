#include <Arduino.h>
#include "ServoBase/main_base.h"
#include "Robogami/main_robogami.h"

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
  Serial1.begin(115200);

  // This while basically waits for the py script to get started
  while (!py_script_running)
  {
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
      if (Serial1.available())
      {
          String msg = Serial1.readStringUntil('\n');
          msg.trim();

          if (msg == "START")
          {
              Serial1.println("Finishing the arduino setup...");
              py_script_running = true;
          }
          else
          {
              Serial1.println("<Unexpected:" + msg + ">");
          }
      }
      delay(500);
  }
  // Before running the python script make sure to put the shield's switch on DYNAMIXEL
  //DYNAMIXEL XM430
  dxl.begin(115200); 
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  init_base();
  init_robogami();
}

void loop() {
    if (Serial1.available() > 0) {
        String cmd = Serial1.readStringUntil('\n');

        static int state = -1; // 0 = base / 1 = robogami

        cmd.trim();

        if (cmd == "BASE") {
            state = 0;
            Serial1.println("Entering Base control mode...");
        } else if (cmd == "ROBOGAMI") {
            state = 1;
            Serial1.println("Entering Robogami control mode...");
        }
        switch (state) {
            case 0:
                loop_base();
                break;
            case 1:
                loop_robogami();
                break;
            default:
                Serial1.println("<Waiting for command: 'BASE' or 'ROBOGAMI'>");
        }
    }
}
