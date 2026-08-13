#include <Arduino.h>
#include "main_base.h"

// ---------- INIT ----------

//SERIAL/BLUETOOTH
float received_pos = 0;
String inputString = "";
String what_message = "";
bool stringComplete = false;
bool received_position=false;
bool Current_pos=false;


//LED 
const int ledPin = 13; //As a visualizer on Arduino due.

//MOTOR STATE
bool motorArmed = false;// motion allowed only after safe init

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
  //MOTOR INITIALISATION
  dxl.torqueOff(DXL_ID); // Disable torque (required for EEPROM writes)
  dxl.setOperatingMode(DXL_ID, OP_POSITION); // Configure the motor to obey POSITION commands only
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID, 100);   // Change between 5 and 200
  dxl.writeControlTableItem(MIN_POSITION_LIMIT, DXL_ID, 60);
  dxl.torqueOn(DXL_ID); // 

  motorArmed = true;

  Serial1.println("Dynamixel initialized safely");
  Serial1.println("Setting initial position...");
  dxl.setGoalPosition(DXL_ID, 205, UNIT_DEGREE);
  Serial1.println("<READY>");
}

void loop() {
  
  // ---------- SERIAL COMM WITH BLE ----------
  while (Serial1.available()>0) {
    inputString = Serial1.readStringUntil('\n');
    stringComplete = true;
  }

  //----------- EXTRACT MESSAGE ------------
  if(stringComplete){
    what_message = getValue(inputString, ' ', 0); //extract first string
    Serial1.print('<');

    if (what_message == "GoalPos") {
      received_pos = getValue(inputString, ' ', 1).toFloat(); //extract value and convert to float
      received_position=true;
    }
    else if (what_message == "PresentPos"){
      Current_pos=true;
    } else {
        Serial1.print("Unknown command: ");
        Serial1.print(inputString);
        Serial1.println(">");
    }
      inputString     = ""; //Reset string
      stringComplete  = false;
  }

  //----------EXECUTE COMMAND -FOR NOW, ONLY READ AND WRITE POSITION- -------

  //WRITE POSITION
  if (received_position){

    //clamp
    if (received_pos > max_position_deg) received_pos = max_position_deg;
    if (received_pos < min_position_deg) received_pos = min_position_deg; 
    
    // move motor
    if (motorArmed) {
      // 205 value depends on how the base was mounted onto the servomotor
      dxl.setGoalPosition(DXL_ID, 205-received_pos, UNIT_DEGREE);
      Serial1.print("Setting goal position to: " + String(received_pos));
      Serial1.println('>');
    }
    received_position=false;
  }

  //READ POSITION
  if (Current_pos){
    Serial1.print("Current position: " + String(205-dxl.getPresentPosition(DXL_ID, UNIT_DEGREE)));
    Serial1.println('>');
    Current_pos=false;
  }
}
