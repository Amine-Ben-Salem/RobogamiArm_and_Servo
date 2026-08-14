#include <Arduino.h>
#include "main_base.h"

// ---------- INIT ----------

//SERIAL/BLUETOOTH
static float received_pos = 0;
static String inputString = "";
static String what_message = "";
static bool stringComplete = false;
static bool received_position=false;
static bool Current_pos=false;


//LED 
static const int ledPin = 13; //As a visualizer on Arduino due.

//MOTOR STATE
static bool motorArmed = false;// motion allowed only after safe init

void init_base() {
  //MOTOR INITIALISATION
  dxl.torqueOff(DXL_ID_base); // Disable torque (required for EEPROM writes)
  dxl.setOperatingMode(DXL_ID_base, OP_POSITION); // Configure the motor to obey POSITION commands only
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_base, 100);   // Change between 5 and 200
  dxl.writeControlTableItem(MIN_POSITION_LIMIT, DXL_ID_base, 60);
  dxl.torqueOn(DXL_ID_base); //

  motorArmed = true;

  Serial1.println("Base initialized safely");
  Serial1.println("Setting base initial position...");
  dxl.setGoalPosition(DXL_ID_base, 205, UNIT_DEGREE);
  Serial1.println("<READY: Base initialized>");
}

void loop_base() {
  
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
    if (received_pos > max_position_deg_base) received_pos = max_position_deg_base;
    if (received_pos < min_position_deg_base) received_pos = min_position_deg_base; 
    
    // move motor
    if (motorArmed) {
      // 205 value depends on how the base was mounted onto the servomotor
      dxl.setGoalPosition(DXL_ID_base, 205-received_pos, UNIT_DEGREE);
      Serial1.print("Setting goal position to: " + String(received_pos));
      Serial1.println('>');
    }
    received_position=false;
  }

  //READ POSITION
  if (Current_pos){
    Serial1.print("Current position: " + String(205-dxl.getPresentPosition(DXL_ID_base, UNIT_DEGREE)));
    Serial1.println('>');
    Current_pos=false;
  }
}
