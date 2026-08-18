#include <Arduino.h>
#include "main_base.h"
#include "../main.h"


// ---------- FCT DECLARATIONS ----------

using namespace ControlTableItem;   // Enables GOAL_POSITION, PRESENT_POSITION, etc...

const uint8_t DXL_ID_base = 41; //   
// const float DXL_PROTOCOL_VERSION_base = 2.0;

// 180° Rotation range
double min_position_deg_base = 0;
double max_position_deg_base = 180;

int32_t min_position_base = 0;
int32_t max_position_base = 4095;

// --------------


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

void init_base(DynamixelShield &dxl, Stream &serial) {
  //MOTOR INITIALISATION
  dxl.torqueOff(DXL_ID_base); // Disable torque (required for EEPROM writes)
  dxl.setOperatingMode(DXL_ID_base, OP_POSITION); // Configure the motor to obey POSITION commands only
  dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID_base, 100);   // Change between 5 and 200
  dxl.writeControlTableItem(MIN_POSITION_LIMIT, DXL_ID_base, 60);
  dxl.torqueOn(DXL_ID_base); //

  motorArmed = true;

  serial.println("Setting base initial position...");
  dxl.setGoalPosition(DXL_ID_base, 205, UNIT_DEGREE);
  serial.println("<READY: Base initialized>");
}

void loop_base(DynamixelShield &dxl, Stream &serial) {
  // Read one full line from Python

  while (serial.available()>0) {
    inputString = serial.readStringUntil('\n');
    inputString.trim();
    // serial.print("In while loop_base(), inputString: ");
    // serial.println(inputString);
  }

  if (inputString.endsWith("ROBOGAMI")) {
    state = STATE_ROBOGAMI;
    stringComplete = false;
    inputString = "";
    serial.println("Entering Robogami control mode...");
    return;
  } else if (inputString.endsWith("BASE")) {
    state = STATE_BASE;
    stringComplete = false;
    inputString = "";
    serial.println("Entering Base control mode...");
    return;
  } else if (inputString.endsWith("DONEbase")) {
    // Remove the DONE suffix before parsing
    inputString.remove(inputString.length() - 8);
    stringComplete = true;
  }

  if (!stringComplete && inputString != "") {
    serial.print("Unknown message (main_base.cpp): ");
    serial.println(inputString);
  }

  if (stringComplete) {
    //----------- EXTRACT MESSAGE ------------
    what_message = getValue(inputString, ' ', 0); //extract first string
    serial.print('<');

    if (what_message == "GoalPos") {
      received_pos = getValue(inputString, ' ', 1).toFloat(); //extract value and convert to float
      received_position=true;
    }
    else if (what_message == "PresentPos"){
      Current_pos=true;
    } else {
        serial.print("Unknown command: ");
        serial.print(inputString);
        serial.println(">");
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
      serial.print("Setting goal position to: " + String(received_pos));
      serial.println('>');
    }
    received_position=false;
  }

  //READ POSITION
  if (Current_pos){
    serial.print("Current position: " + String(205-dxl.getPresentPosition(DXL_ID_base, UNIT_DEGREE)));
    serial.println('>');
    Current_pos=false;
  }
}