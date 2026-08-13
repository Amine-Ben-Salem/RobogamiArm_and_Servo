#include <DynamixelShield.h>
//#include "Timing.cpp"

//-----DYNAMIXEL--------

DynamixelShield dxl;

using namespace ControlTableItem;   // Enables GOAL_POSITION, PRESENT_POSITION, etc...

const uint8_t DXL_ID = 41; //   
const float DXL_PROTOCOL_VERSION = 2.0;

// 180° Rotation range
double min_position_deg = 0;
double max_position_deg = 180;

int32_t min_position = 0;
int32_t max_position = 4095;

//-----MESSAGE EXTRACTOR------

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


//--------NOT USING THIS NOW----------

// float sinWaveGenerator(double offset, double amplitude, double period, double currentTime){
//   return amplitude*sin(2*PI*currentTime/period) + offset;
// }

// String DataBLE = "";

// Timing control
// Timer Timer_01, Timer_02;

// int postionContolLoop = 20, printLoop = 100; // [ms]
// bool print = true;

// //Initialize module class
// //int max_motor_speed = 40, min_motor_speed = -40; // [0.024 rad/s] or [0.229 rotation/minute] // [raw]

// // Initialize
// float setpointAngle=min_position, setpointVelocity = 0; // for sinusoidal angle

// // --- POSITON CONTROL ---
// bool state = false;

// float BaseServoAngle={min_position}; // 
// float BaseServoVelocity={0}; 

// float poseUnit = 0.001534355; // [rad/RAW_DATA]

// float startTime, currentTime, previousTime;

