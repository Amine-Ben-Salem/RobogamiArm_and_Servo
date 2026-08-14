#ifndef MAIN_BASE_H
#define MAIN_BASE_H
#include <DynamixelShield.h>
//#include "Timing.cpp"

//-----DYNAMIXEL--------

#ifndef DXL_SHIELD
#define DXL_SHIELD
DynamixelShield dxl;
#endif

using namespace ControlTableItem;   // Enables GOAL_POSITION, PRESENT_POSITION, etc...

const uint8_t DXL_ID_base = 41; //   
// const float DXL_PROTOCOL_VERSION_base = 2.0;

// 180° Rotation range
double min_position_deg_base = 0;
double max_position_deg_base = 180;

int32_t min_position_base = 0;
int32_t max_position_base = 4095;

// --------------

#ifndef GETVALUE
#define GETVALUE
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
#endif

void init_base();
void loop_base();
#endif