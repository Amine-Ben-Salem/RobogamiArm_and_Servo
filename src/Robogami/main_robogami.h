#ifndef MAIN_ROBOGAMI_H
#define MAIN_ROBOGAMI_H

#include <DynamixelShield.h>
#include "origami_module.h"
#include "Timing.cpp"

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

// Timing control
Timer Timer_01, Timer_02;

int postionContolLoop = 20, printLoop = 100; // [ms]
bool print = true;

//Initialize module class
//int max_motor_speed = 150, min_motor_speed = -150; // [0.024 rad/s] or [0.229 rotation/minute] // [raw]
int max_motor_speed = 100, min_motor_speed = -100; // [0.024 rad/s] or [0.229 rotation/minute] // [raw]
int max_gripper_speed = 40, min_gripper_speed = -40; // [0.024 rad/s] or [0.229 rotation/minute] // [raw]
int max_gripper_current = 120;

float min_position = 10*PI/180; // appx 0.26, before it was 5
float max_position = 70*PI/180; // appx 1.4 [rad], before it was 80

// Initialize
float setpointAngle=min_position, setpointVelocity = 0; // for sinusoidal angle


// --- POSITON CONTROL ---
bool state = false;

int robot_id = 1; // 1 or 2
int n_modules = 2;
const uint8_t DXL_ID_CNT = 6;
const uint8_t DXL_ID[DXL_ID_CNT] = {50,51,52,60,61,62};
const uint8_t DXL_ID_CNT_TOTAL = 6;
const uint8_t DXL_ID_GRIPPER = 80; 
float setAngle[DXL_ID_CNT]={min_position, min_position, min_position, min_position, min_position, min_position}; // 
float setVelocity[DXL_ID_CNT]={0,0,0,0,0,0}; // [rad/s]

float poseUnit = 0.001534355; // [rad/RAW_DATA]

Module module1(DXL_ID[0], DXL_ID[1], DXL_ID[2]);
Module module2(DXL_ID[3], DXL_ID[4], DXL_ID[5]);

float startTime, currentTime, previousTime;

const float DXL_PROTOCOL_VERSION = 2.0;

#ifndef DXL_SHIELD
#define DXL_SHIELD
DynamixelShield dxl; // Serial 
#endif

//This namespace is required to use Control table item names
using namespace ControlTableItem;



// ----- SYNC READ and WRITE  - (from example code)
const uint8_t BROADCAST_ID = 254;

const uint16_t user_pkt_buf_cap = 128; // why 128? maybe not enough for 6 motors?
uint8_t user_pkt_buf[user_pkt_buf_cap];

// --- POSITION DATA TO READ --- 
// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 4;

// --- VELOCITY DATA TO WRITE --- 
// Starting address of the Data to write; Goal Velocity = 104
const uint16_t SW_START_ADDR = 104;
// Length of the Data to write; Length of Goal Velocity data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;

typedef struct sr_data{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;

typedef struct sw_data{
  int32_t goal_velocity;
} __attribute__((packed)) sw_data_t;


sr_data_t sr_data[DXL_ID_CNT_TOTAL];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT_TOTAL];

sw_data_t sw_data[DXL_ID_CNT_TOTAL];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT_TOTAL];

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


float sinWaveGenerator(double offset, double amplitude, double period, double currentTime){
  return amplitude*sin(2*PI*currentTime/period) + offset;
}

void init_robogami();
void loop_robogami();

#endif