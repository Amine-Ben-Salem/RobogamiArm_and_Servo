#include "main_robogami.h"
#include <Arduino.h>
#include "kinematics.h"
#include "origami_module.h"
#include "Timing.cpp"

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

// --- DEFINE MODULE VARIABLES ---
BLA::Matrix<3> ee_velocities = {0.0, 0.0, 0.0};
BLA::Matrix<6> joint_angles_desired = {min_position, min_position, min_position, min_position, min_position, min_position};

// --- DEFINE SERIAL VARIABLES --
static double data_0, data_1, data_2, data_3, data_4, data_5;

static String inputString = "";         // a String to hold incoming data
static bool stringComplete = false;  // whether the string is complete



void printDxlPingCheck(DynamixelShield &dxl, Stream &DEBUG_SERIAL) {
  DEBUG_SERIAL.println("Dynamixel ping check:");
  for(int i = 0; i < DXL_ID_CNT; i++) {
    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(DXL_ID[i]);
    DEBUG_SERIAL.print(": ");
    DEBUG_SERIAL.println(dxl.ping(DXL_ID[i]) ? "OK" : "FAIL");
  }
}

void init_robogami(DynamixelShield &dxl, Stream &DEBUG_SERIAL) {
  // Turn off torque when configuring items in EEPROM area
  for(int i = 0; i < DXL_ID_CNT; i++) {
    dxl.torqueOff(DXL_ID[i]);
  }

  // --- SYNC READ POSITION ---
  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf         = user_pkt_buf;
  sr_infos.packet.buf_capacity  = user_pkt_buf_cap;
  sr_infos.packet.is_completed  = false;
  sr_infos.addr                 = SR_START_ADDR;
  sr_infos.addr_length          = SR_ADDR_LEN;
  sr_infos.p_xels               = info_xels_sr;
  sr_infos.xel_count            = 0;

  // Module
  for(int i = 0; i < DXL_ID_CNT; i++) {
    info_xels_sr[i].id          = DXL_ID[i];
    info_xels_sr[i].p_recv_buf  = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }

  sr_infos.is_info_changed    = true;

  // --- SYNC WRITE VELOCITY ---
  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf         = nullptr;
  sw_infos.packet.is_completed  = false;
  sw_infos.addr                 = SW_START_ADDR;
  sw_infos.addr_length          = SW_ADDR_LEN;
  sw_infos.p_xels               = info_xels_sw;
  sw_infos.xel_count            = 0;

  // Module
  for(int i = 0; i < DXL_ID_CNT; i++) {
    sw_data[i].goal_velocity  = 0;  // sw_data[0].goal_velocity = 0;
    info_xels_sw[i].id        = DXL_ID[i];
    info_xels_sw[i].p_data    = (uint8_t*)&sw_data[i].goal_velocity;
    sw_infos.xel_count++;
  }
  
  sw_infos.is_info_changed    = true;

  // --- MOTOR SETTINGS ---
  // Module
  for(int i = 0; i < DXL_ID_CNT; i++) {
    // to reduce the dead zone
    dxl.writeControlTableItem(MOVING_THRESHOLD, DXL_ID[i], 2); // 0.229 rpm	0 ~ 1,023
    dxl.writeControlTableItem(VELOCITY_LIMIT, DXL_ID[i], max_motor_speed); // 0.229 rpm	0 ~ 1,023
    //dxl.writeControlTableItem(CURRENT_LIMIT, DXL_ID[i], 1450);
    //dxl.writeControlTableItem(SHUTDOWN, DXL_ID[i], 00000101); // turn of overload shutdown
    dxl.setOperatingMode(DXL_ID[i], OP_VELOCITY);
  }

  // Module 1 & 2 - Initialize the positions and velocities
  for(int i = 0; i < 3; i++) { // 3 is number of legs per module
    module1.setpointLegAngle[i]         = min_position;
    module1.setpointLegAngleVelocity[i] = 0;
    module2.setpointLegAngle[i]         = min_position;
    module2.setpointLegAngleVelocity[i] = 0;
  }

  // --- SETUP PID ---
  module1.setupPIDs();
  module2.setupPIDs();

  // Position control  PID
  float kPPosBase     = 1200, kIPosBase      = 0, kDPosBase    = 0;  // double kPPosBase = 450, kIPosBase = 350/450, kDPosBase = 3/2;
  float kPVelBase     = 400, kIVelBase      = 600;  // double kPVelBase = 100, kIVelBase = 100;
  //float kPVelGripper  = 100, kIVelGripper   = 100; // double kPVelGripper = 100, kIVelGripper = 100;

  
  module1.position1PID->SetTunings(kPPosBase, kIPosBase, kDPosBase); // p, i, d
  module1.position2PID->SetTunings(kPPosBase, kIPosBase, kDPosBase);
  module1.position3PID->SetTunings(kPPosBase, kIPosBase, kDPosBase);
  module2.position1PID->SetTunings(kPPosBase, kIPosBase, kDPosBase); 
  module2.position2PID->SetTunings(kPPosBase, kIPosBase, kDPosBase);
  module2.position3PID->SetTunings(kPPosBase, kIPosBase, kDPosBase); 

  // --- TURN ON MOTOR TORQUE ---
  // Module
  for(int i = 0; i < DXL_ID_CNT; i++) {
    dxl.torqueOn(DXL_ID[i]);
  }

  // --- WRTIE PID GAINS ---
  // Module
  dxl.writeControlTableItem(VELOCITY_P_GAIN, module1.motorID1, kPVelBase);  dxl.writeControlTableItem(VELOCITY_I_GAIN, module1.motorID1, kIVelBase);
  dxl.writeControlTableItem(VELOCITY_P_GAIN, module1.motorID2, kPVelBase);  dxl.writeControlTableItem(VELOCITY_I_GAIN, module1.motorID2, kIVelBase);
  dxl.writeControlTableItem(VELOCITY_P_GAIN, module1.motorID3, kPVelBase);  dxl.writeControlTableItem(VELOCITY_I_GAIN, module1.motorID3, kIVelBase);
  dxl.writeControlTableItem(VELOCITY_P_GAIN, module2.motorID1, kPVelBase);  dxl.writeControlTableItem(VELOCITY_I_GAIN, module2.motorID1, kIVelBase);
  dxl.writeControlTableItem(VELOCITY_P_GAIN, module2.motorID2, kPVelBase);  dxl.writeControlTableItem(VELOCITY_I_GAIN, module2.motorID2, kIVelBase);
  dxl.writeControlTableItem(VELOCITY_P_GAIN, module2.motorID3, kPVelBase);  dxl.writeControlTableItem(VELOCITY_I_GAIN, module2.motorID3, kIVelBase);
 
  Timer_01.initialize(postionContolLoop);
  Timer_02.initialize(printLoop);

  startTime = (float)millis(); // [ms]

  DEBUG_SERIAL.println("<READY: Robogami initialized>");

  // Print this only after setup has completed, so the USB serial monitor has
  // time to reconnect after an upload and the result is not missed.
  //delay(5000);
  //printDxlPingCheck(dxl, DEBUG_SERIAL);
  //delay(5000);
}

void loop_robogami(DynamixelShield &dxl, Stream &DEBUG_SERIAL) {
  // --- POSITION CONTROL LOOP ---
  if(Timer_01.Tick()){
    currentTime = (float)millis() - startTime; // [ms]

    // --- SYNC READ DYNAMIXEL ---
    uint8_t recv_cnt;
    recv_cnt = dxl.syncRead(&sr_infos);

    // --- READ THE DYNAMIXEL DATA ---
    if(recv_cnt == DXL_ID_CNT_TOTAL){
      module1.assignLegAngles(poseUnit*sr_data[0].present_position,poseUnit*sr_data[1].present_position, poseUnit*sr_data[2].present_position);
      module2.assignLegAngles(poseUnit*sr_data[3].present_position,poseUnit*sr_data[4].present_position, poseUnit*sr_data[5].present_position);
    } else {
      DEBUG_SERIAL.print("syncRead failed: recv_cnt=");
      DEBUG_SERIAL.print(recv_cnt);
      DEBUG_SERIAL.print(" expected=");
      DEBUG_SERIAL.println(DXL_ID_CNT_TOTAL);
    }
   
    // --- SERIAL COMMUNICATION - VELOCITY CONTROL ---
    while (DEBUG_SERIAL.available()>0) {
      inputString = DEBUG_SERIAL.readStringUntil('\n');
      stringComplete = true;
    }

    if(stringComplete){
            data_0 = getValue(inputString, ' ', 0).toFloat();
            data_1  = getValue(inputString, ' ', 1).toFloat();
            data_2  = getValue(inputString, ' ', 2).toFloat();
            data_3  = getValue(inputString, ' ', 3).toFloat();
            data_4  = getValue(inputString, ' ', 4).toFloat();
            data_5  = getValue(inputString, ' ', 5).toFloat();

            // set joint angle positions
            joint_angles_desired(0) = data_0; // module 1 - leg 1
            joint_angles_desired(1) = data_1; // module 1 - leg 2
            joint_angles_desired(2) = data_2; // module 1 - leg 3
            joint_angles_desired(3) = data_3; // module 2 - leg 1
            joint_angles_desired(4) = data_4; // module 2 - leg 2
            joint_angles_desired(5) = data_5; // module 2 - leg 3

            inputString     = ""; //Reset string
            stringComplete  = false;
    }

    // --- COMPUTE POSITION CONTROLLER ---
    for(int i = 0; i < 3; i++) { // 3 is number of legs per module
      module1.setpointLegVelocityFF[i] = 0;
      module1.setpointLegVelocityFB[i] = 0; // [rad/s]
      module2.setpointLegVelocityFF[i] = 0;
      module2.setpointLegVelocityFB[i] = 0; // [rad/s]

      module1.setpointLegAngle[i] = joint_angles_desired(i);
      module2.setpointLegAngle[i] = joint_angles_desired(i+3); 
      
    }
    // RUN position pids ---
    module1.computePositionController(); // setpointLegVelocityFB is pid output, moduleLegAngle input, setpointLegAngle setpoint
    module2.computePositionController(); // setpointLegVelocityFB is pid output, moduleLegAngle input, setpointLegAngle setpoint
    
    // --- COMPUTE VELOCITY CONTROLLER ---
    for(int i = 0; i < 3; i++) {
      // Feedback and feedforward are combined
      module1.setpointLegAngleVelocity[i] = module1.setpointLegVelocityFB[i] + module1.setpointLegVelocityFF[i]/0.024; // [raw]
      module2.setpointLegAngleVelocity[i] = module2.setpointLegVelocityFB[i] + module2.setpointLegVelocityFF[i]/0.024; // [raw]
    }

    // MIN-MAX Velocity Check
    //ARM
    for(int i = 0; i < 3; i++) {
      if (module1.setpointLegAngleVelocity[i] > max_motor_speed) module1.setpointLegAngleVelocity[i]      = max_motor_speed; // [raw]
      else if (module1.setpointLegAngleVelocity[i] < min_motor_speed) module1.setpointLegAngleVelocity[i] = min_motor_speed; // [raw]
      if (module2.setpointLegAngleVelocity[i] > max_motor_speed) module2.setpointLegAngleVelocity[i]      = max_motor_speed; // [raw]
      else if (module2.setpointLegAngleVelocity[i] < min_motor_speed) module2.setpointLegAngleVelocity[i] = min_motor_speed; // [raw]
    }
  
    // MIN-MAX Position Check
    for(int i = 0; i < 3; i++) {
      // module 1
      if ((module1.moduleLegAngle[i] > max_position)&&(module1.setpointLegAngleVelocity[i]>0))
        module1.setpointLegAngleVelocity[i] = 0;
      else if((module1.moduleLegAngle[i] < min_position)&&(module1.setpointLegAngleVelocity[i]<0))
        module1.setpointLegAngleVelocity[i] = 0;
      
      // module 2
      if ((module2.moduleLegAngle[i] > max_position)&&(module2.setpointLegAngleVelocity[i]>0))
        module2.setpointLegAngleVelocity[i] = 0;
      else if((module2.moduleLegAngle[i] < min_position)&&(module2.setpointLegAngleVelocity[i]<0))
        module2.setpointLegAngleVelocity[i] = 0;

    }

    // --- SYNC WRITE MOTOR COMMANDS ---
    // Module - Insert a new Goal Velocity to the SyncWrite Packet
    for(int i = 0; i < 3; i++) {
      sw_data[i].goal_velocity  = module1.setpointLegAngleVelocity[i]; // [UNIT_RAW]
      sw_data[i+3].goal_velocity  = module2.setpointLegAngleVelocity[i]; // [UNIT_RAW]
    }

    // Update the SyncWrite packet status
    sw_infos.is_info_changed    = true;
    dxl.syncWrite(&sw_infos);
    }

    // --- SERIAL OUTPUT ---
    if(Timer_02.Tick()){
      if (print){
          DEBUG_SERIAL.print("<");
          DEBUG_SERIAL.print(module1.moduleLegAngle[0]); DEBUG_SERIAL.print(" x ");
          DEBUG_SERIAL.print(module1.moduleLegAngle[1]); DEBUG_SERIAL.print(" x ");
          DEBUG_SERIAL.print(module1.moduleLegAngle[2]); DEBUG_SERIAL.print(" x ");
          DEBUG_SERIAL.print(module2.moduleLegAngle[0]); DEBUG_SERIAL.print(" x ");
          DEBUG_SERIAL.print(module2.moduleLegAngle[1]); DEBUG_SERIAL.print(" x ");
          DEBUG_SERIAL.print(module2.moduleLegAngle[2]); DEBUG_SERIAL.print(" x ");
          DEBUG_SERIAL.print(0); DEBUG_SERIAL.print(" x ");
          DEBUG_SERIAL.println("0>");
    }
  }
}
