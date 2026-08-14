#ifndef MAIN_H
#define MAIN_H

#include <DynamixelShield.h>

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


#endif