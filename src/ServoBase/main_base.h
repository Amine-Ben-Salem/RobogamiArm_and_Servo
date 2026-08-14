#ifndef MAIN_BASE_H
#define MAIN_BASE_H

#include <DynamixelShield.h>

void init_base(DynamixelShield &dxl, Stream &serial);
void loop_base(DynamixelShield &dxl, Stream &serial);

#endif