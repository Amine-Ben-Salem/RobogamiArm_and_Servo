#ifndef MAIN_ROBOGAMI_H
#define MAIN_ROBOGAMI_H

#include <DynamixelShield.h>

void init_robogami(DynamixelShield &dxl, Stream &DEBUG_SERIAL);
void loop_robogami(DynamixelShield &dxl, Stream &DEBUG_SERIAL);

#endif