#ifndef MAIN_H
#define MAIN_H

String getValue(String data, char separator, int index);

#ifdef __cplusplus
// Shared state enum across modules
typedef enum {
	STATE_IDLE = -1,
	STATE_BASE = 0,
	STATE_ROBOGAMI = 1
} State;

extern State state;
#endif
#endif