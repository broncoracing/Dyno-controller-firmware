#ifndef STEPPER_H
#define STEPPER_H
#include "main.h"

#define MAX_SPEED 2000 // Steps/(65536usec)
#define MIN_SPEED 50 // Steps/(65536usec)
#define ACCEL 7 // Steps/(65536usec)/msec
#define DECEL 400 // Steps/(65536usec)/256 steps
// TODO Deceleration?

#define MAX_POSITION 12000

void stepper_pulse_finished(void);

int32_t get_pos(void);
void move_to(int32_t target);

void zero(void);

uint8_t is_moving(void);

#endif