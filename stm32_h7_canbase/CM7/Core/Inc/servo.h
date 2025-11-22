#ifndef SERVO_H
#define SERVO_H

#include "main.h"

void Servo_SetPulse_us(uint16_t pulse_us);

void Servo_SetAngleDegrees(float degrees);

#ifndef SERVO_MOVE_MIN_MS
#define SERVO_MOVE_MIN_MS 100U
#endif

void Servo_MoveToDegrees(float degrees);

void Servo_MoveToDegreesForce(float degrees);

#endif
