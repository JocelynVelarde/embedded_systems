#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include "main.h"

typedef struct {
    int8_t throttle_percent; /* -100..100 */
    int16_t steering_deg;    /* -90..90 */

    /* Targeting */
    float target_x;
    float target_y;
    uint8_t has_target_point;
    float target_yaw_deg; 

    /* PID */
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float pid_integrator;
    float pid_last_error;
    uint32_t pid_last_tick_ms;
} Motion_t;

void Motion_Init(Motion_t *m);

void Motion_SetThrottlePercent(Motion_t *m, int8_t percent);

void Motion_MoveForward(Motion_t *m, int8_t percent);
void Motion_MoveBackward(Motion_t *m, int8_t percent);

void Motion_Stop(Motion_t *m);
    
void Motion_SetSteeringDeg(Motion_t *m, int16_t deg);

void Motion_SetTargetPoint(Motion_t *m, float tx, float ty);
void Motion_ClearTargetPoint(Motion_t *m);
void Motion_SetTargetYaw(Motion_t *m, float yaw_deg);

void Motion_SetPIDParams(Motion_t *m, float kp, float ki, float kd);
void Motion_Update(Motion_t *m, float current_x, float current_y, float current_yaw_deg);

#endif /* MOTION_H */
