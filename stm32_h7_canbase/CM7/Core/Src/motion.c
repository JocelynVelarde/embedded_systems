#include "motion.h"
#include "esc.h"
#include "servo.h"
#include <math.h>
#include "main.h" /* for HAL_GetTick */

void Motion_Init(Motion_t *m)
{
    if (!m) return;
    m->throttle_percent = 0;
    m->steering_deg = 0;
    m->target_x = 0.0f;
    m->target_y = 0.0f;
    m->has_target_point = 0;
    m->target_yaw_deg = 0.0f;

    /* PID*/
    m->pid_kp = 2.0f;
    m->pid_ki = 0.01f;
    m->pid_kd = 0.1f;
    m->pid_integrator = 0.0f;
    m->pid_last_error = 0.0f;
    m->pid_last_tick_ms = HAL_GetTick();
    
    /* Ensure ESC is neutral and servo centered */
    stopCarEsc();
    Servo_SetAngleDegrees(90.0f); /* 90° = center */
}

void Motion_SetThrottlePercent(Motion_t *m, int8_t percent)
{
    if (!m) return;
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;
    if (m->throttle_percent == percent) return;
    m->throttle_percent = percent;
    movePercent(percent);
}

void Motion_MoveForward(Motion_t *m, int8_t percent)
{
    if (!m) return;
    if (percent < 0) percent = -percent;
    Motion_SetThrottlePercent(m, percent);
}

void Motion_MoveBackward(Motion_t *m, int8_t percent)
{
    if (!m) return;
    if (percent > 0) percent = -percent;
    Motion_SetThrottlePercent(m, percent);
}

void Motion_Stop(Motion_t *m)
{
    if (!m) return;
    if (m->throttle_percent == 0) return;
    m->throttle_percent = 0;
    stopCarEsc();
}

void Motion_SetSteeringDeg(Motion_t *m, int16_t deg)
{
    if (!m) return;
    if (deg > 90) deg = 90;
    if (deg < -90) deg = -90;
    if (m->steering_deg == deg) return;
    m->steering_deg = deg;
    float servo_deg = 90.0f + (float)deg;
    Servo_SetAngleDegrees(servo_deg);
}

void Motion_SetTargetPoint(Motion_t *m, float tx, float ty)
{
    if (!m) return;
    m->target_x = tx;
    m->target_y = ty;
    m->has_target_point = 1;
}

void Motion_ClearTargetPoint(Motion_t *m)
{
    if (!m) return;
    m->has_target_point = 0;
}

void Motion_SetTargetYaw(Motion_t *m, float yaw_deg)
{
    if (!m) return;
    m->target_yaw_deg = yaw_deg;
    m->has_target_point = 0;
}

void Motion_SetPIDParams(Motion_t *m, float kp, float ki, float kd)
{
    if (!m) return;
    m->pid_kp = kp;
    m->pid_ki = ki;
    m->pid_kd = kd;
}

static float wrap180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void Motion_Update(Motion_t *m, float current_x, float current_y, float current_yaw_deg)
{
    if (!m) return;

    float desired_yaw_deg = m->target_yaw_deg;
    if (m->has_target_point) {
        float dx = m->target_x - current_x;
        float dy = m->target_y - current_y;
        if (fabsf(dx) < 1e-6f && fabsf(dy) < 1e-6f) {
            desired_yaw_deg = current_yaw_deg;
        } else {
            desired_yaw_deg = atan2f(dy, dx) * (180.0f / M_PI);
        }
    }

    float error = wrap180(desired_yaw_deg - current_yaw_deg);

    uint32_t now = HAL_GetTick();
    float dt = (now - m->pid_last_tick_ms) * 0.001f; /* seconds */
    if (dt <= 0.0f) dt = 0.001f;

    m->pid_integrator += error * dt;
    if (m->pid_integrator > 100.0f) m->pid_integrator = 100.0f;
    if (m->pid_integrator < -100.0f) m->pid_integrator = -100.0f;

    float derivative = (error - m->pid_last_error) / dt;
    float out = m->pid_kp * error + m->pid_ki * m->pid_integrator + m->pid_kd * derivative;

    if (out > 90.0f) out = 90.0f;
    if (out < -90.0f) out = -90.0f;

    Motion_SetSteeringDeg(m, (int16_t)out);

    m->pid_last_error = error;
    m->pid_last_tick_ms = now;
}
