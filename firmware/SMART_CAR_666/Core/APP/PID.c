#include "PID.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

static float PID_Clamp(float value, float lower, float upper)
{
    if (value > upper) { return upper; }
    if (value < lower) { return lower; }
    return value;
}

void PID_Reset(PID_TypeDef *pid)
{
    if (pid == NULL) { return; }
    pid->t_k_1 = 0U;
    pid->err_k_1 = 0.0f;
    pid->err_int_k_1 = 0.0f;
    pid->last_output = 0.0f;
    pid->has_previous = 0U;
}

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd)
{
    if (pid == NULL) { return; }
    pid->Kp = isfinite(kp) ? kp : 0.0f;
    pid->Ki = isfinite(ki) ? ki : 0.0f;
    pid->Kd = isfinite(kd) ? kd : 0.0f;
    pid->SP = 0.0f;
    pid->Upperlimit = FLT_MAX;
    pid->Lowerlimit = -FLT_MAX;
    PID_Reset(pid);
}

void PID_LimitConfig(PID_TypeDef *pid, float upper, float lower)
{
    if (pid == NULL) { return; }
    if ((!isfinite(upper)) || (!isfinite(lower)) || (upper < lower))
    {
        pid->Upperlimit = 0.0f;
        pid->Lowerlimit = 0.0f;
        PID_Reset(pid);
        return;
    }
    pid->Upperlimit = upper;
    pid->Lowerlimit = lower;
}

void PID_ChangeSP(PID_TypeDef *pid, float setpoint)
{
    if (pid == NULL) { return; }
    if (!isfinite(setpoint))
    {
        pid->SP = 0.0f;
        PID_Reset(pid);
        return;
    }
    pid->SP = setpoint;
}

float PID_Compute(PID_TypeDef *pid, float feedback)
{
    uint32_t tick;
    uint32_t elapsed_ms;
    float error;
    float dt;
    float derivative = 0.0f;
    float integral;
    float output;

    if (pid == NULL) { return 0.0f; }
    if ((!isfinite(feedback)) || (!isfinite(pid->SP)))
    {
        PID_Reset(pid);
        return 0.0f;
    }

    error = pid->SP - feedback;
    tick = HAL_GetTick();
    elapsed_ms = tick - pid->t_k_1;
    integral = pid->err_int_k_1;
    if (pid->has_previous != 0U)
    {
        if (elapsed_ms == 0U) { return pid->last_output; }
        dt = (float)elapsed_ms * 0.001f;
        derivative = (error - pid->err_k_1) / dt;
        integral += (error + pid->err_k_1) * dt * 0.5f;
    }

    /* 沿用参考工程的积分限幅数值；PI参数和限幅尚需按实际电机整定。 */
    integral = PID_Clamp(integral, pid->Lowerlimit, pid->Upperlimit);
    output = pid->Kp * error + pid->Ki * integral + pid->Kd * derivative;
    if (!isfinite(output))
    {
        PID_Reset(pid);
        return 0.0f;
    }

    pid->t_k_1 = tick;
    pid->err_k_1 = error;
    pid->err_int_k_1 = integral;
    pid->has_previous = 1U;
    pid->last_output = PID_Clamp(output, pid->Lowerlimit, pid->Upperlimit);
    return pid->last_output;
}
