#ifndef SMART_CAR_PID_H
#define SMART_CAR_PID_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float SP;
    uint32_t t_k_1;       /* HAL毫秒计数；无符号相减允许时基回绕。 */
    float err_k_1;
    float err_int_k_1;
    float Upperlimit;
    float Lowerlimit;
    float last_output;
    uint8_t has_previous;
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd);
void PID_ChangeSP(PID_TypeDef *pid, float setpoint);
float PID_Compute(PID_TypeDef *pid, float feedback);
void PID_LimitConfig(PID_TypeDef *pid, float upper, float lower);
void PID_Reset(PID_TypeDef *pid);

#ifdef __cplusplus
}
#endif

#endif
