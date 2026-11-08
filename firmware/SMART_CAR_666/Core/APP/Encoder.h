#ifndef SMART_CAR_ENCODER_H
#define SMART_CAR_ENCODER_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CubeMX：TIM3，PB4/CH1、PB5/CH2，Encoder TI1 and TI2，PSC=0，ARR=65535。 */
/* counts_per_rev：四倍频后每转实际计数(>=1)；传0只统计原始计数，不换算角速度。 */
HAL_StatusTypeDef Encoder_Init(float counts_per_rev);
HAL_StatusTypeDef Encoder_Update(void);
uint8_t Encoder_IsReady(void);
uint8_t Encoder_IsCalibrated(void);
float Encoder_GetCountsPerSecond_L(void); /* 计数/s，不需要每转计数参数。 */
/* 以下两个接口未标定时返回0，调用前需检查Encoder_IsCalibrated。 */
float Encoder_GetSpeed_L(void);       /* rad/s */
float Encoder_GetPos_L(void);         /* degree */
int32_t Encoder_GetDelta_L(void);     /* 最近一次采样的有符号计数差 */
int64_t Encoder_GetCount_L(void);     /* 初始化以来的有符号累计计数 */

#ifdef __cplusplus
}
#endif

#endif
