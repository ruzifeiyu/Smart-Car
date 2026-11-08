#include "Encoder.h"
#include <limits.h>
#include <math.h>

/* 句柄由CubeMX生成；此处只声明，不能再次定义或初始化TIM3。 */
extern TIM_HandleTypeDef htim3;

#define ENCODER_COUNTER_MASK UINT32_C(0xFFFF)
#define ENCODER_HALF_RANGE   UINT32_C(0x8000)
#define ENCODER_COUNTER_SIZE INT32_C(65536)
#define ENCODER_TWO_PI       6.283185307f

static uint32_t previous_count;
static uint32_t previous_tick;
static int32_t last_delta;
static int64_t total_count;
static float counts_one_rev;
static float speed_rad_s;
static float counts_per_second;
static uint8_t encoder_ready;
static uint8_t encoder_calibrated;

HAL_StatusTypeDef Encoder_Init(float counts_per_rev)
{
    HAL_StatusTypeDef status;

    encoder_ready = 0U;
    encoder_calibrated = 0U;
    speed_rad_s = 0.0f;
    counts_per_second = 0.0f;
    last_delta = 0;
    total_count = 0;

    if ((!isfinite(counts_per_rev)) || (counts_per_rev < 0.0f) ||
        ((counts_per_rev > 0.0f) && (counts_per_rev < 1.0f)) ||
        (htim3.Instance != TIM3) || (htim3.Init.Period != UINT32_C(65535)) ||
        (htim3.Init.Prescaler != 0U))
    {
        return HAL_ERROR;
    }

    counts_one_rev = counts_per_rev;
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    previous_count = 0U;
    previous_tick = HAL_GetTick();

    /* 仅启动CubeMX已经配置好的编码器接口，不使用EXTI或定时器中断。 */
    status = HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    if (status != HAL_OK)
    {
        return status;
    }

    encoder_ready = 1U;
    encoder_calibrated = (counts_per_rev >= 1.0f) ? 1U : 0U;
    return HAL_OK;
}

HAL_StatusTypeDef Encoder_Update(void)
{
    uint32_t now_tick;
    uint32_t elapsed_ms;
    uint32_t now_count;
    uint32_t raw_delta;
    int32_t delta;

    if (encoder_ready == 0U)
    {
        return HAL_ERROR;
    }

    now_tick = HAL_GetTick();
    elapsed_ms = now_tick - previous_tick;
    if (elapsed_ms == 0U)
    {
        return HAL_BUSY; /* 保留计数基准，避免同一毫秒重复更新造成除零或丢计数。 */
    }

    now_count = __HAL_TIM_GET_COUNTER(&htim3) & ENCODER_COUNTER_MASK;
    raw_delta = (now_count - previous_count) & ENCODER_COUNTER_MASK;
    previous_count = now_count;
    previous_tick = now_tick;

    /* [WARNING]: 两次采样间的位移计数绝对值必须小于32768，否则会混淆方向。
       应约每20ms采样；若高转速或齿轮比使计数超过此界限，改用32位定时器。
       恰好半圈回绕无法判向，返回错误并清零速度，调用者应停止电机。 */
    if (raw_delta == ENCODER_HALF_RANGE)
    {
        last_delta = 0;
        speed_rad_s = 0.0f;
        counts_per_second = 0.0f;
        return HAL_ERROR;
    }

    if (raw_delta < ENCODER_HALF_RANGE)
    {
        delta = (int32_t)raw_delta;
    }
    else
    {
        delta = (int32_t)raw_delta - ENCODER_COUNTER_SIZE;
    }

    if (((delta > 0) && (total_count > INT64_MAX - (int64_t)delta)) ||
        ((delta < 0) && (total_count < INT64_MIN - (int64_t)delta)))
    {
        encoder_ready = 0U;
        speed_rad_s = 0.0f;
        counts_per_second = 0.0f;
        last_delta = 0;
        return HAL_ERROR;
    }

    last_delta = delta;
    total_count += (int64_t)delta;
    /* 先按实际采样间隔换算计数/s；只有标定后才计算rad/s，避免虚构转速。 */
    counts_per_second = (float)delta * 1000.0f / (float)elapsed_ms;
    if (encoder_calibrated != 0U)
    {
        speed_rad_s = counts_per_second * ENCODER_TWO_PI / counts_one_rev;
    }
    else
    {
        speed_rad_s = 0.0f;
    }
    return HAL_OK;
}

uint8_t Encoder_IsReady(void)
{
    return encoder_ready;
}

uint8_t Encoder_IsCalibrated(void)
{
    return encoder_calibrated;
}

float Encoder_GetCountsPerSecond_L(void)
{
    return counts_per_second;
}

float Encoder_GetSpeed_L(void)
{
    return speed_rad_s;
}

float Encoder_GetPos_L(void)
{
    if ((encoder_ready == 0U) || (encoder_calibrated == 0U))
    {
        return 0.0f;
    }
    return (float)total_count * 360.0f / counts_one_rev;
}

int32_t Encoder_GetDelta_L(void)
{
    return last_delta;
}

int64_t Encoder_GetCount_L(void)
{
    return total_count;
}
