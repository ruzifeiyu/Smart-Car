#include "Motor.h"

extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim14;

static uint8_t motor_ready;
static uint32_t tim8_period_counts;
static uint32_t tim1_period_counts;
static uint32_t tim14_period_counts;

static void Motor_WritePair(TIM_HandleTypeDef *timer, uint32_t channel_a, uint32_t channel_b,
                            uint32_t compare_a, uint32_t compare_b)
{
    uint32_t update_disabled = READ_BIT(timer->Instance->CR1, TIM_CR1_UDIS);

    /* HAL已开启CCR预装载；禁止更新期间一起写入同一电机的两个通道，
       避免写入跨过周期边界时，一路先更新造成暂时双路非零。 */
    SET_BIT(timer->Instance->CR1, TIM_CR1_UDIS);
    __HAL_TIM_SET_COMPARE(timer, channel_a, compare_a);
    __HAL_TIM_SET_COMPARE(timer, channel_b, compare_b);
    if (update_disabled == 0U)
    {
        CLEAR_BIT(timer->Instance->CR1, TIM_CR1_UDIS);
    }
}

static uint32_t Motor_CommandToCompare(int32_t command, uint32_t period_counts)
{
    uint32_t magnitude;
    uint32_t compare;

    /* 先限幅再取负，避免INT32_MIN取绝对值溢出；99%沿用参考工程。 */
    if (command > MOTOR_MAX_DUTY_PERCENT) { command = MOTOR_MAX_DUTY_PERCENT; }
    if (command < -MOTOR_MAX_DUTY_PERCENT) { command = -MOTOR_MAX_DUTY_PERCENT; }
    magnitude = (uint32_t)((command < 0) ? -command : command);
    compare = (magnitude * period_counts + 50U) / 100U;
    /* 不因整数四舍五入变成100%常高；实际最大可用占空比需板级验证。 */
    if (compare >= period_counts) { compare = period_counts - 1U; }
    return compare;
}

static void Motor_SetPair(TIM_HandleTypeDef *timer, uint32_t channel_a, uint32_t channel_b,
                          int32_t command, uint32_t period_counts)
{
    uint32_t compare = Motor_CommandToCompare(command, period_counts);

    if (command >= 0)
    {
        Motor_WritePair(timer, channel_a, channel_b, compare, 0U);
    }
    else
    {
        Motor_WritePair(timer, channel_a, channel_b, 0U, compare);
    }
}

HAL_StatusTypeDef Motor_Init(void)
{
    motor_ready = 0U;
    tim8_period_counts = 0U;
    tim1_period_counts = 0U;
    tim14_period_counts = 0U;
    Motor_Stop();

    if ((htim8.Instance != TIM8) || (htim1.Instance != TIM1) || (htim14.Instance != TIM14) ||
        (htim8.Init.Period == 0U) || (htim8.Init.Period >= UINT16_MAX) ||
        (htim1.Init.Period == 0U) || (htim1.Init.Period >= UINT16_MAX) ||
        (htim14.Init.Period == 0U) || (htim14.Init.Period >= UINT16_MAX) ||
        (htim8.Init.CounterMode != TIM_COUNTERMODE_UP) ||
        (htim1.Init.CounterMode != TIM_COUNTERMODE_UP) ||
        (htim14.Init.CounterMode != TIM_COUNTERMODE_UP))
    {
        return HAL_ERROR;
    }
    tim8_period_counts = htim8.Init.Period + 1U;
    tim1_period_counts = htim1.Init.Period + 1U;
    tim14_period_counts = htim14.Init.Period + 1U;

    /* 启动前让所有零比较值生效；TIM3只用于编码器，不在这里操作。 */
    htim8.Instance->EGR = TIM_EGR_UG;
    htim1.Instance->EGR = TIM_EGR_UG;
    htim14.Instance->EGR = TIM_EGR_UG;
    if ((HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) != HAL_OK))
    {
        Motor_Stop();
        return HAL_ERROR;
    }

    motor_ready = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef Load(int32_t motor1, int32_t motor2, int32_t motor3, int32_t motor4)
{
    if (motor_ready == 0U) { return HAL_ERROR; }

    // [WARNING]: 前三路变号前先置0，待电机减速后换向；第四路固定方向，负命令按0处理。
    Motor_SetPair(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, motor1, tim8_period_counts);
    Motor_SetPair(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, motor2, tim1_period_counts);
    Motor_SetPair(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, motor3, tim1_period_counts);
    if (motor4 < 0) { motor4 = 0; }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, Motor_CommandToCompare(motor4, tim14_period_counts));
    return HAL_OK;
}

void Motor_Stop(void)
{
    if (htim8.Instance == TIM8)
    {
        Motor_WritePair(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, 0U, 0U);
    }
    if (htim1.Instance == TIM1)
    {
        Motor_WritePair(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, 0U, 0U);
        Motor_WritePair(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, 0U, 0U);
    }
    if (htim14.Instance == TIM14)
    {
        __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0U);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

static uint32_t Motor_GetPairDuty(TIM_HandleTypeDef *timer, uint32_t channel_a,
                                  uint32_t channel_b, uint32_t period_counts)
{
    uint32_t compare_a = __HAL_TIM_GET_COMPARE(timer, channel_a);
    uint32_t compare_b = __HAL_TIM_GET_COMPARE(timer, channel_b);
    uint32_t compare = (compare_a > compare_b) ? compare_a : compare_b;

    return (compare * 100U + period_counts / 2U) / period_counts;
}

uint32_t Motor_GetDuty1Percent(void)
{
    if (motor_ready == 0U) { return 0U; }
    return Motor_GetPairDuty(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, tim8_period_counts);
}

uint32_t Motor_GetDuty2Percent(void)
{
    if (motor_ready == 0U) { return 0U; }
    return Motor_GetPairDuty(&htim1, TIM_CHANNEL_3, TIM_CHANNEL_4, tim1_period_counts);
}

uint32_t Motor_GetDuty3Percent(void)
{
    if (motor_ready == 0U) { return 0U; }
    return Motor_GetPairDuty(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, tim1_period_counts);
}

uint32_t Motor_GetDuty4Percent(void)
{
    uint32_t compare;

    if (motor_ready == 0U) { return 0U; }
    compare = __HAL_TIM_GET_COMPARE(&htim14, TIM_CHANNEL_1);
    return (compare * 100U + tim14_period_counts / 2U) / tim14_period_counts;
}
