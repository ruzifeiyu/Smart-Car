#ifndef SMART_CAR_MOTOR_H
#define SMART_CAR_MOTOR_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_MAX_DUTY_PERCENT INT32_C(99)

/* CubeMX先配置TIM8、TIM1、TIM14和PB1；本函数启动七个PWM通道。 */
HAL_StatusTypeDef Motor_Init(void);

/* 单位：PWM百分比，不是电机转速。
   电机1：PC8/TIM8_CH3、PC9/TIM8_CH4，范围-99~99。
   电机2：PA10/TIM1_CH3、PA11/TIM1_CH4，范围-99~99。
   电机3：PA8/TIM1_CH1、PA9/TIM1_CH2，范围-99~99。
   前三路正数选前一个引脚输出，负数选后一个；另一输入为0。
   电机4：PA7/TIM14_CH1输出，PB1固定低电平，范围0~99，负数按0。
   所有参数越界限幅；0清零对应电机的PWM，机械方向取决于接线。 */
HAL_StatusTypeDef Load(int32_t motor1, int32_t motor2, int32_t motor3, int32_t motor4);
void Motor_Stop(void);

/* 返回CCR表示的占空比幅值，不含方向；实际引脚最多滞后一个PWM周期。 */
uint32_t Motor_GetDuty1Percent(void);
uint32_t Motor_GetDuty2Percent(void);
uint32_t Motor_GetDuty3Percent(void);
uint32_t Motor_GetDuty4Percent(void);

#ifdef __cplusplus
}
#endif

#endif
