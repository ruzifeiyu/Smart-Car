#ifndef SMART_CAR_SERIAL_H
#define SMART_CAR_SERIAL_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERIAL_TX_BUFFER_SIZE UINT16_C(128)

HAL_StatusTypeDef Serial_Init(void); /* 仅启动已配置USART3的单字节中断接收。 */
void Serial_Proc(void);             /* 主循环调用：发送超时处理、接收失败重试。 */
HAL_StatusTypeDef Serial_GetLastStatus(void);

/* 以下发送函数：1=已提交DMA，0=忙/参数错误；只允许主循环调用，不排队。 */
uint8_t Serial_SendBuffer(const uint8_t *data, uint16_t length);
uint8_t Serial_SendByte(uint8_t byte);
uint8_t Serial_SendArray(const uint8_t *array, uint16_t length);
uint8_t Serial_SendString(const char *string);
uint8_t Serial_Printf(const char *format, ...);

/* 单字节邮箱；高频连续接收应另外设计接收缓冲，未消费的旧字节会被覆盖。 */
uint8_t Serial_ReadByte(uint8_t *byte);
uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

#ifdef __cplusplus
}
#endif

#endif
