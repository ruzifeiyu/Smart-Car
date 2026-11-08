#include "Serial.h"
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;

#define SERIAL_TX_TIMEOUT_MS UINT32_C(500) /* 9600/8N1发送128字节约134ms。 */

/* DMA发送区独占完整缓存行，清理D-Cache时不会越界影响邻接变量。 */
#if defined(__ICCARM__)
#pragma data_alignment=32
static uint8_t Serial_TxBuffer[SERIAL_TX_BUFFER_SIZE];
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION) || defined(__GNUC__)
static uint8_t Serial_TxBuffer[SERIAL_TX_BUFFER_SIZE] __attribute__((aligned(32)));
#else
#error "Define 32-byte DMA buffer alignment for this compiler."
#endif

static volatile uint8_t tx_busy;
static volatile uint8_t rx_flag;
static volatile uint8_t rx_data;
static volatile uint8_t rx_retry;
static volatile HAL_StatusTypeDef last_status = HAL_OK;
static uint8_t rx_buffer;
static uint32_t tx_start_tick;
static uint8_t serial_ready;

HAL_StatusTypeDef Serial_Init(void)
{
    HAL_StatusTypeDef status;

    if ((huart3.Instance != USART3) || (huart3.hdmatx == NULL) ||
        (huart3.Init.BaudRate != 9600U) || (tx_busy != 0U))
    {
        last_status = HAL_ERROR;
        return HAL_ERROR;
    }

    rx_flag = 0U;
    rx_retry = 0U;
    serial_ready = 1U;
    status = HAL_UART_Receive_IT(&huart3, &rx_buffer, 1U);
    last_status = status;
    rx_retry = (status == HAL_OK) ? 0U : 1U;
    return status;
}

void Serial_Proc(void)
{
    HAL_StatusTypeDef status;

    if ((serial_ready == 0U) || (__get_IPSR() != 0U)) { return; }
    if ((tx_busy != 0U) &&
        ((uint32_t)(HAL_GetTick() - tx_start_tick) >= SERIAL_TX_TIMEOUT_MS))
    {
        /* 主循环中使用HAL自带有超时的终止操作，ISR不等待硬件。 */
        status = HAL_UART_AbortTransmit(&huart3);
        if (status == HAL_OK)
        {
            tx_busy = 0U;
            last_status = HAL_TIMEOUT;
        }
        else
        {
            last_status = status; /* 保留忙状态，不覆盖仍可能被DMA访问的缓冲区。 */
        }
    }

    if (rx_retry != 0U)
    {
        status = HAL_UART_Receive_IT(&huart3, &rx_buffer, 1U);
        last_status = status;
        if (status == HAL_OK) { rx_retry = 0U; }
    }
}

HAL_StatusTypeDef Serial_GetLastStatus(void)
{
    return last_status;
}

uint8_t Serial_SendBuffer(const uint8_t *data, uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((serial_ready == 0U) || (__get_IPSR() != 0U) || (data == NULL) ||
        (length == 0U) || (length > SERIAL_TX_BUFFER_SIZE))
    {
        last_status = HAL_ERROR;
        return 0U;
    }
    if (tx_busy != 0U)
    {
        last_status = HAL_BUSY;
        return 0U;
    }

    /* [WARNING]: STM32H743的DMA1不能读取DTCM。
       链接后须确认Serial_TxBuffer位于AXI/D2等DMA可达SRAM，不可位于0x20000000。 */
    memcpy(Serial_TxBuffer, data, length);
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
    {
        SCB_CleanDCache_by_Addr(Serial_TxBuffer, (int32_t)sizeof(Serial_TxBuffer));
    }
    __DSB();

    tx_start_tick = HAL_GetTick();
    tx_busy = 1U;
    status = HAL_UART_Transmit_DMA(&huart3, Serial_TxBuffer, length);
    last_status = status;
    if (status != HAL_OK)
    {
        tx_busy = 0U;
        return 0U;
    }
    return 1U;
}

uint8_t Serial_SendByte(uint8_t byte)
{
    return Serial_SendBuffer(&byte, 1U);
}

uint8_t Serial_SendArray(const uint8_t *array, uint16_t length)
{
    return Serial_SendBuffer(array, length);
}

uint8_t Serial_SendString(const char *string)
{
    uint16_t length = 0U;
    if (string == NULL) { last_status = HAL_ERROR; return 0U; }
    while ((length < SERIAL_TX_BUFFER_SIZE) && (string[length] != '\0'))
    {
        ++length;
    }
    if (length == SERIAL_TX_BUFFER_SIZE) { last_status = HAL_ERROR; return 0U; }
    return Serial_SendBuffer((const uint8_t *)string, length);
}

uint8_t Serial_Printf(const char *format, ...)
{
    char text[SERIAL_TX_BUFFER_SIZE];
    int length;
    va_list args;

    if ((format == NULL) || (__get_IPSR() != 0U))
    {
        last_status = HAL_ERROR;
        return 0U;
    }
    va_start(args, format);
    length = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if ((length <= 0) || ((size_t)length >= sizeof(text)))
    {
        last_status = HAL_ERROR;
        return 0U;
    }
    return Serial_SendBuffer((const uint8_t *)text, (uint16_t)length);
}

uint8_t Serial_ReadByte(uint8_t *byte)
{
    uint32_t primask;
    uint8_t available;
    if (byte == NULL) { return 0U; }
    primask = __get_PRIMASK();
    __disable_irq();
    available = rx_flag;
    if (available != 0U) { *byte = rx_data; rx_flag = 0U; }
    __set_PRIMASK(primask);
    return available;
}

uint8_t Serial_GetRxFlag(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t available;
    __disable_irq();
    available = rx_flag;
    rx_flag = 0U;
    __set_PRIMASK(primask);
    return available;
}

uint8_t Serial_GetRxData(void)
{
    return rx_data;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if ((uart != NULL) && (uart->Instance == USART3)) { tx_busy = 0U; }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    HAL_StatusTypeDef status;
    if ((uart == NULL) || (uart->Instance != USART3)) { return; }
    rx_data = rx_buffer;
    rx_flag = 1U;
    status = HAL_UART_Receive_IT(&huart3, &rx_buffer, 1U);
    last_status = status;
    rx_retry = (status == HAL_OK) ? 0U : 1U;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if ((uart == NULL) || (uart->Instance != USART3)) { return; }
    last_status = HAL_ERROR;
    if (uart->RxState != HAL_UART_STATE_BUSY_RX) { rx_retry = 1U; }
    /* DMA或UART硬件等待的恢复交给Serial_Proc，回调保持简短。 */
}
