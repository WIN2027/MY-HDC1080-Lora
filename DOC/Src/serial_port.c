/**
 ******************************************************************************
 * @file    serial_port.c
 * @brief   串口模块（serial_module）挂载：端口表 + HAL 回调转发
 *
 * @details 硬件与配置（依据 2.4G.ioc / CubeMX 生成的 Core 代码）：
 *            USART1 : PB6 = TX, PB7 = RX, 115200 8N1
 *            DMA    : USART1_RX = DMA2_Stream2 (CH4), USART1_TX = DMA2_Stream7 (CH4)
 *                     （Normal 模式、Byte/Byte、内存递增）
 *            NVIC   : USART1_IRQn / DMA2_Stream2_IRQn / DMA2_Stream7_IRQn 均使能，
 *                     优先级 5（= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY）
 *
 * @note  为什么必须打开 USART1 全局中断（HAL 源码依据，本工程 Drivers 版本）：
 *        1) 空闲收帧：HAL_UARTEx_ReceiveToIdle_DMA() 只置外设级 CR1.IDLEIE
 *           （stm32f4xx_hal_uart.c:1803），IDLE 事件在 HAL_UART_IRQHandler()
 *           中处理；NVIC 未使能则短帧永不回调，只有 RX DMA 缓冲被填满
 *           128 字节才走 DMA 完成回调。
 *        2) 发送完成：UART_DMATransmitCplt() 只置 CR1.TCIE
 *           （stm32f4xx_hal_uart.c:3028），HAL_UART_TxCpltCallback() 在
 *           HAL_UART_IRQHandler() 内调用（同文件 3586-3616 行）；NVIC 未使能
 *           则回调不来，huart1.gState 停在 BUSY_TX，后续发送全部 HAL_BUSY。
 ******************************************************************************
 */
#include "serial_port.h"

#include <stddef.h>

#include "serial_module.h"
#include "usart.h"

/*============================================================================
 * 端口表：应用层提供，生命周期覆盖整个运行期（模块内部只保存指针）
 *==========================================================================*/
static const SerialModule_PortCfg_t s_ports[] = {
    {
        .huart    = &huart1,      /* CubeMX 生成的 USART1 句柄（含 DMA 链接） */
        .name     = "dbg",        /* 仅调试标识 */
        .frame_cb = NULL,         /* 原始帧旁路钩子：本工程不需要 */
    },
};

/**
 * @brief 初始化串口模块（创建调度线程 + 启动各端口 DMA 空闲接收）
 */
bool SerialPort_Init(void)
{
    uint8_t port_count = (uint8_t)(sizeof(s_ports) / sizeof(s_ports[0]));

    return SerialModule_Init(s_ports, port_count);
}

/*============================================================================
 * HAL 回调 → 模块转发（全工程唯一实现点，勿在别处重复定义）
 *==========================================================================*/

/**
 * @brief 接收事件（IDLE 空闲中断 / DMA 半满 / DMA 完成）
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    SerialModule_OnRxEvent(huart, Size);
}

/**
 * @brief 发送完成（DMA 发完 + TC 中断）
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    SerialModule_OnTxCplt(huart);
}

/**
 * @brief 串口错误（溢出/噪声/帧错误等）—— 模块内部会重启接收通道
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    SerialModule_OnError(huart);
}

/* 文件结束 */
