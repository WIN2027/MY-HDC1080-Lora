/**
 ******************************************************************************
 * @file    serial_port.h
 * @brief   串口模块（serial_module）在本工程中的挂载接口
 *
 * @details 本工程只有 USART1（PB6 = TX, PB7 = RX, 115200 8N1）一个串口，
 *          RX/TX 均为 DMA 传输且已打开 USART1 全局中断（IDLE 收帧与
 *          发送完成回调依赖它）。详见 serial_port.c 顶部说明。
 ******************************************************************************
 */
#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化串口模块并启动 USART1 的 DMA 空闲接收
 * @retval true 成功；false 失败（串口句柄为空或 DMA 启动失败）
 * @note   RTOS 模式（SERIAL_MODULE_USE_OS2 = 1）下内部会创建调度线程，
 *         因此必须在 osKernelInitialize() 之后调用（本工程放在
 *         MX_FREERTOS_Init() 内），且只能调用一次。
 */
bool SerialPort_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_PORT_H */
