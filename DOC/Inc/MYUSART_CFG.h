/**
 * @file    MYUSART_CFG.h
 * @brief   串口通用模块 —— 用户配置文件（移植时只需改这个文件）
 *
 * 使用方法：
 *   1. 把本文件复制到你的工程（或直接修改），按硬件实际情况修改各宏；
 *   2. SERIAL_MODULE_HAL_INCLUDE 指向能提供 UART_HandleTypeDef 的头文件；
 *   3. 裸机工程把 SERIAL_MODULE_USE_OS2 设为 0，并在主循环周期性调用
 *      SerialModule_Poll()；RTOS 工程设为 1，模块自建收发调度线程。
 */
#ifndef MYUSART_CFG_H
#define MYUSART_CFG_H

/* ---------------- 平台 ---------------- */
/* 提供 UART_HandleTypeDef / HAL_UART_* / __get_PRIMASK 等定义的头文件 */
#define SERIAL_MODULE_HAL_INCLUDE       "main.h"

/* 1 = 基于 CMSIS-RTOS2(FreeRTOS) 自动调度；0 = 裸机模式（需轮询 Poll） */
#define SERIAL_MODULE_USE_OS2           1

/* ---------------- 容量 ---------------- */
#define SERIAL_MODULE_PORT_MAX          4       /* 最多支持的串口实例数   */
#define SERIAL_MODULE_PAYLOAD_MAX       64      /* 帧净荷最大字节数       */
#define SERIAL_MODULE_TX_DEPTH          8       /* 每端口发送队列深度     */
#define SERIAL_MODULE_RX_DEPTH          8       /* 每端口待处理帧队列深度 */
#define SERIAL_MODULE_RX_DMA_BUF        128     /* 接收 DMA 缓冲区大小    */
#define SERIAL_MODULE_CMD_MAX           16      /* 命令分发表容量         */

/* ---------------- 行为 ---------------- */
/* 收到未知命令/CRC 错误时是否自动回错误应答帧（1=回，0=静默仅计数） */
#define SERIAL_MODULE_AUTO_NACK         1

/* ---------------- RTOS 模式专用 ---------------- */
#if SERIAL_MODULE_USE_OS2
#include "cmsis_os2.h"
#define SERIAL_MODULE_TASK_NAME         "ser_mod"
#define SERIAL_MODULE_TASK_STACK        1024            /* 字节 */
#define SERIAL_MODULE_TASK_PRIO         osPriorityNormal
#define SERIAL_MODULE_POLL_PERIOD_MS    100             /* 兜底扫描周期 ms */
#endif

#endif /* MYUSART_CFG_H */
