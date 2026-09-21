//
// Created by WIN on 2026/9/19.
//

#ifndef FreeRTOS_Task_H
#define FreeRTOS_Task_H
#include "cmsis_os2.h"
#include "MYUSART.h"
#include "App_Config.h"

/* ---------------- 线程标志位 ----------------
 * CMSIS-RTOS2 的线程标志按线程独立编号，不同线程用同一个位值互不影响，
 * 这里按“信号语义”命名，便于阅读。
 */
#define BIT_SENSOR_TICK   0x01U   /* TIM4 1s 节拍        -> HDC1080Task */
#define BIT_LED_BLINK     0x01U   /* HDC1080Task         -> LEDTask    */
#define BIT_TX_QUEUE      0x01U   /* 任意生产者          -> USARTTask  */
#define BIT_SX1280_IRQ    0x01U   /* DIO1/2/3 EXTI       -> SX1280Task */
#define BIT_SX1280_TXREQ  0x02U   /* 生产者              -> SX1280Task */

/* ---------------- 任务间串口发送消息 ----------------
 * 生产者只填 cmd/len/data 并投递；组帧（SOF/LEN/CRC8）与 DMA 发送
 * 全部由 MYUSART 模块在 SerialModule_Send() 内完成，生产者不碰硬件。
 * 本工程串口端口索引固定为 0（对应 USART1）。
 */
#define USART_TX_PORT         0u
#define USART_TX_QUEUE_DEPTH  8u

typedef struct {
    uint8_t cmd;                     /* 命令码（帧头之后的第一个字节） */
    uint8_t len;                     /* 净荷长度，<= SM_PAYLOAD_MAX    */
    uint8_t data[SM_PAYLOAD_MAX];    /* 净荷                           */
} UsartTxMsg_t;

extern osThreadId_t LEDTaskHandle;
extern osThreadId_t HDC1080TaskHandle;
extern osThreadId_t USARTTaskHandle;
extern osMessageQueueId_t UsartTxQueue;

/**
 * @brief  投递一帧到串口发送队列（任意任务上下文可调用）
 * @param  cmd  命令码
 * @param  data 净荷指针（可为 NULL，此时 len 必须为 0）
 * @param  len  净荷长度，不得超过 SM_PAYLOAD_MAX
 * @retval true  已入队并已唤醒 USARTTask
 * @retval false 参数非法 / 队列满 / 队列尚未创建
 * @note   非阻塞：入队失败立即返回，不等待。队列满时本帧被丢弃，
 *         下层的 MYUSART 模块另有 tx_dropped 统计。
 */
bool UsartTx_Post(uint8_t cmd, const uint8_t *data, uint8_t len);

/* ---------------- HDC1080 遥测净荷（CMD = 0x18） ----------------
 * 每 1 秒一帧，净荷 7 字节，全部小端：
 *   [0]    status  0 = 本次测量成功；非 0 = HDC1080_Status_t 返回码
 *                  （-1 I2C 失败 / -3 参数错误 / -5 器件 ID 不符 …）
 *   [1..2] seq     递增序号（uint16），用于判断丢帧与乱序
 *   [3..4] temp    温度 ×100（int16），例 2356 → 23.56 °C
 *   [5..6] hum     湿度 ×100（uint16），例 4821 → 48.21 %RH
 * 测量失败时 temp/hum 填 0，失败原因完全由 status 给出。
 */
#define HDC_TELEMETRY_CMD   0x18u
#define HDC_TELEMETRY_LEN   7u

/**
 * @brief  记录 HDC1080 初始化结果（由 main.c 在 RTOS 启动前调用）
 * @param  status HDC1080_Init() 的返回值（HDC1080_OK 或错误码）
 * @note   HDC1080Task 每个节拍把它放进遥测净荷的 status 字节上报，
 *         这样初始化失败不会被静默吞掉（原先只能挂调试器才发现）。
 */
void App_SetHdcInitStatus(int8_t status);

/**
 * @brief  记录 SX1280 初始化结果（由 main.c 在任何 osThreadNew 之前调用）
 * @param  status MYSX1280_Init() 的返回值
 *
 * @warning ⚠ 必须在【任何 osThreadNew / xTaskCreate 之前】调用，否则会死机。
 *   原因：FreeRTOS 的 Cortex-M4F 移植把临界区嵌套计数 uxCriticalNesting 的初值
 *   设为哨兵 0xaaaaaaaa，只有调度器启动第一个任务时(vPortSVCHandler)才清成 0。
 *   因此调度器启动前只要发生过一次 xTaskCreate，临界区退出时计数无法归零，
 *   portENABLE_INTERRUPTS() 永不执行，BASEPRI 就永久停在
 *   configMAX_SYSCALL_INTERRUPT_PRIORITY(=0x50)，从而屏蔽掉优先级 5 的
 *   TIM10 时基 —— 此后任何 HAL_Delay() 都会死等（uwTick 不再递增）。
 *   MYSX1280_Init() 内部的 SX1280_Reset() 会调用 HAL_Delay，所以它必须放在
 *   SerialModule_Init()（其内部 osThreadNew 会创建任务）之前。
 *   实测：放错位置时 BASEPRI=0x50、uwTick 冻结在 16、栈回溯停在
 *   SX1280_Reset → SX1280_DelayMs → HAL_Delay。
 */
void App_SetSxInitStatus(int8_t status);

/* ---------------- SX1280 待发空中帧 ----------------
 * ⚠ 驱动 SX1280.c 的互斥用的是 PRIMASK 临界区 + try-lock（SX1280_SpiLock 在
 *    已占用时返回 SX1280_ERR_BUSY，不会阻塞等待），因此 SX1280_* 的所有接口
 *    都【必须】在同一个任务里串行调用。本工程统一由 SX1280Task 独占访问：
 *    其它任务只通过下面这个队列把待发帧交给它。
 */
typedef struct {
    uint8_t len;                     /* 实际长度，<= AIR_FRAME_LEN */
    uint8_t data[AIR_FRAME_LEN];     /* 空中帧（含 2 字节包头）    */
} Sx1280TxMsg_t;

extern osThreadId_t SX1280TaskHandle;
extern osMessageQueueId_t SxTxQueue;

/**
 * @brief  投递一帧到 SX1280 发送队列（任意任务上下文可调用）
 * @param  frame 空中帧字节（含包头）
 * @param  len   帧长度，不得超过 AIR_FRAME_LEN
 * @retval true  已入队并已唤醒 SX1280Task
 * @retval false 参数非法 / 队列满 / 队列尚未创建
 * @note   非阻塞。真正的 SX1280_StartTransmit() 由 SX1280Task 执行，
 *         避免跨任务并发访问 SPI 与驱动状态机。
 */
bool Sx1280Tx_Post(const uint8_t *frame, uint8_t len);

void MX_FREERTOS_Init(void);

/* 【临时诊断】启动阶段与中断屏蔽状态快照（定位启动期卡死用）
 * 每调用一次记录当前 PRIMASK/BASEPRI 到 stage 槽位，便于判断是哪个调用
 * 抬起了中断屏蔽位。stage 取值 0~7。【定位完成后删除】 */
void App_BootMark(uint32_t stage);

#endif //FreeRTOS_Task_H
