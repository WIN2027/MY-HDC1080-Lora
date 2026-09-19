//
// Created by WIN on 2026/9/19.
//

#ifndef FreeRTOS_Task_H
#define FreeRTOS_Task_H
#include "cmsis_os2.h"
#include "MYUSART.h"

/* ---------------- 线程标志位 ----------------
 * CMSIS-RTOS2 的线程标志按线程独立编号，不同线程用同一个位值互不影响，
 * 这里按“信号语义”命名，便于阅读。
 */
#define BIT_SENSOR_TICK  0x01U   /* TIM4 1s 节拍        -> HDC1080Task */
#define BIT_LED_BLINK    0x01U   /* HDC1080Task         -> LEDTask    */
#define BIT_TX_QUEUE     0x01U   /* 任意生产者          -> USARTTask  */

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

void MX_FREERTOS_Init(void);

#endif //FreeRTOS_Task_H
