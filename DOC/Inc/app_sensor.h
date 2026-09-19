/**
 ******************************************************************************
 * @file    app_sensor.h
 * @brief   应用工作流：TIM4(1s) 中断 → 采集温湿度 → SX1280 发射 → 串口上报
 *
 * @details 执行模型（不在中断里做重活）：
 *            TIM4 更新中断（1s，NVIC 优先级 5）
 *              └─ App_Sensor_Tim4Callback()  ← 在 HAL_TIM_PeriodElapsedCallback 中调用
 *                   └─ osSemaphoreRelease(二值信号量)  ← ISR 内自动走 FromISR 版本
 *            SensorTask（osPriorityNormal）
 *              └─ 等信号量 → HDC1080_Measure() → SX1280_Transmit() → SerialModule_Send()
 *
 *          原因：HDC1080 一次采集需等待转换约 14 ms，SX1280 一次发射为
 *          阻塞式轮询（数百 ms 量级），且 sx1280.c 顶部明确要求"不要在中断
 *          上下文调用"；信号量为二值（max=1），若上一轮尚未处理完，多余
 *          的中断只丢弃计数，不会堆积。
 *
 * @note    串口上报帧格式（serial_module 协议）：
 *            SOF(5A) | CMD | LEN | DATA | CRC8
 *            CMD = 0x10 数据帧：LEN = 4，DATA = 温度 raw(2B 大端) + 湿度 raw(2B 大端)
 *                                换算见 HDC1080 手册 8.6 节：
 *                                T(°C) = raw/2^16*165 - 40；RH(%) = raw/2^16*100
 *            CMD = 0x11 状态帧：LEN = 1，DATA = 故障码（见下方宏）
 *          0x11 为本工程新增的诊断帧，若上位机不需要，删除对应
 *          SerialModule_Send() 调用即可，不影响 0x10 数据帧。
 ******************************************************************************
 */
#ifndef APP_SENSOR_H
#define APP_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * 串口上报命令码与净荷定义
 *==========================================================================*/
#define APP_SENSOR_CMD_DATA     (0x10U)  /*!< 数据帧：温/湿度原始值（各 2B 大端） */
#define APP_SENSOR_CMD_STATUS   (0x11U)  /*!< 状态帧：5B，见下                 */

/* 故障码（APP_SENSOR_CMD_STATUS 净荷的第 0 字节） */
#define APP_SENSOR_ST_OK            (0U) /*!< 故障恢复（传感器重新通信成功）           */
#define APP_SENSOR_ST_HDC1080_FAIL  (1U) /*!< HDC1080 初始化/采集失败（I2C 无应答等） */
#define APP_SENSOR_ST_SX1280_FAIL   (2U) /*!< SX1280 初始化失败（SPI 通信/BUSY 超时）  */
#define APP_SENSOR_ST_TX_FAIL       (3U) /*!< SX1280 发射失败（超时或命令错误）        */

/**
 * @brief 状态帧净荷布局（CMD = 0x11，LEN = 5）
 *
 *   [0] 故障码            APP_SENSOR_ST_xxx
 *   [1] 驱动返回码         HDC1080_xxx / SX1280_xxx 的错误码（负数按补码存放）
 *   [2] HAL 返回状态       HAL_StatusTypeDef：0=OK 1=ERROR 2=BUSY 3=TIMEOUT
 *   [3] hi2c1.ErrorCode    bit0=BERR bit1=ARLO **bit2=AF(NACK 无应答)**
 *                          bit3=OVR bit5=TIMEOUT
 *   [4] hi2c1.State        0x20=READY 0x21=BUSY_TX 0x22=BUSY_RX
 *                          0x24=TIMEOUT 0x25=ERROR
 *
 * 判读速查（HDC1080 故障时）：
 *   2 0 0x20    → HAL_BUSY 且总线看起来空闲 = BUSY 标志卡住/总线被拉死（零波形）
 *   1 0x04 *    → HAL_ERROR + AF = 从机不应答（器件未供电/地址不对/接线断）
 *   3 0x20 *    → 超时（上拉太弱、总线电容过大、SDA/SCL 接反）
 *   对 SX1280 故障（2/3），[2]..[4] 无意义（填 0）。
 */
#define APP_SENSOR_STATUS_LEN       (5U)

/**
 * @brief HDC1080 初始化失败后的重试间隔（秒，等于 TIM4 节拍数）
 * @note  失败后不再永久锁死：每隔该间隔重试一次 HDC1080_Init()，
 *        使总线上周期性出现事务（便于示波器/逻辑分析仪抓取），
 *        并支持"传感器后上电/插拔后自恢复"。
 */
#define APP_SENSOR_HDC_RETRY_S      (5U)

/*============================================================================
 * API
 *==========================================================================*/

/**
 * @brief  创建工作流（信号量 + SensorTask）
 * @note   必须在 osKernelInitialize() 之后、osKernelStart() 之前调用（本工程
 *         放在 MX_FREERTOS_Init() 内）；传感器与射频的实际初始化在任务里
 *         完成，避免调度器启动前长时间阻塞。
 */
void App_Sensor_Init(void);

/**
 * @brief  TIM4 更新中断回调（1s 周期）
 * @note   在中断上下文执行，只释放信号量，不做任何阻塞操作。
 *         由 main.c 的 HAL_TIM_PeriodElapsedCallback() 调用。
 */
void App_Sensor_Tim4Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSOR_H */
