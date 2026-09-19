/**
 ******************************************************************************
 * @file    app_sensor.c
 * @brief   应用工作流实现：TIM4(1s) 中断 → 采集温湿度 → SX1280 发射 → 串口上报
 *
 * @details 流程与上下文说明见 app_sensor.h。
 *          本文件只调用各模块的对外 API，不直接碰寄存器。
 ******************************************************************************
 */
#include "app_sensor.h"

#include "main.h"
#include "tim.h"
#include "cmsis_os2.h"

#include "hdc1080.h"
#include "sx1280.h"
#include "serial_module.h"
#include "bsp_port.h"

/*============================================================================
 * 配置
 *==========================================================================*/
#define APP_SENSOR_TASK_STACK       (1024U)  /*!< SensorTask 栈大小（字节）
                                                  @note -O0 调试构建下调用链较深
                                                  （任务→SX1280_Init→CmdOp→SpiFrame→
                                                  HAL_SPI_TransmitReceive，且含 FPU
                                                  上下文），512B 余量不足，故取 1KB。 */
#define APP_SENSOR_TX_TIMEOUT_MS    (500U)   /*!< 单次发射的轮询超时（ms）       */
#define APP_SENSOR_SERIAL_PORT      (0U)     /*!< 上报使用的串口模块端口索引     */
#define APP_SENSOR_PAYLOAD_LEN      (4U)     /*!< 数据帧净荷长度                 */

/*============================================================================
 * 内部变量
 *==========================================================================*/
static osSemaphoreId_t s_tick_sem = NULL;    /*!< 二值信号量：TIM4 中断 → 任务 */
static osThreadId_t    s_task     = NULL;

/*============================================================================
 * 内部函数声明
 *==========================================================================*/
static void app_sensor_task(void *argument);
static void app_sensor_report_status(uint8_t code, int8_t driver_ret,
                                     uint8_t use_i2c_diag);
static uint8_t app_sensor_try_hdc_init(int8_t *ret_out);

/*============================================================================
 * 对外接口
 *==========================================================================*/

void App_Sensor_Init(void)
{
    static const osThreadAttr_t task_attr = {
        .name       = "SensorTask",
        .stack_size = APP_SENSOR_TASK_STACK,
        .priority   = (osPriority_t)osPriorityNormal,
    };

    /* 二值信号量：max_count = 1，初值 0（上电先等 TIM4 的第一个中断） */
    s_tick_sem = osSemaphoreNew(1U, 0U, NULL);
    if (s_tick_sem == NULL) {
        Error_Handler();
    }

    s_task = osThreadNew(app_sensor_task, NULL, &task_attr);
    if (s_task == NULL) {
        Error_Handler();
    }
}

void App_Sensor_Tim4Callback(void)
{
    /* 中断上下文：仅释放信号量。
     * osSemaphoreRelease() 在中断中会自动使用 xSemaphoreGiveFromISR() +
     * portYIELD_FROM_ISR()（cmsis_os2.c:1631-1638），TIM4 优先级为 5，
     * 等于 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY，允许调用。
     * 信号量已满（上一轮未处理完）时返回 osErrorResource，此处有意忽略。 */
    if (s_tick_sem != NULL) {
        (void)osSemaphoreRelease(s_tick_sem);
    }
}

/*============================================================================
 * 内部函数实现
 *==========================================================================*/

/**
 * @brief 上报一个状态帧（5 字节，布局见 app_sensor.h）
 * @param code        故障码 APP_SENSOR_ST_xxx
 * @param driver_ret  驱动返回码（HDC1080_xxx / SX1280_xxx）
 * @param use_i2c_diag 1 = 附带最近一次 HAL I2C 状态/ErrorCode/State；0 = 填 0
 */
static void app_sensor_report_status(uint8_t code, int8_t driver_ret,
                                     uint8_t use_i2c_diag)
{
    uint8_t status[APP_SENSOR_STATUS_LEN];
    BSP_I2cDiag_t diag = {0};

    status[0] = code;
    status[1] = (uint8_t)driver_ret;
    if (use_i2c_diag != 0U) {
        BSP_I2C_GetDiag(&diag);
        status[2] = diag.hal_status;
        status[3] = diag.error_code;
        status[4] = diag.state;
    } else {
        status[2] = 0U;
        status[3] = 0U;
        status[4] = 0U;
    }

    (void)SerialModule_Send(APP_SENSOR_SERIAL_PORT, APP_SENSOR_CMD_STATUS,
                            status, (uint8_t)sizeof(status));
}

/**
 * @brief 尝试初始化 HDC1080（先做总线恢复，再初始化）
 * @retval 1 成功；0 失败（失败原因可由 BSP_I2C_GetDiag() 与返回码取得）
 */
static uint8_t app_sensor_try_hdc_init(int8_t *ret_out)
{
    int8_t ret;

    /* 总线恢复：发 9 个手动时钟 + STOP，并复位 I2C 外设。
     * 解决"从机把 SDA 拉死 → BUSY 标志卡住 → HAL 直接返回 BUSY、零波形"，
     * 以及热插拔/上电时序不同步的问题。 */
    // BSP_I2C_BusRecover();

    ret = HDC1080_Init();            /* 内含上电 15ms 等待 + 器件 ID 校验 */
    if (ret_out != NULL)
    {
        *ret_out = ret;
    }
    return (ret == HDC1080_OK) ? 1U : 0U;
}

/**
 * @brief 工作流任务：初始化外设 → 等节拍 → 采集 → 发射 → 上报
 */
static void app_sensor_task(void *argument)
{
    SX1280_Config_t cfg = SX1280_CONFIG_DEFAULT;  /* 2400MHz/SF7/BW800/CR4-5/0x12/+13dBm */
    HDC1080_data_t  data;
    uint8_t         payload[APP_SENSOR_PAYLOAD_LEN];
    uint8_t         sensor_ready;
    uint8_t         radio_ready;
    uint8_t         hdc_retry_cnt = 0U;
    int8_t          hdc_ret = HDC1080_ERR_I2C;
    int8_t          radio_ret;

    (void)argument;

    /* DWT 周期计数器：SX1280 移植层微秒延时用 */
    BSP_Port_Init();

    /* ---- 1. 传感器初始化（总线恢复 + 上电等待 + 器件 ID 校验） ---- */
    sensor_ready = app_sensor_try_hdc_init(&hdc_ret);
    if (sensor_ready == 0U) {
        app_sensor_report_status(APP_SENSOR_ST_HDC1080_FAIL, hdc_ret, 1U);
    }

    /* ---- 2. 射频初始化：NRESET 复位 + LoRa 全参数配置 ---- */
    radio_ret = SX1280_Init(&cfg);
    radio_ready = (radio_ret == SX1280_OK) ? 1U : 0U;
    if (radio_ready == 0U) {
        app_sensor_report_status(APP_SENSOR_ST_SX1280_FAIL, radio_ret, 0U);
    }

    /* ---- 3. 启动 TIM4 1s 节拍（初始化完成后再启动，避免期间节拍堆积） ---- */
    if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) {
        Error_Handler();
    }

    /* ---- 4. 主循环：每个 TIM4 中断处理一轮 ---- */
    for (;;) {
        if (osSemaphoreAcquire(s_tick_sem, osWaitForever) != osOK) {
            continue;
        }

        /* 心跳指示：每轮翻转一次 PC13（LED） */
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

        /* 传感器不可用：按 APP_SENSOR_HDC_RETRY_S 秒周期重试初始化，
         * 期间每拍上报一次带诊断信息的状态帧（不再永久锁死） */
        if (sensor_ready == 0U) {
            hdc_retry_cnt++;
            if (hdc_retry_cnt < APP_SENSOR_HDC_RETRY_S) {
                app_sensor_report_status(APP_SENSOR_ST_HDC1080_FAIL, hdc_ret, 1U);
                continue;
            }
            hdc_retry_cnt = 0U;

            sensor_ready = app_sensor_try_hdc_init(&hdc_ret);
            if (sensor_ready == 0U) {
                app_sensor_report_status(APP_SENSOR_ST_HDC1080_FAIL, hdc_ret, 1U);
            } else {
                /* 恢复成功：上报一次 OK 状态帧（净荷第 1 字节为驱动返回码 0） */
                app_sensor_report_status(APP_SENSOR_ST_OK, hdc_ret, 1U);
            }
            continue;
        }

        /* 触发 + 等待转换（约 14ms，内部走 osDelay）+ 一次事务读回 4 字节 */
        {
            int8_t meas_ret = HDC1080_Measure(&data);
            if (meas_ret != HDC1080_OK) {
                hdc_ret = meas_ret;      /* 保留真实驱动错误码供上报 */
                app_sensor_report_status(APP_SENSOR_ST_HDC1080_FAIL, hdc_ret, 1U);
                continue;
            }
        }

        /* 净荷：温度 raw 大端 + 湿度 raw 大端 */
        payload[0] = (uint8_t)(data.temp_raw >> 8);
        payload[1] = (uint8_t)(data.temp_raw & 0xFFU);
        payload[2] = (uint8_t)(data.hum_raw >> 8);
        payload[3] = (uint8_t)(data.hum_raw & 0xFFU);

        /* 2.4G 发射（阻塞式轮询 TxDone/超时） */
        if (radio_ready != 0U) {
            int8_t tx_ret = SX1280_Transmit(payload, (uint8_t)sizeof(payload),
                                            APP_SENSOR_TX_TIMEOUT_MS);
            if (tx_ret != SX1280_OK) {
                app_sensor_report_status(APP_SENSOR_ST_TX_FAIL, tx_ret, 0U);
            }
        }

        /* 串口上报（入队即返回，DMA 后台发送；发射失败也照常上报数据） */
        (void)SerialModule_Send(APP_SENSOR_SERIAL_PORT, APP_SENSOR_CMD_DATA,
                                payload, (uint8_t)sizeof(payload));
    }
}

/* 文件结束 */
