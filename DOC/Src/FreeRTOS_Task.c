//
// Created by WIN on 2026/9/19.
//
#include <string.h>

#include "main.h"
/* cmsis_os2.h 只带 <stdint.h>/<stddef.h>，不含 FreeRTOS 头；
 * 栈溢出钩子需要 TaskHandle_t（task.h）与 configMAX_TASK_NAME_LEN（FreeRTOS.h）。 */
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_Task.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "MYUSART.h"
#include "HDC1080.h"

osThreadId_t LEDTaskHandle;
osThreadId_t HDC1080TaskHandle;
osThreadId_t USARTTaskHandle;
osMessageQueueId_t UsartTxQueue;

static osThreadAttr_t LEDTask_attributes = {.name = "LEDTask",.stack_size = 512,.priority = osPriorityLow,};
/* HDC1080Task 会经 HDC1080_Measure 调用 HAL_I2C_Mem_Read，调用链较深
 * （Measure→ReadConfig→ReadReg16→I2C_Read→HAL_I2C_Mem_Read→I2C_WaitOnFlagUntilTimeout）。
 * 1024 字节偏紧：一旦栈溢出会静默破坏 FreeRTOS 堆内存，最终以 configASSERT
 * 死循环（关中断 + 死等）表现为整机死机 —— 实测到过 uwTick 停摆、
 * hi2c1.State 永停 BUSY_RX、软件复位无法恢复（只能断电）。故提高到 2048。 */
static osThreadAttr_t HDC1080Task_attributes = {.name = "HDC1080",.stack_size = 2048,.priority = osPriorityNormal,};
static osThreadAttr_t USARTTask_attributes = {.name = "USART",.stack_size = 2024,.priority = osPriorityHigh,};

/* ================= 串口模块（MYUSART）工程适配 =================
 * 端口表内容会被模块引用，须在整个运行期有效，故声明为 static const。
 * 当前只挂 USART1（PB6/PB7，115200，RX=DMA2_Stream2 / TX=DMA2_Stream7）。
 * 后续增加串口只需在表里追加一项，数量不得超过 SERIAL_MODULE_PORT_MAX(4)。
 */
static const SerialModule_PortCfg_t s_serial_ports[] = {{ .huart = &huart1, .name = "usart1", .frame_cb = NULL },};

/* ================= HDC1080 初始化结果 =================
 * main.c 在 osKernelStart() 之前调用 HDC1080_Init()（此时 TIM10 时基已在跑，
 * 内部 HAL_Delay(15) 有效）。结果记录在这里，由 HDC1080Task 每个节拍放进
 * 遥测净荷的 status 字节上报，避免初始化失败被静默吞掉。
 */
static volatile int8_t s_hdc_init_status = HDC1080_OK;

void App_SetHdcInitStatus(int8_t status)
{
    s_hdc_init_status = status;
}

/* ================= 任务间串口发送接口 ================= */

bool UsartTx_Post(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    UsartTxMsg_t msg;

    if (UsartTxQueue == NULL)         { return false; }
    if (len > SM_PAYLOAD_MAX)         { return false; }
    if ((len > 0u) && (data == NULL)) { return false; }

    msg.cmd = cmd;
    msg.len = len;
    if (len > 0u) { memcpy(msg.data, data, len); }

    /* 超时 0：队列满就立即失败返回，绝不阻塞生产者 */
    if (osMessageQueuePut(UsartTxQueue, &msg, 0U, 0U) != osOK) { return false; }

    (void)osThreadFlagsSet(USARTTaskHandle, BIT_TX_QUEUE);
    return true;
}

/* ================= 任务 ================= */

static void LEDTask(void *pvParameters)
{
    while (1)
    {
        osThreadFlagsWait(BIT_LED_BLINK,osFlagsWaitAny,osWaitForever);

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}

/* 浮点 → 定点 ×100（四舍五入）。
 * 取值范围：温度 -40~125°C → -4000~12500；湿度 0~100% → 0~10000，
 * 均落在 int16 内，因此遥测净荷用定点即可，PC 端不必解浮点。 */
static int16_t Hdc_ToX100(float v)
{
    return (int16_t)((v >= 0.0f) ? (v * 100.0f + 0.5f) : (v * 100.0f - 0.5f));
}

/* ================= FreeRTOS 栈溢出钩子 =================
 * cmsis_os2.c 里这个钩子是空的弱函数 —— 栈溢出会静默破坏堆内存，之后往往以
 * configASSERT 死循环收场（taskDISABLE_INTERRUPTS + for(;;)），表现为整机死机。
 * 这里覆盖为可观测实现：只复制任务名 + 置标志，不调用任何 RTOS API
 * （本钩子可能在调度器临界区内被调用），由 HDC1080Task 在下一个节拍发帧上报。
 * 独立命令码 0xE0，净荷 = 溢出的任务名（ASCII）。
 */
#define STACK_OVF_CMD   0xE0u

static volatile uint8_t s_stack_ovf_flag = 0u;
static char             s_stack_ovf_task[configMAX_TASK_NAME_LEN];

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    uint8_t i;

    (void)xTask;

    if (pcTaskName != NULL) {
        for (i = 0u; i < (uint8_t)(configMAX_TASK_NAME_LEN - 1u); i++) {
            s_stack_ovf_task[i] = pcTaskName[i];
            if (pcTaskName[i] == '\0') { break; }
        }
        s_stack_ovf_task[configMAX_TASK_NAME_LEN - 1u] = '\0';
    }
    s_stack_ovf_flag = 1u;
}

static void HDC1080Task(void *pvParameters)
{
    HDC1080_data_t data;
    uint8_t  payload[HDC_TELEMETRY_LEN];
    uint16_t seq = 0u;
    int8_t   status;

    while (1)
    {
        /* TIM4 的 1s 节拍（见 Back.c 的 HAL_TIM_PeriodElapsedCallback） */
        osThreadFlagsWait(BIT_SENSOR_TICK,osFlagsWaitAny,osWaitForever);

        /* 流程走通指示：HDC1080Task 每被唤醒一次，LED 翻转一次 */
        osThreadFlagsSet(LEDTaskHandle,BIT_LED_BLINK);

        /* 0. 栈溢出上报：钩子里不能调用 RTOS API，统一在这里发一帧 */
        if (s_stack_ovf_flag != 0u) {
            uint8_t n = 0u;
            while ((n < (uint8_t)(configMAX_TASK_NAME_LEN - 1u)) &&
                   (s_stack_ovf_task[n] != '\0')) { n++; }
            (void)UsartTx_Post(STACK_OVF_CMD, (const uint8_t *)s_stack_ovf_task, n);
            s_stack_ovf_flag = 0u;
        }

        /* 1. 测量：初始化就失败时直接沿用该错误码，不再去碰 I2C */
        status = s_hdc_init_status;
        if (status == HDC1080_OK) {
            status = HDC1080_Measure(&data);
        }

        /* 2. 组帧：status + 序号 + 温度×100 + 湿度×100（全部小端）。 */
        payload[0] = (uint8_t)status;
        payload[1] = (uint8_t)(seq & 0xFFu);
        payload[2] = (uint8_t)((seq >> 8) & 0xFFu);
        if (status == HDC1080_OK) {
            int16_t  t100 = Hdc_ToX100(data.temp_c);
            uint16_t h100 = (uint16_t)Hdc_ToX100(data.hum_rh);
            payload[3] = (uint8_t)((uint16_t)t100 & 0xFFu);
            payload[4] = (uint8_t)(((uint16_t)t100 >> 8) & 0xFFu);
            payload[5] = (uint8_t)(h100 & 0xFFu);
            payload[6] = (uint8_t)((h100 >> 8) & 0xFFu);
        } else {
            /* 测量失败：数据域清零，失败原因由 status 说明 */
            payload[3] = 0u;
            payload[4] = 0u;
            payload[5] = 0u;
            payload[6] = 0u;
        }
        seq++;

        /* 3. 交给 USARTTask 发送；SX1280 后续在同一处并行投递 */
        (void)UsartTx_Post(HDC_TELEMETRY_CMD, payload, HDC_TELEMETRY_LEN);
    }
}

static void USARTTask(void *pvParameters)
{
    UsartTxMsg_t msg;

    while (1)
    {
        osThreadFlagsWait(BIT_TX_QUEUE,osFlagsWaitAny,osWaitForever);

        /* 一次唤醒可能对应多帧：把队列排空再回去等标志。
         * SerialModule_Send() 内部再入模块自己的发送队列并由 DMA 发出，
         * 这里不直接碰 HAL_UART_Transmit_DMA，避免与模块的 TX 状态机冲突。 */
        while (osMessageQueueGet(UsartTxQueue, &msg, NULL, 0U) == osOK)
        {
            (void)SerialModule_Send(USART_TX_PORT, msg.cmd, msg.data, msg.len);
        }
    }
}

void MX_FREERTOS_Init(void)
{
    /* SerialModule_Init 必须在 osKernelInitialize() 之后调用（内部会创建调度线程）。
     * main.c 的时序为 osKernelInitialize() -> MX_FREERTOS_Init() -> osKernelStart()，
     * 在此处调用刚好满足模块要求。失败说明 huart1 未就绪或 DMA 启动失败。 */
    if (!SerialModule_Init(s_serial_ports, 1)) {Error_Handler();}

    /* 发送队列必须先于任务创建，否则 UsartTx_Post 会因队列为 NULL 而丢帧 */
    UsartTxQueue = osMessageQueueNew(USART_TX_QUEUE_DEPTH, sizeof(UsartTxMsg_t), NULL);
    if (UsartTxQueue == NULL) {Error_Handler();}

    LEDTaskHandle = osThreadNew(LEDTask, NULL, &LEDTask_attributes);
    HDC1080TaskHandle = osThreadNew(HDC1080Task, NULL, &HDC1080Task_attributes);
    USARTTaskHandle = osThreadNew(USARTTask, NULL, &USARTTask_attributes);
}
