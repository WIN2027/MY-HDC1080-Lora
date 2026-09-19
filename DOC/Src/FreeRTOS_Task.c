//
// Created by WIN on 2026/9/19.
//
#include <string.h>

#include "main.h"
#include "FreeRTOS_Task.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "MYUSART.h"

osThreadId_t LEDTaskHandle;
osThreadId_t HDC1080TaskHandle;
osThreadId_t USARTTaskHandle;
osMessageQueueId_t UsartTxQueue;

static osThreadAttr_t LEDTask_attributes = {.name = "LEDTask",.stack_size = 512,.priority = osPriorityLow,};
static osThreadAttr_t HDC1080Task_attributes = {.name = "HDC1080",.stack_size = 1024,.priority = osPriorityNormal,};
static osThreadAttr_t USARTTask_attributes = {.name = "USART",.stack_size = 2024,.priority = osPriorityHigh,};

/* ================= 串口模块（MYUSART）工程适配 =================
 * 端口表内容会被模块引用，须在整个运行期有效，故声明为 static const。
 * 当前只挂 USART1（PB6/PB7，115200，RX=DMA2_Stream2 / TX=DMA2_Stream7）。
 * 后续增加串口只需在表里追加一项，数量不得超过 SERIAL_MODULE_PORT_MAX(4)。
 */
static const SerialModule_PortCfg_t s_serial_ports[] = {{ .huart = &huart1, .name = "usart1", .frame_cb = NULL },};

/* ================= HDC1080 测试数据（占位，待替换） =================
 * 命令码沿用 FOC 上位机 Text.py 的 CMD_TELEMETRY(0x18)，便于直接观察。
 * 净荷 4 字节：
 *   data[0..1] 递增序号（小端）—— 用于判断丢帧、以及收发是否同步
 *   data[2..3] 固定模式 A5 5A —— 用于判断字节序与帧对齐
 * SX1280 与 HDC1080 驱动尚未纳入工程，此处先用测试数据打通链路。
 */
#define TEST_TELEMETRY_CMD   0x18u
#define TEST_TELEMETRY_LEN   4u

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

static void HDC1080Task(void *pvParameters)
{
    uint16_t seq = 0u;
    uint8_t  payload[TEST_TELEMETRY_LEN];

    while (1)
    {
        /* TIM4 的 1s 节拍（见 Back.c 的 HAL_TIM_PeriodElapsedCallback） */
        osThreadFlagsWait(BIT_SENSOR_TICK,osFlagsWaitAny,osWaitForever);

        /* 流程走通指示：HDC1080Task 每被唤醒一次，LED 翻转一次 */
        osThreadFlagsSet(LEDTaskHandle,BIT_LED_BLINK);

        /* TODO: HDC1080 驱动接入后，此处改为 HDC1080_Measure() 取真实温湿度，
         *       并同步投递给 SX1280（暂未实现）。当前用测试数据占位。 */
        payload[0] = (uint8_t)(seq & 0xFFu);
        payload[1] = (uint8_t)((seq >> 8) & 0xFFu);
        payload[2] = 0xA5u;
        payload[3] = 0x5Au;
        seq++;

        /* 交给 USARTTask 发送；失败说明队列满，本条测试帧丢弃 */
        (void)UsartTx_Post(TEST_TELEMETRY_CMD, payload, TEST_TELEMETRY_LEN);
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
