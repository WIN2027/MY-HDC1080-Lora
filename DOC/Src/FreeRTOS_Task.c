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
#include "spi.h"          /* 【临时诊断】SPI 链路测试需要 hspi1 */
#include "MYUSART.h"
#include "HDC1080.h"
#include "SX1280.h"

osThreadId_t LEDTaskHandle;
osThreadId_t HDC1080TaskHandle;
osThreadId_t USARTTaskHandle;
osThreadId_t SX1280TaskHandle;
osMessageQueueId_t UsartTxQueue;
osMessageQueueId_t SxTxQueue;

static osThreadAttr_t LEDTask_attributes = {.name = "LEDTask",.stack_size = 512,.priority = osPriorityLow,};
/* HDC1080Task 会经 HDC1080_Measure 调用 HAL_I2C_Mem_Read，调用链较深
 * （Measure→ReadConfig→ReadReg16→I2C_Read→HAL_I2C_Mem_Read→I2C_WaitOnFlagUntilTimeout）。
 * 1024 字节偏紧：一旦栈溢出会静默破坏 FreeRTOS 堆内存，最终以 configASSERT
 * 死循环（关中断 + 死等）表现为整机死机 —— 实测到过 uwTick 停摆、
 * hi2c1.State 永停 BUSY_RX、软件复位无法恢复（只能断电）。故提高到 2048。 */
static osThreadAttr_t HDC1080Task_attributes = {.name = "HDC1080",.stack_size = 2048,.priority = osPriorityNormal,};
static osThreadAttr_t USARTTask_attributes = {.name = "USART",.stack_size = 2024,.priority = osPriorityHigh,};
/* SX1280Task 独占 SX1280 驱动（见 FreeRTOS_Task.h 里关于 try-lock 的说明）。
 * 优先级高于 HDC1080Task：DIO 中断到达后要尽快把包取出来，否则下一包会盖掉上一包。
 * 栈给 1536 而不是 1024：它要跑 HAL_SPI_TransmitReceive 等较深的 HAL 调用链。 */
static osThreadAttr_t SX1280Task_attributes = {.name = "SX1280",.stack_size = SX1280_TASK_STACK,.priority = osPriorityAboveNormal,};

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

/* SX1280 初始化结果。由 main.c 在【任何 osThreadNew 之前】写入 —— 原因见
 * FreeRTOS_Task.h 里 App_SetSxInitStatus() 的 @warning（BASEPRI 泄漏把 TIM10 屏蔽，
 * 之后 HAL_Delay 会死等）。
 * 这里提前声明：本文件靠后的 SX1280 业务段要用到它。 */
static volatile int8_t s_sx_init_status = SX1280_OK;

void App_SetSxInitStatus(int8_t status)
{
    s_sx_init_status = status;
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

/* ================= 【临时诊断】SPI 物理链路测试 =================
 * 为什么需要：SX1280_Init 目前返回 -4(ERR_CMD)，而且埋点显示芯片返回的状态字节
 *             是 0xFF —— 即复位后第一条 SPI 命令 SET_STANDBY 就拿到了"全 1"，
 *             说明 MISO 上没有任何有效数据。MCU 侧配置已逐项核对无误
 *             （SPI Mode0 / 12.5MHz / PA5-PA7 均为 AF / NSS 正确），
 *             所以要确认 MCU 与 SX1280 之间的 SPI 物理链路到底有没有信号。
 *
 * 做法：在 LEDTask 里周期性、【连续重复】地用 SPI1 发一条真实且安全的命令
 *       SET_STANDBY(STDBY_RC) = 0x80 0x00，并回读 MISO。
 *       测试期间把 SPI 时钟临时降到 100MHz/256 ≈ 390 kHz —— 这个速度示波器
 *       看得清，万用表也能测到平均值。
 *
 * 在 SX1280 端（对芯片引脚）应该看到：
 *   NSS  (pin 19) : 大部分时间被拉低（选中），周期性短暂拉高
 *   SCK  (pin 18) : 约 390kHz 的连续时钟方波（万用表约 1.6V 上下）
 *   MOSI (pin 17) : 每个 NSS 周期内先一个窄脉冲(0x80)再一字节 0x00
 *   MISO (pin 16) : 芯片正常应答时电平会明显翻转；
 *                   ★ 若该线断路/虚焊，会呈现恒定的高或低 ——
 *                     对应回读值是 0xFF 或 0x00，这正是当前的怀疑点
 *
 * 结果查看（可直接在 CLion 调试器里看，也可用调试探针读内存）：
 *   g_spi_test_cnt  传输次数
 *   g_spi_test_ok   成功次数（HAL_OK）
 *   g_spi_test_err  失败次数
 *   g_spi_test_rx[2] 最近一次 MISO 回读：全 0xFF / 全 0x00 都说明 MISO 无驱动
 *
 * ⚠ 临时诊断代码，三个注意点：
 *   1) 测试窗口内会占满 CPU（LEDTask 是最低优先级，不会影响其它任务），
 *      且会独占 SPI1 —— 所以只在 SX1280 驱动已经不工作时用它；
 *   2) 用完把 SPI_TEST_ENABLE 改成 0，或把本段连同 LEDTask 里的调用一起删除；
 *   3) 若 SPI_TEST_SLOW_CLOCK=1，测试结束会把时钟恢复成 12.5MHz。
 */
#define SPI_TEST_ENABLE      1
#define SPI_TEST_SLOW_CLOCK  1        /* 1=测试期间降到 390kHz，便于观察 */
#define SPI_TEST_WINDOW_MS   300u     /* 每次唤醒内连续发送的时长 */
#define SPI_TEST_LOOP_MS     500u     /* LEDTask 等待超时（决定测试重复频率） */

volatile uint32_t g_spi_test_cnt    = 0u;
volatile uint32_t g_spi_test_ok     = 0u;
volatile uint32_t g_spi_test_err    = 0u;
volatile uint8_t  g_spi_test_rx[2]  = {0u, 0u};

static void SpiLinkTest_Run(void)
{
    /* SET_STANDBY(STDBY_RC)：真实且安全的命令，芯片正常时会在 MISO 回状态字节 */
    static const uint8_t tx[2] = { 0x80u, 0x00u };
    uint8_t  rx[2];
    uint32_t t0;

#if SPI_TEST_SLOW_CLOCK
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;   /* 100MHz/256 ≈ 390kHz */
    (void)HAL_SPI_Init(&hspi1);
#endif

    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < SPI_TEST_WINDOW_MS)
    {
        HAL_GPIO_WritePin(SX1280_NSS_GPIO_Port, SX1280_NSS_Pin, GPIO_PIN_RESET);

        if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, 2U, 100U) == HAL_OK) {
            g_spi_test_ok++;
        } else {
            g_spi_test_err++;
        }

        HAL_GPIO_WritePin(SX1280_NSS_GPIO_Port, SX1280_NSS_Pin, GPIO_PIN_SET);

        g_spi_test_rx[0] = rx[0];
        g_spi_test_rx[1] = rx[1];
        g_spi_test_cnt++;
    }

#if SPI_TEST_SLOW_CLOCK
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;     /* 恢复 12.5MHz */
    (void)HAL_SPI_Init(&hspi1);
#endif
}

static void LEDTask(void *pvParameters)
{
    while (1)
    {
#if SPI_TEST_ENABLE
        /* 超时 SPI_TEST_LOOP_MS：既保住 1 秒的 LED 心跳，也周期性跑 SPI 测试 */
        uint32_t flags = osThreadFlagsWait(BIT_LED_BLINK, osFlagsWaitAny, SPI_TEST_LOOP_MS);

        if ((flags & BIT_LED_BLINK) != 0u) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }

        SpiLinkTest_Run();
#else
        osThreadFlagsWait(BIT_LED_BLINK,osFlagsWaitAny,osWaitForever);

        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
#endif
    }
}

#if (NODE_ROLE == NODE_ROLE_TX)
/* 浮点 → 定点 ×100（四舍五入）。仅发射端需要（接收端只转发放手）。
 * 取值范围：温度 -40~125°C → -4000~12500；湿度 0~100% → 0~10000，
 * 均落在 int16 内，因此遥测净荷用定点即可，PC 端不必解浮点。 */
static int16_t Hdc_ToX100(float v)
{
    return (int16_t)((v >= 0.0f) ? (v * 100.0f + 0.5f) : (v * 100.0f - 0.5f));
}
#endif

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

/* ================= 【临时诊断】启动阶段与中断屏蔽状态快照 =================
 * 现象：烧录后系统卡在 MX_FREERTOS_Init() → MYSX1280_Init() → SX1280_Reset()
 *       → SX1280_DelayMs()(=HAL_Delay) 内；TIM10 的 UIF 挂起却无人处理、
 *       uwTick 停在 16 不动，说明中断在此前已被屏蔽（HAL_Delay 死等）。
 * 用途：在启动流程每一步前后记录 PRIMASK / BASEPRI，确定是哪个调用抬起了屏蔽位。
 *       BASEPRI == 0x50 (configMAX_SYSCALL_INTERRUPT_PRIORITY<<4) 表示
 *       正处在 FreeRTOS 临界区内；Bit5..7 对应优先级 5 —— 恰好会屏蔽
 *       优先级为 5 的 TIM10。
 * 【定位完成后删除本段及相关 App_BootMark() 调用】
 */
volatile uint32_t g_boot_stage    = 0u;
volatile uint32_t g_boot_primask[8];
volatile uint32_t g_boot_basepri[8];

void App_BootMark(uint32_t stage)
{
    if (stage < 8u) {
        g_boot_primask[stage] = __get_PRIMASK();
        g_boot_basepri[stage] = __get_BASEPRI();
        g_boot_stage          = stage;      /* 最后写：保证 stage 与快照一致 */
    }
}

/* ================= SX1280 业务 =================
 * 角色由 App_Config.h 的 NODE_ROLE 决定：
 *   NODE_ROLE_TX : HDC1080Task 采集 → 本地 0x18 遥测 + 空中发帧（经 SxTxQueue 交给本任务）
 *   NODE_ROLE_RX : 连续接收 → 校验包头 → 串口 0x19 上报（含 RSSI/SNR）
 * 驱动回调（on_tx_done / on_rx_done / on_error）全部在【本任务上下文】被
 * SX1280_Process() 调用，所以回调里可以安全使用 RTOS 接口。
 */
/* 注：s_sx_init_status 已在本文件靠前处声明（App_SetSxInitStatus 需要先见到它） */
static volatile uint32_t s_sx_tx_done     = 0u;         /* 调试计数：可直接在调试器里查看 */
static volatile uint32_t s_sx_tx_calls    = 0u;         /* StartTransmit 被调用次数 */
static volatile int8_t   s_sx_last_tx_ret = 0x7F;       /* StartTransmit 最近一次返回码 */
static volatile uint32_t s_sx_rx_ok       = 0u;
static volatile uint32_t s_sx_err_cnt     = 0u;
static volatile int8_t   s_sx_last_err    = 0;

/* ---- 每拍的一次性上报：栈溢出任务名 + SX1280 初始化失败码 ---- */
static volatile uint8_t s_sx_init_reported = 0u;   /* 只上报一次，但保留原值供排查 */

static void App_ReportPending(void)
{
    if (s_stack_ovf_flag != 0u) {
        uint8_t n = 0u;
        while ((n < (uint8_t)(configMAX_TASK_NAME_LEN - 1u)) &&
               (s_stack_ovf_task[n] != '\0')) { n++; }
        (void)UsartTx_Post(STACK_OVF_CMD, (const uint8_t *)s_stack_ovf_task, n);
        s_stack_ovf_flag = 0u;
    }

    /* 注意：这里【不能】把 s_sx_init_status 清成 SX1280_OK ——
     * 它的初值恰好就是 SX1280_OK(0)，清掉之后从外部读到的 0 无法区分
     * "初始化成功"与"失败过但已被清"，实测就因此误判过一轮。
     * 改为用独立的 reported 标志保证只上报一次。 */
    if ((s_sx_init_status != SX1280_OK) && (s_sx_init_reported == 0u)) {
        uint8_t st = (uint8_t)s_sx_init_status;
        (void)UsartTx_Post(SX_INIT_CMD, &st, 1u);
        s_sx_init_reported = 1u;
    }
}

/* ---- 驱动回调 ---- */

static void Sx_OnTxDone(void)
{
    s_sx_tx_done++;
}

static void Sx_OnError(int8_t error)
{
    s_sx_last_err = error;
    s_sx_err_cnt++;
}

static void Sx_OnRxDone(const uint8_t *data, uint8_t len, const SX1280_RxInfo_t *info)
{
    uint8_t payload[RX_REPORT_LEN];
    int8_t  status = SX1280_OK;
    uint8_t i;

    if ((data == NULL) || (info == NULL)) { return; }

    /* 空中帧校验：长度 + 2 字节包头。
     * 空口本身的完整性由 SX1280 的 LoRa 包 CRC（硬件）保证，驱动在 CRC 错时
     * 走的是 error 回调，不会进到这里。 */
    if (len != AIR_FRAME_LEN)                                { status = SX1280_ERR_SIZE; }
    else if ((data[0] != AIR_HDR0) || (data[1] != AIR_HDR1)) { status = SX1280_ERR_CMD;  }

    payload[0] = (uint8_t)status;
    for (i = 0u; i < AIR_TELEMETRY_LEN; i++) {
        payload[1u + i] = (status == SX1280_OK) ? data[AIR_HDR_LEN + i] : 0u;
    }

    /* RSSI(dBm, int16 小端) + SNR(dB, int8) */
    payload[8]  = (uint8_t)((uint16_t)info->rssi_dbm & 0xFFu);
    payload[9]  = (uint8_t)(((uint16_t)info->rssi_dbm >> 8) & 0xFFu);
    payload[10] = (uint8_t)info->snr_db;

    if (status == SX1280_OK) { s_sx_rx_ok++; } else { s_sx_err_cnt++; }

    (void)UsartTx_Post(RX_REPORT_CMD, payload, RX_REPORT_LEN);
}

/* ---- 待发空中帧投递（生产者调用，任意任务上下文安全） ---- */

bool Sx1280Tx_Post(const uint8_t *frame, uint8_t len)
{
    Sx1280TxMsg_t msg;

    if (SxTxQueue == NULL)               { return false; }
    if ((frame == NULL) || (len == 0u))  { return false; }
    if (len > AIR_FRAME_LEN)             { return false; }

    msg.len = len;
    memcpy(msg.data, frame, len);

    if (osMessageQueuePut(SxTxQueue, &msg, 0U, 0U) != osOK) { return false; }

    (void)osThreadFlagsSet(SX1280TaskHandle, BIT_SX1280_TXREQ);
    return true;
}

/* ---- SX1280 任务：驱动的唯一访问者 ---- */

static void SX1280Task(void *pvParameters)
{
#if (NODE_ROLE == NODE_ROLE_TX)
    Sx1280TxMsg_t msg;
#endif

    SX1280_RegisterCallbacks(Sx_OnTxDone, Sx_OnRxDone, Sx_OnError);

#if (NODE_ROLE == NODE_ROLE_RX)
    /* 接收端：进入连续接收。驱动在连续模式下收完一包不会退出 RX，
     * 无需在回调里重启接收。 */
    (void)SX1280_StartReceive(SX1280_RX_TIMEOUT_CONTINUOUS);
#endif

    while (1)
    {
        /* DIO1/2/3 中断唤醒；SX1280_POLL_MS 兜底扫描，
         * 防止漏掉一次中断后彻底停摆（与串口模块 worker 同一思路）。 */
        (void)osThreadFlagsWait(BIT_SX1280_IRQ | BIT_SX1280_TXREQ,
                                osFlagsWaitAny, SX1280_POLL_MS);

        /* 驱动要求：IRQ 的实质性处理必须在任务上下文完成 */
        SX1280_Process();

#if (NODE_ROLE == NODE_ROLE_TX)
        /* 空闲且有排队帧就发出去。StartTransmit 是非阻塞的，
         * 完成与否由 TX_DONE 中断 → Process → Sx_OnTxDone 通知。
         * 返回码记录到 s_sx_last_tx_ret：该函数有多处会静默失败
         * （inited==0→ERR_STATE、BUSY 超时→ERR_TIMEOUT、SPI 失败等），
         * 若直接 (void) 丢弃，现场只能看到"一帧都没发出去"而无从判断。 */
        if (SX1280_GetState() == SX1280_STATE_IDLE) {
            if (osMessageQueueGet(SxTxQueue, &msg, NULL, 0U) == osOK) {
                s_sx_tx_calls++;
                s_sx_last_tx_ret = SX1280_StartTransmit(msg.data, msg.len);
            }
        }
#endif
    }
}

static void HDC1080Task(void *pvParameters)
{
#if (NODE_ROLE == NODE_ROLE_TX)
    HDC1080_data_t data;
    uint8_t  payload[HDC_TELEMETRY_LEN];
    uint8_t  air[AIR_FRAME_LEN];
    uint16_t seq = 0u;
    int8_t   status;

    while (1)
    {
        /* TIM4 的 1s 节拍（见 Back.c 的 HAL_TIM_PeriodElapsedCallback） */
        osThreadFlagsWait(BIT_SENSOR_TICK,osFlagsWaitAny,osWaitForever);

        /* 流程走通指示：HDC1080Task 每被唤醒一次，LED 翻转一次 */
        osThreadFlagsSet(LEDTaskHandle,BIT_LED_BLINK);

        /* 0. 一次性上报（栈溢出任务名 / SX1280 初始化失败码） */
        App_ReportPending();

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

        /* 3a. 本地串口遥测（保留：便于单独验证采集链路） */
        (void)UsartTx_Post(HDC_TELEMETRY_CMD, payload, HDC_TELEMETRY_LEN);

        /* 3b. 同一份遥测加 2 字节包头后交给 SX1280Task 发送。
         *     不直接调 SX1280_StartTransmit —— 驱动不是任务安全的（见头文件说明）。 */
        air[0] = AIR_HDR0;
        air[1] = AIR_HDR1;
        memcpy(&air[AIR_HDR_LEN], payload, AIR_TELEMETRY_LEN);
        (void)Sx1280Tx_Post(air, AIR_FRAME_LEN);
    }

#else  /* NODE_ROLE_RX */

    /* 接收端：本地业务只有"SX1280 接收 → 串口上报"（见 SX1280Task）。
     * 本任务不做 I2C 采集（接收端不需要本地温湿度，也无谓占用总线），
     * 只保留 TIM4 节拍驱动的 LED 心跳，作为"系统在跑"的可视指示，
     * 同时承担一次性上报（栈溢出 / SX1280 初始化失败）。 */
    while (1)
    {
        osThreadFlagsWait(BIT_SENSOR_TICK,osFlagsWaitAny,osWaitForever);

        osThreadFlagsSet(LEDTaskHandle,BIT_LED_BLINK);

        App_ReportPending();
    }

#endif
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
    App_BootMark(3u);   /* 【临时诊断】进入本函数（= osKernelInitialize() 之后） */

    /* 1. SerialModule_Init 必须在 osKernelInitialize() 之后调用（内部会创建调度线程）。
     * main.c 的时序为 osKernelInitialize() -> MX_FREERTOS_Init() -> osKernelStart()，
     * 在此处调用刚好满足模块要求。失败说明 huart1 未就绪或 DMA 启动失败。
     *
     * ⚠⚠ 关键约束：本函数一旦执行过任何 osThreadNew/xTaskCreate（下面这行就是），
     *    到 osKernelStart() 之前的这段区间里，BASEPRI 会一直停在
     *    configMAX_SYSCALL_INTERRUPT_PRIORITY(=0x50)，把优先级 5 的 TIM10 时基屏蔽掉。
     *    因此【这段区间内绝对不能再调用任何依赖中断的阻塞函数，尤其是 HAL_Delay】，
     *    否则 uwTick 不再递增 → HAL_Delay 死等 → 整机卡死在启动阶段。
     *    凡是用到 HAL_Delay 的初始化（如 MYSX1280_Init），都必须放到 main.c 里、
     *    在本函数之前完成。详见 FreeRTOS_Task.h 中 App_SetSxInitStatus() 的说明。 */
    App_BootMark(4u);
    if (!SerialModule_Init(s_serial_ports, 1)) {Error_Handler();}

    /* 2. 队列必须先于任务创建，否则投递接口会因队列为 NULL 而丢帧。
     *    osMessageQueueNew 不依赖中断，放在这段区间里是安全的。 */
    UsartTxQueue = osMessageQueueNew(USART_TX_QUEUE_DEPTH, sizeof(UsartTxMsg_t), NULL);
    if (UsartTxQueue == NULL) {Error_Handler();}

#if (NODE_ROLE == NODE_ROLE_TX)
    SxTxQueue = osMessageQueueNew(SX1280_TX_QUEUE_DEPTH, sizeof(Sx1280TxMsg_t), NULL);
    if (SxTxQueue == NULL) {Error_Handler();}
#endif
    App_BootMark(5u);   /* 【临时诊断】队列创建之后 */

    /* 3. 创建任务 */
    LEDTaskHandle = osThreadNew(LEDTask, NULL, &LEDTask_attributes);
    HDC1080TaskHandle = osThreadNew(HDC1080Task, NULL, &HDC1080Task_attributes);
    USARTTaskHandle = osThreadNew(USARTTask, NULL, &USARTTask_attributes);
    SX1280TaskHandle = osThreadNew(SX1280Task, NULL, &SX1280Task_attributes);
}
