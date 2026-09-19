/**
 ******************************************************************************
 * @file    bsp_port.c
 * @brief   SX1280 / HDC1080 硬件适配层实现（STM32F411CEU6 + HAL）
 *
 * @details 引脚与时钟取自本工程 2.4G.ioc 与 CubeMX 生成的 Core 代码，
 *          代码中一律使用 main.h 里的生成宏，不硬编码引脚号：
 *            SPI1   : PA5 = SCK, PA6 = MISO, PA7 = MOSI
 *                     NSS 由软件管理（SPI_NSS_SOFT），片选 = PA4 = SX1280NSS
 *                     （CubeMX 已配为 GPIO 推挽输出、空闲高）
 *            SX1280 : PB10 = DX1820_BUSY（模块板名 DX1820，芯片为 SX1280）
 *                     PB12 = SX1280NRESET（输出，低有效）
 *            I2C1   : PB8 = SCL, PB9 = SDA（HDC1080，7bit 地址 0x40）
 *
 * @note    SPI 时序依据 SX1280/SX1281 数据手册 Rev 3.2 表 9-1（第 60 页）：
 *            t1 NSS 下降沿 → SCK 建立时间 ≥ 25 ns
 *            t2 SCK 周期 ≥ 55 ns（即 SCK ≤ 18.18 MHz）
 *            t3 SCK 高电平 ≥ 25 ns
 *            t8 SCK → NSS 上升沿保持时间 ≥ 25 ns
 *            t9 NSS 高电平时间 ≥ 100 ns
 *            t10 睡眠唤醒后 NSS 下降沿 → SCK ≥ 125 µs（或用 ≥2 µs 的 NSS 低脉冲，
 *                再等 BUSY 拉低，见手册 9.3.3）
 *          手册 9.3 节同时要求："事务期间不得抬高 NSS"，因此片选在
 *          HAL_SPI_TransmitReceive() 前后各翻转一次，全程保持低电平。
 *          本工程 SPI1 = PCLK2(100 MHz) / 8 = 12.5 MHz（周期 80 ns）满足 t2；
 *          NSS 低电平脉宽由 BSP_NSS_PULSE_US 保证 ≥ 2 µs，使睡眠唤醒路径
 *          在无需改动驱动的前提下也符合手册 9.3.3 的第二种方法。
 *
 * @note    本文件不包含 sx1280.h / hdc1080.h，原因见 bsp_port.h 的 @warning。
 ******************************************************************************
 */
#include "bsp_port.h"

#include <stddef.h>

#include "main.h"
#include "spi.h"
#include "i2c.h"
#include "cmsis_os2.h"
#include "stm32f4xx_hal_gpio.h"
/*============================================================================
 * 内部变量
 *==========================================================================*/
/** @brief DWT 周期计数器就绪标志（微秒延时基础） */
static uint8_t s_dwt_ready = 0U;

/**
 * @brief 丢弃用接收缓冲：驱动唤醒探测帧会以 rx = NULL 调用
 *        （sx1280.c: SX1280_DoWake → SX1280_SPI_Transceive(&probe, NULL, 1)），
 *        而 HAL_SPI_TransmitReceive() 要求提供接收指针，故用本缓冲兜底。
 * @note  驱动为阻塞式单线程使用，此处静态缓冲无并发风险。
 */
static uint8_t s_spi_sink[BSP_SX1280_FRAME_MAX];

/**
 * @brief I2C 引脚定义（用于总线恢复阶段的 GPIO 手动时钟）
 * @note  CubeMX 没有为 I2C 引脚生成标签宏，这里按 2.4G.ioc 的配置写出：
 *        PB8 = I2C1_SCL、PB9 = I2C1_SDA（AF4、开漏、无内部上拉，外部 4.7k 上拉）。
 *        若硬件接线改动，请同步修改这三处宏。
 */
#define BSP_I2C_SCL_PORT        GPIOB
#define BSP_I2C_SCL_PIN         GPIO_PIN_8
#define BSP_I2C_SDA_PORT        GPIOB
#define BSP_I2C_SDA_PIN         GPIO_PIN_9
#define BSP_I2C_BIT_US          (5U)    /*!< 手动时钟半周期（100kHz 对应 5µs） */

/** @brief 最近一次 I2C 调用的诊断快照 */
static BSP_I2cDiag_t s_i2c_diag;

/*============================================================================
 * 内部辅助函数
 *==========================================================================*/

/**
 * @brief 微秒级忙等延时（DWT 周期计数器）
 * @note  DWT->CYCCNT 为 32 位，100 MHz 下约 42.9 s 回绕一次；
 *        下面的无符号差值比较可正确处理回绕。
 */
static void bsp_delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t ticks;

    if (us == 0U) {
        return;
    }
    if (s_dwt_ready == 0U) {
        HAL_Delay(1U);              /* 未初始化 DWT 时的退路 */
        return;
    }

    ticks = us * (SystemCoreClock / 1000000U);   /* 100 MHz → 100 tick/µs */
    start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
        /* 忙等 */
    }
}

/**
 * @brief 记录一次 HAL I2C 调用的结果（供故障上报定位）
 */
static void bsp_i2c_record(HAL_StatusTypeDef st)
{
    s_i2c_diag.hal_status = (uint8_t)st;
    s_i2c_diag.error_code = (uint8_t)(hi2c1.ErrorCode & 0xFFU);
    s_i2c_diag.state      = (uint8_t)(hi2c1.State & 0xFFU);
}

/**
 * @brief 毫秒延时：调度器已运行时让出 CPU（osDelay），否则用 HAL 忙等
 * @note  SX1280 驱动的收发轮询以 1 ms 为步进（sx1280.c SX1280_PollIrq），
 *        在任务上下文使用 osDelay 可避免长时间独占 CPU。
 */
static void bsp_delay_ms(uint32_t ms)
{
    if (ms == 0U) {
        return;
    }
    if (osKernelGetState() == osKernelRunning) {
        (void)osDelay(ms);
    } else {
        HAL_Delay(ms);
    }
}

/*============================================================================
 * BSP 初始化
 *==========================================================================*/
void BSP_Port_Init(void)
{
    /* 使能 DWT 周期计数器：Cortex-M4 的 DWT->CYCCNT 在 100 MHz 下 1 计数 = 10 ns */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_dwt_ready = 1U;
}

/*============================================================================
 * SX1280 移植接口实现
 *==========================================================================*/

/**
 * @brief 一次完整 SPI 全双工事务：NSS 拉低 → 收发 len 字节 → NSS 拉高
 * @note  片选时序见文件头注释（手册表 9-1 的 t1/t8/t9 与 9.3 节要求）。
 */
int8_t SX1280_SPI_Transceive(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_StatusTypeDef st;
    uint8_t          *rxbuf = rx;

    if ((tx == NULL) || (len == 0U) || (len > BSP_SX1280_FRAME_MAX)) {
        return BSP_SX1280_ERR_SPI;
    }
    if (rxbuf == NULL) {
        rxbuf = s_spi_sink;         /* 唤醒探测帧：丢弃 MISO 数据 */
    }

    /* 片选拉低：t1 要求 NSS↓ → SCK ≥25ns，tPulse 要求低电平 ≥2µs（睡眠唤醒） */
    HAL_GPIO_WritePin(SX1280NSS_GPIO_Port, SX1280NSS_Pin, GPIO_PIN_RESET);
    bsp_delay_us(BSP_NSS_PULSE_US);

    /* 全双工收发：期间 NSS 必须保持低电平（手册 9.3 节） */
    st = HAL_SPI_TransmitReceive(&hspi1, tx, rxbuf, len, BSP_SPI_TIMEOUT_MS);

    /* 片选拉高：t8 要求 SCK 结束 → NSS↑ ≥25ns（HAL 返回时已满足），
     * t9 要求两次事务之间 NSS 高电平 ≥100ns（软件开销已满足） */
    HAL_GPIO_WritePin(SX1280NSS_GPIO_Port, SX1280NSS_Pin, GPIO_PIN_SET);

    return (st == HAL_OK) ? BSP_RET_OK : BSP_SX1280_ERR_SPI;
}

/**
 * @brief 读 BUSY 引脚（高 = 芯片忙，不可下命令）
 * @note  引脚标签为 DX1820_BUSY（模块板名 DX1820，实际芯片 SX1280），
 *        常规查询用轮询方式，EXTI 下降沿中断仍由 stm32f4xx_it.c 保留。
 */
uint8_t SX1280_ReadBusy(void)
{
    return (HAL_GPIO_ReadPin(DX1820_BUSY_GPIO_Port, DX1820_BUSY_Pin)
            == GPIO_PIN_SET) ? 1U : 0U;
}

/**
 * @brief 控制 NRESET 引脚（低有效）
 * @note  上电时 CubeMX 生成的 gpio.c 把 PB12 初始化为低电平（= 复位保持），
 *        驱动的 SX1280_Init() → SX1280_Reset() 会执行"拉低 ≥1ms → 拉高 →
 *        等 BUSY 变低"的完整复位时序，因此上电电平为低不影响功能。
 */
void SX1280_SetResetPin(uint8_t level)
{
    HAL_GPIO_WritePin(SX1280NRESET_GPIO_Port, SX1280NRESET_Pin,
                      (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief 微秒级忙等延时（DWT 周期计数器）
 * @note  实现见 bsp_delay_us()；保留本函数以覆盖 sx1280 驱动的弱声明。
 */
void SX1280_DelayUs(uint32_t us)
{
    bsp_delay_us(us);
}

/**
 * @brief 毫秒延时（SX1280 驱动用）
 */
void SX1280_DelayMs(uint32_t ms)
{
    bsp_delay_ms(ms);
}

/*============================================================================
 * I2C 诊断与总线恢复实现
 *==========================================================================*/

void BSP_I2C_GetDiag(BSP_I2cDiag_t *diag)
{
    if (diag == NULL) {
        return;
    }
    *diag = s_i2c_diag;
}

void BSP_I2C_BusRecover(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t          i;

    /* 1) SCL 切为开漏输出并置高；SDA 释放为输入（由外部 4.7k 上拉拉高） */
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Pin   = BSP_I2C_SCL_PIN;
    HAL_GPIO_Init(BSP_I2C_SCL_PORT, &gpio);
    HAL_GPIO_WritePin(BSP_I2C_SCL_PORT, BSP_I2C_SCL_PIN, GPIO_PIN_SET);

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pin  = BSP_I2C_SDA_PIN;
    HAL_GPIO_Init(BSP_I2C_SDA_PORT, &gpio);

    /* 2) 若从机把 SDA 拉死，补 9 个时钟让它把剩余位走完并释放总线 */
    for (i = 0U; i < 9U; i++) {
        HAL_GPIO_WritePin(BSP_I2C_SCL_PORT, BSP_I2C_SCL_PIN, GPIO_PIN_RESET);
        bsp_delay_us(BSP_I2C_BIT_US);
        HAL_GPIO_WritePin(BSP_I2C_SCL_PORT, BSP_I2C_SCL_PIN, GPIO_PIN_SET);
        bsp_delay_us(BSP_I2C_BIT_US);
    }

    /* 3) 补一个 STOP 条件：SCL 为高时 SDA 由低变高 */
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pin  = BSP_I2C_SDA_PIN;
    HAL_GPIO_Init(BSP_I2C_SDA_PORT, &gpio);
    HAL_GPIO_WritePin(BSP_I2C_SDA_PORT, BSP_I2C_SDA_PIN, GPIO_PIN_RESET);
    bsp_delay_us(BSP_I2C_BIT_US);
    HAL_GPIO_WritePin(BSP_I2C_SCL_PORT, BSP_I2C_SCL_PIN, GPIO_PIN_SET);
    bsp_delay_us(BSP_I2C_BIT_US);
    HAL_GPIO_WritePin(BSP_I2C_SDA_PORT, BSP_I2C_SDA_PIN, GPIO_PIN_SET);
    bsp_delay_us(BSP_I2C_BIT_US);

    /* 4) 复位 I2C 外设（清掉可能卡住的 BUSY/状态机），再重新初始化。
     *    HAL_I2C_Init() 内部回调 HAL_I2C_MspInit()，会重新使能 I2C1 时钟并把
     *    PB8/PB9 恢复为 AF4 开漏 —— 与 CubeMX 生成的配置一致。 */
    (void)HAL_I2C_DeInit(&hi2c1);

    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        s_i2c_diag.hal_status = (uint8_t)HAL_ERROR;
        s_i2c_diag.error_code = (uint8_t)(hi2c1.ErrorCode & 0xFFU);
        s_i2c_diag.state      = (uint8_t)(hi2c1.State & 0xFFU);
        return;
    }
    bsp_i2c_record(HAL_OK);
}

/*============================================================================
 * HDC1080 移植接口实现
 *==========================================================================*/

/**
 * @brief I2C 写事务
 * @note  len == 0 表示"仅写指针字节"（触发测量）：
 *        START → W地址 → 指针字节 → STOP；
 *        驱动传入的是 7bit 地址（0x40），HAL 需要 8bit（0x80），此处 <<1。
 */
int8_t HDC1080_I2C_Write(uint8_t dev_addr, uint8_t reg_addr,
                         const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st;
    uint16_t          addr8 = (uint16_t)((uint16_t)dev_addr << 1);

    if (len == 0U) {
        /* 仅指针字节：HAL_I2C_Mem_Write 要求 Size ≥ 1，故用 Master_Transmit */
        st = HAL_I2C_Master_Transmit(&hi2c1, addr8, &reg_addr, 1U,
                                     BSP_I2C_TIMEOUT_MS);
    } else {
        if (buf == NULL) {
            bsp_i2c_record(HAL_ERROR);
            return BSP_HDC1080_ERR_I2C;
        }
        st = HAL_I2C_Mem_Write(&hi2c1, addr8, (uint16_t)reg_addr,
                               I2C_MEMADD_SIZE_8BIT, (uint8_t *)(uintptr_t)buf,
                               len, BSP_I2C_TIMEOUT_MS);
    }
    bsp_i2c_record(st);

    return (st == HAL_OK) ? BSP_RET_OK : BSP_HDC1080_ERR_I2C;
}

/**
 * @brief I2C 读事务（指针写 → 重复起始 → 连续读 len 字节）
 * @note  与 HAL_I2C_Mem_Read 语义一致：START → W地址 → 指针字节 →
 *        (RESTART) → R地址 → 读 len 字节（末字节 NACK）→ STOP。
 */
int8_t HDC1080_I2C_Read(uint8_t dev_addr, uint8_t reg_addr,
                        uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st;
    uint16_t          addr8 = (uint16_t)((uint16_t)dev_addr << 1);

    if ((buf == NULL) || (len == 0U)) {
        bsp_i2c_record(HAL_ERROR);
        return BSP_HDC1080_ERR_I2C;
    }

    st = HAL_I2C_Mem_Read(&hi2c1, addr8, (uint16_t)reg_addr,
                          I2C_MEMADD_SIZE_8BIT, buf, len, BSP_I2C_TIMEOUT_MS);
    bsp_i2c_record(st);

    return (st == HAL_OK) ? BSP_RET_OK : BSP_HDC1080_ERR_I2C;
}

/**
 * @brief 毫秒延时（HDC1080 驱动等待转换用，14bit 顺序采集约 14ms）
 */
void HDC1080_DelayMs(uint32_t ms)
{
    bsp_delay_ms(ms);
}

/* 文件结束 */