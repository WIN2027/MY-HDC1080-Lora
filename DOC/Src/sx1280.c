/**
 ******************************************************************************
 * @file    sx1280.c
 * @brief   Semtech SX1280/SX1281 2.4GHz 收发芯片驱动 —— 源文件
 * @details 基于 SX1280/SX1281 数据手册 Rev 3.2 (DS.SX1280-1.W.APP, 2020-03)
 *          - SPI 传输层：每帧 [操作码][参数][1 个哑字节 NOP][数据区]，
 *            数据区由帧末 n 个字节返回（手册 11.4/11.8/11.9 各命令表）；
 *            ReadRegister/ReadBuffer 读帧在地址/偏移后同样带 1 个哑字节。
 *          - BUSY 引脚：帧前等待拉低（可接收新命令），芯片睡眠时 BUSY 不会
 *            拉低 → 驱动自动执行“探测帧唤醒 + 等 BUSY 变低”（手册 9.3.3）。
 *          - LoRa 深度封装：SetPacketType → 频率 → 调制参数（含 SF 附加
 *            寄存器 0x925 与频率误差补偿寄存器 0x93C）→ 包参数 → 同步字
 *            → 发射参数 → IRQ 映射（手册 14.4.1）。
 *          - 阻塞收发：SPI 轮询 IRQ 状态寄存器（手册 11.9.2），
 *            不依赖 DIO 外部中断，返回带状态码。
 *
 * @note   1) 本驱动未使用 ISR/RTOS 信号量，所有 API 均为阻塞式，
 *            不适合在中断上下文调用；多任务并发调用同一芯片需自行加锁。
 *         2) 移植方法：在用户工程中定义同名非弱函数覆盖 SX1280_SPI_Transceive /
 *            SX1280_ReadBusy / SX1280_SetResetPin / SX1280_DelayUs /
 *            SX1280_DelayMs（占位实现见文件底部，详见 README.md）。
 ******************************************************************************
 */
#include "sx1280.h"

/*============================================================================
 * 内部常量
 *==========================================================================*/
/* 单帧最大长度：读寄存器 = 操作码1 + 地址2 + 哑1 + 数据255 = 259 字节 */
#define SX1280_SCRATCH_SIZE      (260U)

#define SX1280_IRQ_DEF_MASK      (SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE | \
                                  SX1280_IRQ_RX_TX_TIMEOUT | \
                                  SX1280_IRQ_HEADER_ERROR | SX1280_IRQ_CRC_ERROR)
#define SX1280_DIO1_DEF_MASK     (SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE | \
                                  SX1280_IRQ_RX_TX_TIMEOUT)
#define SX1280_DIO2_DEF_MASK     (SX1280_IRQ_HEADER_ERROR | SX1280_IRQ_CRC_ERROR)

/* 本驱动内部使用的传输缓冲（阻塞式单任务使用；每帧只用其一） */
static uint8_t s_txbuf[SX1280_SCRATCH_SIZE];
static uint8_t s_rxbuf[SX1280_SCRATCH_SIZE];

/* 驱动内部状态缓存 */
typedef struct {
    uint8_t inited;      /*!< Init 成功标志（配置已生效）                */
    uint8_t sleeping;    /*!< 芯片处于睡眠模式标志                       */
    uint8_t packet_type; /*!< 当前包类型（SX1280_PKT_xxx）              */
    uint8_t tx_base;     /*!< Tx 缓冲基地址（默认 0x00）                 */
    uint8_t header_type; /*!< LoRa 报头类型（显式/隐式）                 */
    uint8_t payload_len; /*!< LoRa 配置载荷长度（隐式报头固定长度用）    */
} SX1280_State_t;

static SX1280_State_t s_state = {
    .inited      = 0U,
    .sleeping    = 0U,
    .packet_type = SX1280_PKT_LORA,   /* 与 POR 后首次 Init 目标一致 */
    .tx_base     = 0x00U,
    .header_type = SX1280_HEADER_EXPLICIT,
    .payload_len = SX1280_PAYLOAD_MAX,
};

/*============================================================================
 * 内部静态函数声明
 *==========================================================================*/
static int8_t SX1280_WaitBusyLow(uint32_t timeout_ms);
static int8_t SX1280_DoWake(void);
static int8_t SX1280_SpiFrame(const uint8_t *tx, uint8_t *rx, uint16_t len);
static int8_t SX1280_StatusCheck(uint8_t st);
static int8_t SX1280_CmdOp(uint8_t opcode, const uint8_t *param, uint8_t n);
static int8_t SX1280_GetData(uint8_t opcode, uint8_t *out, uint8_t n);
static int8_t SX1280_PollIrq(uint16_t ok_bits, uint16_t err_bits,
                             uint32_t timeout_ms, uint16_t *irq_out);

/*============================================================================
 * 平台移植接口 —— 弱函数占位实现
 * （用户定义同名非弱函数后，链接时自动覆盖以下占位实现）
 *==========================================================================*/

/**
 * @brief SPI 全双工事务占位实现（用户必须替换为实际 SPI 驱动）
 * @note  协议要求：CPOL=0/CPHA=0、MSB 优先、SCK ≤ 18MHz；
 *        NSS 片选在本函数内部拉低/拉高（可由硬件 SPI 管理）；
 *        单帧最大长度 259 字节。
 * @return 0 成功；非 0 失败（驱动将返回 SX1280_ERR_SPI）
 */
SX1280_WEAK int8_t SX1280_SPI_Transceive(const uint8_t *tx, uint8_t *rx,
                                         uint16_t len)
{
    /* 占位实现：不做任何通信，直接返回失败，提示用户尚未移植 */
    (void)tx;
    (void)rx;
    (void)len;
    return SX1280_ERR_SPI;
}

/**
 * @brief BUSY 引脚电平读取占位实现（用户必须替换为实际 GPIO 读取）
 * @note  返回 1 表示芯片忙（BUSY 高）。
 */
SX1280_WEAK uint8_t SX1280_ReadBusy(void)
{
    /* 占位实现：恒返回“空闲”，移植时替换为 GPIO 读取 */
    return 0U;
}

/**
 * @brief NRESET 引脚控制占位实现（用户必须替换为实际 GPIO 控制）
 */
SX1280_WEAK void SX1280_SetResetPin(uint8_t level)
{
    /* 占位实现：空操作，移植时替换为 GPIO 输出 */
    (void)level;
}

/**
 * @brief 微秒延时占位实现（用户必须替换为实际延时函数）
 */
SX1280_WEAK void SX1280_DelayUs(uint32_t us)
{
    /* 占位实现：空延时，移植时替换为 delay_us(us) 等 */
    (void)us;
}

/**
 * @brief 毫秒延时占位实现（用户必须替换为实际延时函数）
 */
SX1280_WEAK void SX1280_DelayMs(uint32_t ms)
{
    /* 占位实现：空延时，移植时替换为 HAL_Delay(ms) / delay_ms(ms) 等 */
    (void)ms;
}

/*============================================================================
 * 内部静态函数实现
 *==========================================================================*/

/**
 * @brief 等待 BUSY 引脚拉低（芯片就绪，可接收新命令）
 * @note  以 1ms 为步进轮询；芯片睡眠时 BUSY 不会拉低，由调用方处理唤醒。
 */
static int8_t SX1280_WaitBusyLow(uint32_t timeout_ms)
{
    uint32_t waited = 0U;

    while (SX1280_ReadBusy() != 0U) {
        if (waited >= timeout_ms) {
            return SX1280_ERR_TIMEOUT;
        }
        SX1280_DelayMs(1U);
        waited++;
    }
    return SX1280_OK;
}

/**
 * @brief 执行睡眠唤醒：发一个探测帧触发 NSS 下降沿唤醒，
 *        然后等待芯片完成启动（BUSY 拉低 = 进入 STDBY_RC）。
 * @note  手册 9.3.3 节：睡眠期间任何 SPI 事务都会先拉低 NSS 唤醒芯片；
 *        唤醒启动期间收到的帧内容被忽略（接口仍在初始化），
 *        只要 BUSY 最终拉低即认为唤醒成功，后续命令正常执行。
 */
static int8_t SX1280_DoWake(void)
{
    int8_t  ret;
    uint8_t probe = 0x00U;   /* 哑操作码：即使被当成命令也不产生副作用 */

    ret = SX1280_SPI_Transceive(&probe, NULL, 1U);
    if (ret != 0) {
        return SX1280_ERR_SPI;
    }

    ret = SX1280_WaitBusyLow(SX1280_WAKE_WAIT_MS);
    if (ret == SX1280_OK) {
        s_state.sleeping = 0U;
    }
    return ret;
}

/**
 * @brief 内部 SPI 事务入口：睡眠自动唤醒 → 等 BUSY 低 → 收发 → 错误映射
 */
static int8_t SX1280_SpiFrame(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    int8_t ret;

    if ((tx == NULL) || (len == 0U)) {
        return SX1280_ERR_PARAM;
    }

    /* 1. 芯片睡眠：先唤醒 */
    if (s_state.sleeping != 0U) {
        ret = SX1280_DoWake();
        if (ret != SX1280_OK) {
            return ret;
        }
    }

    /* 2. 等 BUSY 低后再发帧 */
    ret = SX1280_WaitBusyLow(SX1280_BUSY_WAIT_MS);
    if (ret != SX1280_OK) {
        return ret;
    }

    /* 3. 全双工收发（NSS 片选由移植层内部管理） */
    ret = SX1280_SPI_Transceive(tx, rx, len);
    if (ret != 0) {
        return SX1280_ERR_SPI;
    }

    return SX1280_OK;
}

/**
 * @brief 解析状态字节中的命令处理状态（手册 Table 11-5）
 * @return SX1280_OK / SX1280_ERR_TIMEOUT / SX1280_ERR_CMD
 */
static int8_t SX1280_StatusCheck(uint8_t st)
{
    uint8_t cmd = SX1280_STATUS_CMD(st);

    if ((cmd == SX1280_CMDST_OK) ||
        (cmd == SX1280_CMDST_DATA_READY) ||
        (cmd == SX1280_CMDST_TX_DONE)) {
        return SX1280_OK;
    }
    if (cmd == SX1280_CMDST_TIMEOUT) {
        return SX1280_ERR_TIMEOUT;   /* 内部看门狗超时，需重发 */
    }
    return SX1280_ERR_CMD;           /* 处理错误 / 执行失败 */
}

/**
 * @brief 发送“操作码 + 参数”型命令（帧较短，适合全部配置命令）
 * @param opcode 操作码
 * @param param  参数区（可为 NULL 当 n==0）
 * @param n      参数长度（字节）
 */
static int8_t SX1280_CmdOp(uint8_t opcode, const uint8_t *param, uint8_t n)
{
    uint8_t tx[16];   /* 本驱动用到的配置命令参数 ≤ 8 字节 */
    uint8_t rx[16];
    uint16_t i;
    int8_t  ret;

    if (n > 10U) {
        return SX1280_ERR_PARAM;
    }

    tx[0] = opcode;
    for (i = 0; i < n; i++) {
        tx[1U + i] = (param != NULL) ? param[i] : 0x00U;
    }

    ret = SX1280_SpiFrame(tx, rx, (uint16_t)(1U + n));
    if (ret != SX1280_OK) {
        return ret;
    }

    return SX1280_StatusCheck(rx[0]);
}

/**
 * @brief 读取型命令：发送 [操作码][1 哑字节][n 个 NOP]，
 *        MISO 上数据从第 3 字节（帧下标 2）开始连续 n 字节。
 * @param opcode 操作码（GetPacketType/GetIrqStatus/GetRxBufferStatus/
 *                GetPacketStatus/GetRssiInst 等，手册 11.8/11.9）
 * @param out    输出数据区（长度 n）
 * @param n      期望返回数据字节数（≤5）
 */
static int8_t SX1280_GetData(uint8_t opcode, uint8_t *out, uint8_t n)
{
    uint8_t tx[8];
    uint8_t rx[8];
    uint16_t i;
    uint16_t len;
    int8_t  ret;

    if ((out == NULL) || (n > 5U)) {
        return SX1280_ERR_PARAM;
    }

    /* 帧 = 操作码 + 1 哑字节 + n 个 NOP */
    len = (uint16_t)(2U + n);
    tx[0] = opcode;
    for (i = 1; i < len; i++) {
        tx[i] = 0x00U;
    }

    ret = SX1280_SpiFrame(tx, rx, len);
    if (ret != SX1280_OK) {
        return ret;
    }

    for (i = 0; i < n; i++) {
        out[i] = rx[2U + i];
    }
    return SX1280_OK;
}

/**
 * @brief 轮询 IRQ 状态直到出现期望位或错误位/超时（内部收发等待核心）
 * @param ok_bits    成功位集合（出现任一即返回 SX1280_OK）
 * @param err_bits   错误位集合（出现任一即返回 SX1280_ERR_TIMEOUT，调用方
 *                   依据 irq_out 自行判定，如 CRC 错误）
 * @param timeout_ms 超时 ms；0 = 无限等待
 * @param irq_out    输出最后一次读到的 IRQ 状态
 * @return SX1280_OK / SX1280_ERR_TIMEOUT / 通信错误
 */
static int8_t SX1280_PollIrq(uint16_t ok_bits, uint16_t err_bits,
                             uint32_t timeout_ms, uint16_t *irq_out)
{
    uint32_t elapsed = 0U;
    uint16_t irq = 0U;
    int8_t  ret;

    for (;;) {
        ret = SX1280_GetIrqStatus(&irq);
        if (ret != SX1280_OK) {
            return ret;
        }
        if ((irq & err_bits) != 0U) {
            if (irq_out != NULL) {
                *irq_out = irq;
            }
            return SX1280_ERR_TIMEOUT;   /* 具体错误类型由调用方判断 irq 位 */
        }
        if ((irq & ok_bits) != 0U) {
            if (irq_out != NULL) {
                *irq_out = irq;
            }
            return SX1280_OK;
        }
        if ((timeout_ms != 0U) && (elapsed >= timeout_ms)) {
            if (irq_out != NULL) {
                *irq_out = irq;
            }
            return SX1280_ERR_TIMEOUT;
        }
        SX1280_DelayMs(1U);
        elapsed++;
    }
}

/*============================================================================
 * 内部工具：小命令封装 / 参数换算
 *==========================================================================*/

/**
 * @brief 计算 Rx/Tx 内部 RTC 超时参数（手册 Table 11-24/11-25）
 * @param timeout_ms 期望超时 ms；0 → 单包模式无超时（0x0000）
 * @param base       输出时间基准（SX1280_TimeBase_t）
 * @param count      输出 16bit 计数
 */
static void SX1280_CalcTimeout(uint32_t timeout_ms, uint8_t *base,
                               uint16_t *count)
{
    uint32_t c;

    if (timeout_ms == 0U) {
        *base  = SX1280_TIMEBASE_15_625US;
        *count = 0x0000U;              /* Rx 单包模式，无超时 */
        return;
    }
    if (timeout_ms > SX1280_RX_TIMEOUT_MAX_MS) {
        timeout_ms = SX1280_RX_TIMEOUT_MAX_MS;   /* 芯片上限 262s */
    }

    if (timeout_ms <= 1023U) {
        *base = SX1280_TIMEBASE_15_625US;        /* 64 ticks/ms */
        c = timeout_ms * 64U;
    } else if (timeout_ms <= 4095U) {
        *base = SX1280_TIMEBASE_62_5US;          /* 16 ticks/ms */
        c = timeout_ms * 16U;
    } else if (timeout_ms <= 65535U) {
        *base = SX1280_TIMEBASE_1MS;
        c = timeout_ms;
    } else {
        *base = SX1280_TIMEBASE_4MS;
        c = (timeout_ms + 3U) / 4U;              /* 向上取整 */
    }

    *count = (uint16_t)((c > 0xFFFFU) ? 0xFFFFU : c);
}

/**
 * @brief 读单字节寄存器（内部快捷方式）
 */
static int8_t SX1280_ReadReg8(uint16_t addr, uint8_t *val)
{
    return SX1280_ReadRegister(addr, val, 1U);
}

/**
 * @brief 写单字节寄存器（内部快捷方式）
 */
static int8_t SX1280_WriteReg8(uint16_t addr, uint8_t val)
{
    return SX1280_WriteRegister(addr, &val, 1U);
}

/*============================================================================
 * 初始化 / 复位 / 电源模式实现
 *==========================================================================*/

int8_t SX1280_Reset(void)
{
    int8_t ret;

    /* NRESET 拉低保持 → 释放（手册 10.1：复位后芯片校准并进入 STDBY_RC，
     * BUSY 拉低即就绪；本驱动保守拉低 1ms） */
    SX1280_SetResetPin(0U);
    SX1280_DelayMs(SX1280_RESET_LOW_MS);
    SX1280_SetResetPin(1U);

    /* 复位后 BUSY 高（启动/校准中），等待其拉低 */
    ret = SX1280_WaitBusyLow(SX1280_RESET_WAIT_MS);

    /* 硬件复位后芯片为 POR 状态，缓存全部失效 */
    if (ret == SX1280_OK) {
        s_state.sleeping    = 0U;
        s_state.inited      = 0U;
        s_state.packet_type = SX1280_PKT_LORA;
        s_state.tx_base     = 0x00U;
        s_state.header_type = SX1280_HEADER_EXPLICIT;
        s_state.payload_len = SX1280_PAYLOAD_MAX;
    }
    return ret;
}

int8_t SX1280_Init(const SX1280_Config_t *cfg)
{
    SX1280_Config_t local;
    const SX1280_Config_t *c;
    int8_t ret;

    /* 1. 取配置（NULL → 默认配置） */
    if (cfg == NULL) {
        static const SX1280_Config_t def = SX1280_CONFIG_DEFAULT;
        c = &def;
    } else {
        c = cfg;
    }

    /* 2. 参数校验 */
    if ((c->rf_freq_hz < SX1280_FREQ_MIN_HZ) ||
        (c->rf_freq_hz > SX1280_FREQ_MAX_HZ)) {
        return SX1280_ERR_PARAM;
    }
    if ((c->preamble_len < 1U) || (c->payload_len < 1U)) {
        return SX1280_ERR_PARAM;
    }
    if ((c->header_type != SX1280_HEADER_EXPLICIT) &&
        (c->header_type != SX1280_HEADER_IMPLICIT)) {
        return SX1280_ERR_PARAM;
    }
    if ((c->crc_en != SX1280_CRC_ON) && (c->crc_en != SX1280_CRC_OFF)) {
        return SX1280_ERR_PARAM;
    }
    if ((c->iq_mode != SX1280_IQ_STANDARD) &&
        (c->iq_mode != SX1280_IQ_INVERTED)) {
        return SX1280_ERR_PARAM;
    }
    if ((c->power_dbm < -18) || (c->power_dbm > 13)) {
        return SX1280_ERR_PARAM;
    }

    /* 3. 硬件复位（清除上次残留状态） */
    ret = SX1280_Reset();
    if (ret != SX1280_OK) {
        return ret;
    }

    /* 4. LoRa 配置序列（手册 14.4.1：类型 → 频率 → 缓冲 → 调制 → 包参数） */
    ret = SX1280_SetStandby(SX1280_STDBY_RC);
    if (ret != SX1280_OK) {
        return ret;
    }
    ret = SX1280_SetPacketType(SX1280_PKT_LORA);
    if (ret != SX1280_OK) {
        return ret;
    }
    ret = SX1280_SetRfFrequency(c->rf_freq_hz);
    if (ret != SX1280_OK) {
        return ret;
    }
    ret = SX1280_SetBufferBaseAddress(0x00U, 0x00U);
    if (ret != SX1280_OK) {
        return ret;
    }
    ret = SX1280_SetLoRaModulation(c->sf, c->bw, c->cr);
    if (ret != SX1280_OK) {
        return ret;
    }
    ret = SX1280_SetLoRaPacketParams(c->preamble_len, c->header_type,
                                     c->payload_len, c->crc_en, c->iq_mode);
    if (ret != SX1280_OK) {
        return ret;
    }
    if (c->syncword != 0U) {
        ret = SX1280_SetLoRaSyncWord(c->syncword);
        if (ret != SX1280_OK) {
            return ret;
        }
    }
    ret = SX1280_SetTxParams(c->power_dbm, (SX1280_RampTime_t)c->ramp_time);
    if (ret != SX1280_OK) {
        return ret;
    }

    /* 5. IRQ 使能与 DIO 映射（全 0 时采用默认集） */
    local = *c;
    if ((c->irq_mask == 0U) && (c->dio1_mask == 0U) &&
        (c->dio2_mask == 0U) && (c->dio3_mask == 0U)) {
        local.irq_mask  = SX1280_IRQ_DEF_MASK;
        local.dio1_mask = SX1280_DIO1_DEF_MASK;
        local.dio2_mask = SX1280_DIO2_DEF_MASK;
        local.dio3_mask = 0x0000U;
    }
    ret = SX1280_SetDioIrqParams(local.irq_mask, local.dio1_mask,
                                 local.dio2_mask, local.dio3_mask);
    if (ret != SX1280_OK) {
        return ret;
    }

    /* 6. 清残留 IRQ，标记就绪 */
    ret = SX1280_ClearIrqStatus(SX1280_IRQ_ALL);
    if (ret != SX1280_OK) {
        return ret;
    }

    s_state.inited      = 1U;
    s_state.header_type = c->header_type;
    s_state.payload_len = c->payload_len;
    return SX1280_OK;
}

int8_t SX1280_SetStandby(SX1280_StandbyMode_t mode)
{
    uint8_t param = (uint8_t)mode;

    if (param > SX1280_STDBY_XOSC) {
        return SX1280_ERR_PARAM;
    }
    return SX1280_CmdOp(SX1280_CMD_SET_STANDBY, &param, 1U);
}

int8_t SX1280_SetSleep(uint8_t sleep_cfg)
{
    int8_t ret;

    sleep_cfg &= (uint8_t)(SX1280_SLEEP_RAM_RET | SX1280_SLEEP_BUF_RET);

    /* 睡眠只能从 STDBY_RC 进入（手册 11.6.1） */
    ret = SX1280_SetStandby(SX1280_STDBY_RC);
    if (ret != SX1280_OK) {
        return ret;
    }

    ret = SX1280_CmdOp(SX1280_CMD_SET_SLEEP, &sleep_cfg, 1U);
    if (ret != SX1280_OK) {
        return ret;
    }

    s_state.sleeping = 1U;
    /* 不保留数据 RAM → 唤醒后配置全部丢失，需重新 Init */
    if ((sleep_cfg & SX1280_SLEEP_RAM_RET) == 0U) {
        s_state.inited = 0U;
    }
    return SX1280_OK;
}

int8_t SX1280_Wake(void)
{
    if (s_state.sleeping == 0U) {
        return SX1280_OK;   /* 未睡眠，无需唤醒 */
    }
    return SX1280_DoWake();
}

/*============================================================================
 * 配置接口实现
 *==========================================================================*/

int8_t SX1280_SetPacketType(SX1280_PacketType_t type)
{
    uint8_t t = (uint8_t)type;
    int8_t ret;

    if (t > SX1280_PKT_BLE) {
        return SX1280_ERR_PARAM;
    }

    /* 包类型切换必须在 STDBY_RC 进行（手册 11.7.1） */
    ret = SX1280_SetStandby(SX1280_STDBY_RC);
    if (ret != SX1280_OK) {
        return ret;
    }

    ret = SX1280_CmdOp(SX1280_CMD_SET_PACKET_TYPE, &t, 1U);
    if (ret == SX1280_OK) {
        s_state.packet_type = t;
    }
    return ret;
}

int8_t SX1280_SetRfFrequency(uint32_t freq_hz)
{
    uint8_t param[3];
    uint32_t freq_reg;

    if ((freq_hz < SX1280_FREQ_MIN_HZ) || (freq_hz > SX1280_FREQ_MAX_HZ)) {
        return SX1280_ERR_PARAM;
    }

    /* 频率寄存器值 = f(Hz) × 2^18 / 52e6，24bit，MSB 先发（手册 11.7.3） */
    freq_reg = (uint32_t)(((uint64_t)freq_hz * SX1280_PLL_STEP_NUM) /
                          SX1280_PLL_STEP_DEN);
    param[0] = (uint8_t)((freq_reg >> 16) & 0xFFU);
    param[1] = (uint8_t)((freq_reg >> 8)  & 0xFFU);
    param[2] = (uint8_t)(freq_reg & 0xFFU);

    return SX1280_CmdOp(SX1280_CMD_SET_RF_FREQUENCY, param, 3U);
}

int8_t SX1280_SetLoRaModulation(SX1280_LoRaSF_t sf, SX1280_LoRaBW_t bw,
                                SX1280_LoRaCR_t cr)
{
    uint8_t param[3];
    uint8_t sf_cfg;
    int8_t ret;

    /* 仅 LoRa 包类型下参数才有 SF/BW/CR 语义（手册 Table 11-57） */
    if (s_state.packet_type != SX1280_PKT_LORA) {
        ret = SX1280_SetPacketType(SX1280_PKT_LORA);
        if (ret != SX1280_OK) {
            return ret;
        }
    }

    param[0] = (uint8_t)sf;   /* 扩频因子 SF5~SF12 */
    param[1] = (uint8_t)bw;   /* 带宽              */
    param[2] = (uint8_t)cr;   /* 编码率            */

    ret = SX1280_SetStandby(SX1280_STDBY_RC);
    if (ret != SX1280_OK) {
        return ret;
    }
    ret = SX1280_CmdOp(SX1280_CMD_SET_MODULATION_PARAMS, param, 3U);
    if (ret != SX1280_OK) {
        return ret;
    }

    /* LoRa 附加配置（手册 14.4.1）：
     * - SF5/SF6   → 寄存器 0x925 = 0x1E
     * - SF7/SF8   → 寄存器 0x925 = 0x37
     * - SF9~SF12  → 寄存器 0x925 = 0x32
     * - 所有 SF   → 寄存器 0x93C = 0x01（频率误差补偿模式） */
    if (((uint8_t)sf == SX1280_SF5) || ((uint8_t)sf == SX1280_SF6)) {
        sf_cfg = 0x1EU;
    } else if (((uint8_t)sf == SX1280_SF7) || ((uint8_t)sf == SX1280_SF8)) {
        sf_cfg = 0x37U;
    } else {
        sf_cfg = 0x32U;
    }
    ret = SX1280_WriteReg8(SX1280_REG_SF_ADD_CFG, sf_cfg);
    if (ret != SX1280_OK) {
        return ret;
    }
    ret = SX1280_WriteReg8(SX1280_REG_FREQ_ERR_COMP, 0x01U);
    if (ret != SX1280_OK) {
        return ret;
    }

    return SX1280_OK;
}

int8_t SX1280_SetLoRaPacketParams(uint8_t preamble_len, uint8_t header_type,
                                  uint8_t payload_len, uint8_t crc_en,
                                  uint8_t iq_mode)
{
    uint8_t param[7];
    int8_t ret;

    if ((preamble_len < 1U) || (payload_len < 1U) ||
        ((header_type != SX1280_HEADER_EXPLICIT) &&
         (header_type != SX1280_HEADER_IMPLICIT)) ||
        ((crc_en != SX1280_CRC_ON) && (crc_en != SX1280_CRC_OFF)) ||
        ((iq_mode != SX1280_IQ_STANDARD) && (iq_mode != SX1280_IQ_INVERTED))) {
        return SX1280_ERR_PARAM;
    }

    if (s_state.packet_type != SX1280_PKT_LORA) {
        ret = SX1280_SetPacketType(SX1280_PKT_LORA);
        if (ret != SX1280_OK) {
            return ret;
        }
    }

    /* 手册 Table 11-58/14-50~54：7 参数定长帧（LoRa 用前 5 个） */
    param[0] = preamble_len;   /* packetParam1：前导码符号数          */
    param[1] = header_type;    /* packetParam2：显式 0x00 / 隐式 0x80 */
    param[2] = payload_len;    /* packetParam3：载荷长度（1~255）     */
    param[3] = crc_en;         /* packetParam4：CRC 使能 0x20 / 关 0  */
    param[4] = iq_mode;        /* packetParam5：IQ 常规 0x40 / 交换 0 */
    param[5] = 0x00U;          /* packetParam6：未使用               */
    param[6] = 0x00U;          /* packetParam7：未使用               */

    ret = SX1280_CmdOp(SX1280_CMD_SET_PACKET_PARAMS, param, 7U);
    if (ret == SX1280_OK) {
        s_state.header_type = header_type;
        s_state.payload_len = payload_len;
    }
    return ret;
}

int8_t SX1280_SetLoRaSyncWord(uint8_t syncword)
{
    uint8_t val;
    int8_t ret;

    /* 手册 14.4.1：同步字 0xXY 拆分写入两个寄存器高半字节 [7:4]，
     * 低半字节 [3:0] 需读-改-写保留 */
    ret = SX1280_ReadReg8(SX1280_REG_LORA_SYNCWORD_MSB, &val);
    if (ret != SX1280_OK) {
        return ret;
    }
    val = (uint8_t)((val & 0x0FU) | ((syncword >> 4) & 0x0FU) << 4);
    ret = SX1280_WriteReg8(SX1280_REG_LORA_SYNCWORD_MSB, val);
    if (ret != SX1280_OK) {
        return ret;
    }

    ret = SX1280_ReadReg8(SX1280_REG_LORA_SYNCWORD_LSB, &val);
    if (ret != SX1280_OK) {
        return ret;
    }
    val = (uint8_t)((val & 0x0FU) | ((syncword & 0x0FU) << 4));
    ret = SX1280_WriteReg8(SX1280_REG_LORA_SYNCWORD_LSB, val);
    if (ret != SX1280_OK) {
        return ret;
    }
    return SX1280_OK;
}

int8_t SX1280_SetTxParams(int8_t power_dbm, SX1280_RampTime_t ramp_time)
{
    uint8_t param[2];

    /* 输出功率：Pout(dBm) = -18 + power，power 0~31（手册 11.7.4） */
    if ((power_dbm < -18) || (power_dbm > 13)) {
        return SX1280_ERR_PARAM;
    }
    param[0] = (uint8_t)(power_dbm + 18);
    param[1] = (uint8_t)ramp_time;
    return SX1280_CmdOp(SX1280_CMD_SET_TX_PARAMS, param, 2U);
}

int8_t SX1280_SetDioIrqParams(uint16_t irq_mask, uint16_t dio1_mask,
                              uint16_t dio2_mask, uint16_t dio3_mask)
{
    uint8_t param[8];

    /* 手册 Table 11-74：irqMask / dio1Mask / dio2Mask / dio3Mask，
     * 每项 2 字节，高字节在前 */
    param[0] = (uint8_t)((irq_mask  >> 8) & 0xFFU);
    param[1] = (uint8_t)(irq_mask  & 0xFFU);
    param[2] = (uint8_t)((dio1_mask >> 8) & 0xFFU);
    param[3] = (uint8_t)(dio1_mask & 0xFFU);
    param[4] = (uint8_t)((dio2_mask >> 8) & 0xFFU);
    param[5] = (uint8_t)(dio2_mask & 0xFFU);
    param[6] = (uint8_t)((dio3_mask >> 8) & 0xFFU);
    param[7] = (uint8_t)(dio3_mask & 0xFFU);

    return SX1280_CmdOp(SX1280_CMD_SET_DIO_IRQ_PARAMS, param, 8U);
}

int8_t SX1280_SetBufferBaseAddress(uint8_t tx_base, uint8_t rx_base)
{
    uint8_t param[2];
    int8_t ret;

    param[0] = tx_base;
    param[1] = rx_base;

    ret = SX1280_CmdOp(SX1280_CMD_SET_BUFFER_BASE_ADDR, param, 2U);
    if (ret == SX1280_OK) {
        s_state.tx_base = tx_base;
    }
    return ret;
}

/*============================================================================
 * 数据传输接口实现（阻塞式）
 *==========================================================================*/

int8_t SX1280_Transmit(const uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    uint16_t irq = 0U;
    int8_t ret;

    if ((data == NULL) || (len == 0U)) {
        return SX1280_ERR_PARAM;
    }
    if (s_state.inited == 0U) {
        return SX1280_ERR_STATE;   /* 睡眠未保留 RAM 或未 Init */
    }

    /* 手册 14.4.2 Tx 流程 */
    ret = SX1280_SetStandby(SX1280_STDBY_RC);
    if (ret != SX1280_OK) {
        return ret;
    }
    ret = SX1280_WriteBuffer(s_state.tx_base, data, len);   /* 写载荷   */
    if (ret != SX1280_OK) {
        return ret;
    }
    ret = SX1280_ClearIrqStatus(SX1280_IRQ_ALL);            /* 清 IRQ   */
    if (ret != SX1280_OK) {
        return ret;
    }
    /* SetTx：单包模式，芯片内部不设超时（0x0000），发射完自动回 STDBY_RC */
    {
        uint8_t param[3] = {0x00U, 0x00U, 0x00U};
        ret = SX1280_CmdOp(SX1280_CMD_SET_TX, param, 3U);
        if (ret != SX1280_OK) {
            return ret;
        }
    }

    /* 轮询 TxDone / RxTxTimeout（手册 14.4.2 步骤 4） */
    ret = SX1280_PollIrq(SX1280_IRQ_TX_DONE, SX1280_IRQ_RX_TX_TIMEOUT,
                         timeout_ms, &irq);
    if (ret != SX1280_OK) {
        (void)SX1280_ClearIrqStatus(irq);   /* 清除超时/完成标志 */
        if ((irq & SX1280_IRQ_RX_TX_TIMEOUT) != 0U) {
            return SX1280_ERR_TIMEOUT;
        }
        return ret;
    }

    /* 清除 TxDone，返回成功 */
    return SX1280_ClearIrqStatus(SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_TX_TIMEOUT);
}

int8_t SX1280_Receive(uint8_t *data, uint8_t max_len, uint8_t *rx_len,
                      SX1280_RxInfo_t *info, uint32_t timeout_ms)
{
    uint8_t  pkt[2];
    uint8_t  plen;
    uint8_t  ptr;
    uint8_t  base;
    uint16_t count;
    uint16_t irq = 0U;
    int8_t  ret;

    if ((data == NULL) || (max_len == 0U)) {
        return SX1280_ERR_PARAM;
    }
    if (s_state.inited == 0U) {
        return SX1280_ERR_STATE;
    }
    if (rx_len != NULL) {
        *rx_len = 0U;
    }

    /* 手册 14.4.3 Rx 流程 */
    ret = SX1280_SetStandby(SX1280_STDBY_RC);
    if (ret != SX1280_OK) {
        return ret;
    }
    ret = SX1280_ClearIrqStatus(SX1280_IRQ_ALL);
    if (ret != SX1280_OK) {
        return ret;
    }

    /* SetRx：按期望超时折算 RTC 参数；单包模式（收完自动回 STDBY_RC） */
    SX1280_CalcTimeout(timeout_ms, &base, &count);
    {
        uint8_t param[3];
        param[0] = base;
        param[1] = (uint8_t)((count >> 8) & 0xFFU);
        param[2] = (uint8_t)(count & 0xFFU);
        ret = SX1280_CmdOp(SX1280_CMD_SET_RX, param, 3U);
        if (ret != SX1280_OK) {
            return ret;
        }
    }

    /* 等待 RxDone（含报头错误/CRC 错误/超时提前退出路径） */
    ret = SX1280_PollIrq(SX1280_IRQ_RX_DONE,
                         SX1280_IRQ_RX_TX_TIMEOUT |
                         SX1280_IRQ_HEADER_ERROR |
                         SX1280_IRQ_CRC_ERROR,
                         timeout_ms, &irq);
    if (ret != SX1280_OK) {
        if ((irq & SX1280_IRQ_RX_TX_TIMEOUT) != 0U) {
            (void)SX1280_ClearIrqStatus(irq);
            return SX1280_ERR_TIMEOUT;
        }
        if ((irq & SX1280_IRQ_HEADER_ERROR) != 0U) {
            (void)SX1280_ClearIrqStatus(irq);
            return SX1280_ERR_CMD;   /* 报头校验失败 */
        }
        if ((irq & SX1280_IRQ_CRC_ERROR) != 0U) {
            (void)SX1280_ClearIrqStatus(irq);
            return SX1280_ERR_CRC;
        }
        return ret;   /* 纯等待超时 / 通信错误 */
    }

    /* RxDone：若同时带 CRC 错误位，按 CRC 错误处理 */
    if ((irq & SX1280_IRQ_CRC_ERROR) != 0U) {
        (void)SX1280_ClearIrqStatus(irq);
        return SX1280_ERR_CRC;
    }

    /* 取收包长度与起始指针（手册 11.8.1 GetRxBufferStatus） */
    ret = SX1280_GetData(SX1280_CMD_GET_RX_BUFFER_STATUS, pkt, 2U);
    if (ret != SX1280_OK) {
        (void)SX1280_ClearIrqStatus(SX1280_IRQ_RX_DONE);
        return ret;
    }
    plen = pkt[0];
    ptr  = pkt[1];

    /* 隐式（定长）报头模式：GetRxBufferStatus 恒返回 0，
     * 使用配置的固定长度（手册 11.8.1 注释） */
    if ((plen == 0U) && (s_state.header_type == SX1280_HEADER_IMPLICIT)) {
        plen = s_state.payload_len;
    }
    if (plen > max_len) {
        (void)SX1280_ClearIrqStatus(SX1280_IRQ_RX_DONE);
        return SX1280_ERR_SIZE;   /* 用户缓冲区不足 */
    }

    /* 读取载荷（手册 11.5.2 ReadBuffer） */
    ret = SX1280_ReadBuffer(ptr, data, plen);
    if (ret != SX1280_OK) {
        (void)SX1280_ClearIrqStatus(SX1280_IRQ_RX_DONE);
        return ret;
    }

    /* 可选：LoRa 包 RSSI/SNR（手册 11.8.2 GetPacketStatus，前 2 字节） */
    if (info != NULL) {
        uint8_t ps[2];
        ret = SX1280_GetData(SX1280_CMD_GET_PACKET_STATUS, ps, 2U);
        if (ret == SX1280_OK) {
            info->rssi_dbm = (int16_t)(-((int16_t)ps[0] >> 1)); /* -raw/2 */
            info->snr_q4   = (int8_t)ps[1];
            info->snr_db   = (int8_t)(info->snr_q4 / 4);
        } else {
            info->rssi_dbm = 0;
            info->snr_q4   = 0;
            info->snr_db   = 0;
        }
    }

    /* 清 IRQ，输出长度，成功 */
    ret = SX1280_ClearIrqStatus(SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR |
                                SX1280_IRQ_HEADER_ERROR);
    if (ret != SX1280_OK) {
        return ret;
    }
    if (rx_len != NULL) {
        *rx_len = plen;
    }
    return SX1280_OK;
}

/*============================================================================
 * 状态与诊断接口实现
 *==========================================================================*/

int8_t SX1280_GetStatus(uint8_t *status)
{
    uint8_t tx;
    int8_t ret;

    if (status == NULL) {
        return SX1280_ERR_PARAM;
    }

    /* SPI：单字节帧（手册 Table 11-6），MISO 首字节即状态 */
    tx = SX1280_CMD_GET_STATUS;
    ret = SX1280_SpiFrame(&tx, status, 1U);
    if (ret != SX1280_OK) {
        return ret;
    }
    return SX1280_OK;
}

int8_t SX1280_GetIrqStatus(uint16_t *irq)
{
    uint8_t raw[2];
    int8_t ret;

    if (irq == NULL) {
        return SX1280_ERR_PARAM;
    }

    ret = SX1280_GetData(SX1280_CMD_GET_IRQ_STATUS, raw, 2U);
    if (ret != SX1280_OK) {
        return ret;
    }
    /* 返回顺序：irqStatus[15:8] 在前（手册 Table 11-76） */
    *irq = (uint16_t)(((uint16_t)raw[0] << 8) | (uint16_t)raw[1]);
    return SX1280_OK;
}

int8_t SX1280_ClearIrqStatus(uint16_t irq_mask)
{
    uint8_t param[2];

    param[0] = (uint8_t)((irq_mask >> 8) & 0xFFU);
    param[1] = (uint8_t)(irq_mask & 0xFFU);
    return SX1280_CmdOp(SX1280_CMD_CLEAR_IRQ_STATUS, param, 2U);
}

int8_t SX1280_GetPacketType(uint8_t *packet_type)
{
    int8_t ret;

    if (packet_type == NULL) {
        return SX1280_ERR_PARAM;
    }
    ret = SX1280_GetData(SX1280_CMD_GET_PACKET_TYPE, packet_type, 1U);
    if (ret == SX1280_OK) {
        s_state.packet_type = *packet_type;
    }
    return ret;
}

int8_t SX1280_GetRssiInst(uint8_t *rssi_raw)
{
    if (rssi_raw == NULL) {
        return SX1280_ERR_PARAM;
    }
    return SX1280_GetData(SX1280_CMD_GET_RSSI_INST, rssi_raw, 1U);
}

int8_t SX1280_ReadBuffer(uint8_t offset, uint8_t *buf, uint8_t len)
{
    uint16_t total;
    uint16_t i;
    int8_t ret;

    if ((buf == NULL) || (len == 0U)) {
        return SX1280_ERR_PARAM;
    }

    /* 读帧：操作码 + 偏移 + 1 哑字节 + len 个 NOP（手册 Table 11-14），
     * 数据区位于帧下标 3 起 */
    total = (uint16_t)(3U + len);
    if (total > SX1280_SCRATCH_SIZE) {
        return SX1280_ERR_PARAM;
    }
    s_txbuf[0] = SX1280_CMD_READ_BUFFER;
    s_txbuf[1] = offset;
    for (i = 2; i < total; i++) {
        s_txbuf[i] = 0x00U;
    }

    ret = SX1280_SpiFrame(s_txbuf, s_rxbuf, total);
    if (ret != SX1280_OK) {
        return ret;
    }
    for (i = 0; i < len; i++) {
        buf[i] = s_rxbuf[3U + i];
    }
    return SX1280_OK;
}

int8_t SX1280_WriteBuffer(uint8_t offset, const uint8_t *buf, uint8_t len)
{
    uint16_t total;
    uint16_t i;
    int8_t ret;

    if ((buf == NULL) || (len == 0U)) {
        return SX1280_ERR_PARAM;
    }

    /* 写帧：操作码 + 偏移 + 数据（手册 Table 11-12），无哑字节 */
    total = (uint16_t)(2U + len);
    if (total > SX1280_SCRATCH_SIZE) {
        return SX1280_ERR_PARAM;
    }
    s_txbuf[0] = SX1280_CMD_WRITE_BUFFER;
    s_txbuf[1] = offset;
    for (i = 0; i < len; i++) {
        s_txbuf[2U + i] = buf[i];
    }

    ret = SX1280_SpiFrame(s_txbuf, s_rxbuf, total);
    if (ret != SX1280_OK) {
        return ret;
    }
    return SX1280_StatusCheck(s_rxbuf[0]);
}

int8_t SX1280_ReadRegister(uint16_t addr, uint8_t *buf, uint8_t len)
{
    uint16_t total;
    uint16_t i;
    int8_t ret;

    if ((buf == NULL) || (len == 0U)) {
        return SX1280_ERR_PARAM;
    }

    /* 读帧：操作码 + 2 字节地址 + 1 哑字节 + len 个 NOP
     * （手册 Table 11-10 与注：地址后须 1 个 NOP，数据从下一 NOP 开始），
     * 数据区位于帧下标 4 起 */
    total = (uint16_t)(4U + len);
    if (total > SX1280_SCRATCH_SIZE) {
        return SX1280_ERR_PARAM;
    }
    s_txbuf[0] = SX1280_CMD_READ_REGISTER;
    s_txbuf[1] = (uint8_t)((addr >> 8) & 0xFFU);
    s_txbuf[2] = (uint8_t)(addr & 0xFFU);
    for (i = 3; i < total; i++) {
        s_txbuf[i] = 0x00U;
    }

    ret = SX1280_SpiFrame(s_txbuf, s_rxbuf, total);
    if (ret != SX1280_OK) {
        return ret;
    }
    for (i = 0; i < len; i++) {
        buf[i] = s_rxbuf[4U + i];
    }
    return SX1280_OK;
}

int8_t SX1280_WriteRegister(uint16_t addr, const uint8_t *buf, uint8_t len)
{
    uint16_t total;
    uint16_t i;
    int8_t ret;

    if ((buf == NULL) || (len == 0U)) {
        return SX1280_ERR_PARAM;
    }

    /* 写帧：操作码 + 2 字节地址 + 数据（手册 Table 11-8） */
    total = (uint16_t)(3U + len);
    if (total > SX1280_SCRATCH_SIZE) {
        return SX1280_ERR_PARAM;
    }
    s_txbuf[0] = SX1280_CMD_WRITE_REGISTER;
    s_txbuf[1] = (uint8_t)((addr >> 8) & 0xFFU);
    s_txbuf[2] = (uint8_t)(addr & 0xFFU);
    for (i = 0; i < len; i++) {
        s_txbuf[3U + i] = buf[i];
    }

    ret = SX1280_SpiFrame(s_txbuf, s_rxbuf, total);
    if (ret != SX1280_OK) {
        return ret;
    }
    return SX1280_StatusCheck(s_rxbuf[0]);
}

/* 文件结束 */
