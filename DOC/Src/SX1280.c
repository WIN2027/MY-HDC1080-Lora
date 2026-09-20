//
// Created by WIN on 2026/9/21.
//

#include "../Inc/SX1280.h"


/**
 ******************************************************************************
 * @file    sx1280.c
 * @brief   Semtech SX1280/SX1281 2.4GHz 收发芯片驱动 —— 源文件（LoRa 专用，非阻塞）
 * @version V2.2
 * @details 基于 SX1280/SX1281 数据手册 Rev 3.2 (DS.SX1280-1.W.APP, 2020-03)
 *
 *          V2.2 修复清单（相对 V2.1）：
 *            (1) SX1280_StartReceive 在进入 SetRx 前，用 s_config 缓存重写一次
 *                包参数，消除上一次 StartTransmit 留下的 TX 长度污染：
 *                  - 隐式报头：避免 RX 按 TX 长度做定长接收导致长度失配；
 *                  - 显式报头：避免 payload_len 作为最大长度过滤门限
 *                              导致超长包被静默丢弃。
 *
 *          V2.1 修复清单（相对 V2.0）：
 *            (1) EncodePreamble 改为向上取整到最近的 m·2^n，接受 1~255；
 *            (2) 连续接收模式下 HEADER_ERROR 重新进入 SetRx(0xFFFF) 而非停机；
 *            (3) 连续模式超时成功重入后不再误报错误回调；
 *            (4) SetLoRaPacketParams 新增 update_cache 参数，
 *                避免 StartTransmit 覆盖接收端已配置的 payload_len；
 *            (5) IDLE 分支仅清 DRIVER_MASK，不再误清用户自定义 IRQ；
 *            (6) 新增 SpiLock/SpiUnlock/SpiFrameLocked，
 *                Buffer/Register API 全程持锁保护 s_txbuf/s_rxbuf；
 *            (7) SX1280_Reset 显式清零 s_spi_in_progress；
 *            (8) 新增 SX1280_SetAutoFs / SX1280_GetFrequencyError。
 ******************************************************************************
 */
#include "sx1280.h"
#include <string.h>

/*============================================================================
 * 内部常量
 *==========================================================================*/
#define SX1280_SCRATCH_SIZE      (260U)
#define SX1280_GETDATA_MAX_N     (5U)

/* 本驱动内部使用的传输缓冲（受 s_spi_in_progress 保护） */
static uint8_t s_txbuf[SX1280_SCRATCH_SIZE];
static uint8_t s_rxbuf[SX1280_SCRATCH_SIZE];

/* 驱动内部状态 */
typedef struct {
    uint8_t  inited;
    uint8_t  sleeping;
    uint8_t  packet_type;
    uint8_t  sf;
    uint8_t  bw;
    uint8_t  cr;
    uint8_t  tx_base;
    uint8_t  header_type;
    uint8_t  preamble_len;
    uint8_t  payload_len;
    uint8_t  crc_en;
    uint8_t  iq_mode;
    uint8_t  rx_continuous;
    int8_t   power_dbm;
    uint8_t  ramp_time;
    uint32_t rf_freq_hz;
} SX1280_InternalConfig_t;

static SX1280_InternalConfig_t s_config = {
    .inited        = 0U,
    .sleeping      = 0U,
    .packet_type   = SX1280_PKT_LORA,
    .sf            = SX1280_SF7,
    .bw            = SX1280_BW800,
    .cr            = SX1280_CR45,
    .tx_base       = 0x00U,
    .header_type   = SX1280_HEADER_EXPLICIT,
    .preamble_len  = 12U,
    .payload_len   = 255U,
    .crc_en        = SX1280_CRC_ON,
    .iq_mode       = SX1280_IQ_STANDARD,
    .rx_continuous = 0U,
    .power_dbm     = 13,
    .ramp_time     = SX1280_RAMP_20US,
    .rf_freq_hz    = SX1280_FREQ_MIN_HZ,
};

static SX1280_State_t s_state = SX1280_STATE_IDLE;
static volatile uint8_t s_irq_flag = 0U;
static volatile uint8_t s_spi_in_progress = 0U;
static uint8_t s_rx_data[SX1280_RX_BUFFER_SIZE];

/* 回调 */
static SX1280_TxDoneCallback_t s_tx_done_cb = NULL;
static SX1280_RxDoneCallback_t s_rx_done_cb = NULL;
static SX1280_ErrorCallback_t  s_error_cb   = NULL;

/*============================================================================
 * 内部静态函数声明
 *==========================================================================*/
static int8_t SX1280_WaitBusyLow(uint32_t timeout_ms);
static int8_t SX1280_DoWake(void);
static int8_t SX1280_SpiLock(void);
static void   SX1280_SpiUnlock(void);
static int8_t SX1280_SpiFrameLocked(const uint8_t *tx, uint8_t *rx, uint16_t len);
static int8_t SX1280_SpiFrame(const uint8_t *tx, uint8_t *rx, uint16_t len);
static int8_t SX1280_StatusCheck(uint8_t st);
static int8_t SX1280_CmdOp(uint8_t opcode, const uint8_t *param, uint8_t n);
static int8_t SX1280_GetData(uint8_t opcode, uint8_t *out, uint8_t n);
static void    SX1280_CalcTimeout(uint32_t timeout_ms, uint8_t *base, uint16_t *count);
static int8_t SX1280_ReadReg8(uint16_t addr, uint8_t *val);
static int8_t SX1280_WriteReg8(uint16_t addr, uint8_t val);
static int8_t SX1280_EncodePreamble(uint8_t len, uint8_t *out);
static int8_t SX1280_RestartRxContinuous(void);

/*============================================================================
 * 平台移植接口 —— STM32 HAL 实现
 *==========================================================================*/

static void SX1280_DelayUs(uint32_t us)
{
    volatile uint32_t loops = (uint32_t)((SX1280_CPU_MHZ / 4U) + 1U) * us;
    while (loops--) {
        __NOP();
    }
}

static void SX1280_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

static void SX1280_SetResetPin(uint8_t level)
{
    HAL_GPIO_WritePin(SX1280_RESET_GPIO_Port, SX1280_RESET_Pin,
                      (level != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t SX1280_ReadBusy(void)
{
    return (HAL_GPIO_ReadPin(SX1280_BUSY_GPIO_Port, SX1280_BUSY_Pin) == GPIO_PIN_SET)
           ? 1U : 0U;
}

/**
 * @brief 底层 SPI 全双工传输（含软件 NSS 控制）
 * @note  调用前必须已持有 s_spi_in_progress 锁（由 SpiLock/SpiFrameLocked 保证）。
 */
static int8_t SX1280_SPI_Transceive(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    HAL_StatusTypeDef st;

    if ((tx == NULL) || (len == 0U)) {
        return SX1280_ERR_PARAM;
    }

    HAL_GPIO_WritePin(SX1280_NSS_GPIO_Port, SX1280_NSS_Pin, GPIO_PIN_RESET);

    st = HAL_SPI_TransmitReceive((SPI_HandleTypeDef *)SX1280_SPI_HANDLE,
                                 (uint8_t *)tx, rx, len,
                                 SX1280_SPI_TIMEOUT_MS);

    HAL_GPIO_WritePin(SX1280_NSS_GPIO_Port, SX1280_NSS_Pin, GPIO_PIN_SET);

    if (st != HAL_OK) {
        return SX1280_ERR_SPI;
    }
    return SX1280_OK;
}

/*============================================================================
 * SPI 锁与帧处理
 *==========================================================================*/

/**
 * @brief 获取 SPI 锁（关中断，嵌套安全）
 * @return SX1280_OK 或 SX1280_ERR_BUSY
 */
static int8_t SX1280_SpiLock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_spi_in_progress != 0U) {
        __set_PRIMASK(primask);
        return SX1280_ERR_BUSY;
    }
    s_spi_in_progress = 1U;
    __set_PRIMASK(primask);
    return SX1280_OK;
}

static void SX1280_SpiUnlock(void)
{
    s_spi_in_progress = 0U;
}

static int8_t SX1280_WaitBusyLow(uint32_t timeout_ms)
{
    uint32_t waited_ms = 0U;
    uint32_t i;

    while (SX1280_ReadBusy() != 0U) {
        for (i = 0U; i < 100U; i++) {
            if (SX1280_ReadBusy() == 0U) {
                return SX1280_OK;
            }
            SX1280_DelayUs(10U);
        }
        waited_ms++;
        if (waited_ms >= timeout_ms) {
            return SX1280_ERR_TIMEOUT;
        }
    }
    return SX1280_OK;
}

static int8_t SX1280_DoWake(void)
{
    int8_t  ret;
    uint8_t probe = 0x00U;
    uint8_t dummy_rx = 0U;

    ret = SX1280_SPI_Transceive(&probe, &dummy_rx, 1U);
    if (ret != SX1280_OK) {
        return SX1280_ERR_SPI;
    }
    ret = SX1280_WaitBusyLow(SX1280_WAKE_WAIT_MS);
    if (ret == SX1280_OK) {
        s_config.sleeping = 0U;
    }
    return ret;
}

/**
 * @brief SPI 帧处理（内部版本，调用者必须已持有 SPI 锁）
 */
static int8_t SX1280_SpiFrameLocked(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    int8_t ret;

    do {
        if (s_config.sleeping != 0U) {
            ret = SX1280_DoWake();
            if (ret != SX1280_OK) {
                break;
            }
        }

        ret = SX1280_WaitBusyLow(SX1280_BUSY_WAIT_MS);
        if (ret != SX1280_OK) {
            break;
        }

        ret = SX1280_SPI_Transceive(tx, rx, len);
        if (ret != SX1280_OK) {
            break;
        }
        ret = SX1280_OK;
    } while (0);

    return ret;
}

/**
 * @brief SPI 帧处理（外部版本，自动加锁/解锁）
 */
static int8_t SX1280_SpiFrame(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    int8_t ret;

    if ((tx == NULL) || (len == 0U)) {
        return SX1280_ERR_PARAM;
    }

    ret = SX1280_SpiLock();
    if (ret != SX1280_OK) {
        return ret;
    }

    ret = SX1280_SpiFrameLocked(tx, rx, len);

    SX1280_SpiUnlock();
    return ret;
}

static int8_t SX1280_StatusCheck(uint8_t st)
{
    uint8_t cmd = SX1280_STATUS_CMD(st);
    if ((cmd == SX1280_CMDST_OK) ||
        (cmd == SX1280_CMDST_DATA_READY) ||
        (cmd == SX1280_CMDST_TX_DONE)) {
        return SX1280_OK;
    }
    if (cmd == SX1280_CMDST_TIMEOUT) {
        return SX1280_ERR_TIMEOUT;
    }
    return SX1280_ERR_CMD;
}

static int8_t SX1280_CmdOp(uint8_t opcode, const uint8_t *param, uint8_t n)
{
    uint8_t tx[16];
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

static int8_t SX1280_GetData(uint8_t opcode, uint8_t *out, uint8_t n)
{
    uint8_t tx[8];
    uint8_t rx[8];
    uint16_t i;
    uint16_t len;
    int8_t  ret;

    if ((out == NULL) || (n == 0U) || (n > SX1280_GETDATA_MAX_N)) {
        return SX1280_ERR_PARAM;
    }

    len = (uint16_t)(2U + n);
    tx[0] = opcode;
    for (i = 1U; i < len; i++) {
        tx[i] = 0x00U;
    }

    ret = SX1280_SpiFrame(tx, rx, len);
    if (ret != SX1280_OK) {
        return ret;
    }

    ret = SX1280_StatusCheck(rx[0]);
    if (ret != SX1280_OK) {
        return ret;
    }

    for (i = 0U; i < n; i++) {
        out[i] = rx[2U + i];
    }
    return SX1280_OK;
}

static void SX1280_CalcTimeout(uint32_t timeout_ms, uint8_t *base, uint16_t *count)
{
    uint32_t c;

    if (timeout_ms == SX1280_RX_TIMEOUT_CONTINUOUS) {
        *base  = SX1280_TIMEBASE_15_625US;
        *count = 0xFFFFU;
        return;
    }

    if (timeout_ms == 0U) {
        *base  = SX1280_TIMEBASE_15_625US;
        *count = 0x0000U;
        return;
    }

    if (timeout_ms > SX1280_RX_TIMEOUT_MAX_MS) {
        timeout_ms = SX1280_RX_TIMEOUT_MAX_MS;
    }

    if (timeout_ms <= 1023U) {
        *base = SX1280_TIMEBASE_15_625US;
        c = timeout_ms * 64U;
    } else if (timeout_ms <= 4095U) {
        *base = SX1280_TIMEBASE_62_5US;
        c = timeout_ms * 16U;
    } else if (timeout_ms <= 65535U) {
        *base = SX1280_TIMEBASE_1MS;
        c = timeout_ms;
    } else {
        *base = SX1280_TIMEBASE_4MS;
        c = (timeout_ms + 3U) / 4U;
    }

    if (c > 0xFFFEU) {
        c = 0xFFFEU;
    }
    *count = (uint16_t)c;
}

static int8_t SX1280_ReadReg8(uint16_t addr, uint8_t *val)
{
    return SX1280_ReadRegister(addr, val, 1U);
}

static int8_t SX1280_WriteReg8(uint16_t addr, uint8_t val)
{
    return SX1280_WriteRegister(addr, &val, 1U);
}

/**
 * @brief 前导码长度编码（手册 Table 14-50）
 *
 * 硬件形式：preamble = MANT · 2^EXP，MANT ∈ [1,15]，EXP ∈ [0,15]。
 * 本函数对任意输入 len（1~255）查找 >= len 的最小合法值，
 * 再进行编码。例如：
 *   len = 12  → mant=12, exp=0   （精确）
 *   len = 17  → mant=9,  exp=1   （= 18）
 *   len = 255 → mant=1,  exp=8   （= 256，超出 255 但硬件可表达）
 */
static int8_t SX1280_EncodePreamble(uint8_t len, uint8_t *out)
{
    uint8_t  exp, mant;
    uint8_t  best_mant = 0U;
    uint8_t  best_exp  = 0U;
    uint32_t best_val  = 0xFFFFFFFFUL;

    if ((out == NULL) || (len < 1U)) {
        return SX1280_ERR_PARAM;
    }

    for (exp = 0U; exp <= 15U; exp++) {
        for (mant = 1U; mant <= 15U; mant++) {
            uint32_t val = ((uint32_t)mant) << exp;
            if ((val >= (uint32_t)len) && (val < best_val)) {
                best_val  = val;
                best_mant = mant;
                best_exp  = exp;
            }
        }
    }

    if (best_mant == 0U) {
        return SX1280_ERR_PARAM;
    }

    *out = (uint8_t)(((best_exp & 0x0FU) << 4) | (best_mant & 0x0FU));
    return SX1280_OK;
}

/*============================================================================
 * 初始化 / 复位 / 电源模式实现
 *==========================================================================*/

int8_t SX1280_Reset(void)
{
    int8_t ret;

    SX1280_SetResetPin(0U);
    SX1280_DelayMs(SX1280_RESET_LOW_MS);
    SX1280_SetResetPin(1U);

    /* 强制清零 SPI 锁，避免上一次操作残留导致后续全部 ERR_BUSY */
    s_spi_in_progress = 0U;

    ret = SX1280_WaitBusyLow(SX1280_RESET_WAIT_MS);

    if (ret == SX1280_OK) {
        s_config.sleeping      = 0U;
        s_config.inited        = 0U;
        s_config.packet_type   = SX1280_PKT_LORA;
        s_config.sf            = SX1280_SF7;
        s_config.bw            = SX1280_BW800;
        s_config.cr            = SX1280_CR45;
        s_config.tx_base       = 0x00U;
        s_config.header_type   = SX1280_HEADER_EXPLICIT;
        s_config.preamble_len  = 12U;
        s_config.payload_len   = 255U;
        s_config.crc_en        = SX1280_CRC_ON;
        s_config.iq_mode       = SX1280_IQ_STANDARD;
        s_config.rx_continuous = 0U;
        s_state = SX1280_STATE_IDLE;
        s_irq_flag = 0U;
    }
    return ret;
}

int8_t SX1280_Init(const SX1280_Config_t *cfg)
{
    const SX1280_Config_t *c;
    int8_t ret;
    uint8_t preamble_code;

    if (cfg == NULL) {
        static const SX1280_Config_t def = SX1280_CONFIG_DEFAULT;
        c = &def;
    } else {
        c = cfg;
    }

    /* 参数校验 */
    if ((c->rf_freq_hz < SX1280_FREQ_MIN_HZ) ||
        (c->rf_freq_hz > SX1280_FREQ_MAX_HZ)) {
        return SX1280_ERR_PARAM;
    }
    if ((c->sf < SX1280_SF5) || (c->sf > SX1280_SF12)) {
        return SX1280_ERR_PARAM;
    }
    if ((c->bw != SX1280_BW1600) && (c->bw != SX1280_BW800) &&
        (c->bw != SX1280_BW400) && (c->bw != SX1280_BW200)) {
        return SX1280_ERR_PARAM;
    }
    if ((c->cr != SX1280_CR45) && (c->cr != SX1280_CR46) &&
        (c->cr != SX1280_CR47) && (c->cr != SX1280_CR48)) {
        return SX1280_ERR_PARAM;
    }
    if ((c->preamble_len < 1U) || (c->payload_len < 1U)) {
        return SX1280_ERR_PARAM;
    }
    ret = SX1280_EncodePreamble(c->preamble_len, &preamble_code);
    if (ret != SX1280_OK) {
        return ret;
    }
    (void)preamble_code;
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
    if ((c->ramp_time != SX1280_RAMP_2US) && (c->ramp_time != SX1280_RAMP_4US) &&
        (c->ramp_time != SX1280_RAMP_6US) && (c->ramp_time != SX1280_RAMP_8US) &&
        (c->ramp_time != SX1280_RAMP_10US) && (c->ramp_time != SX1280_RAMP_12US) &&
        (c->ramp_time != SX1280_RAMP_16US) && (c->ramp_time != SX1280_RAMP_20US)) {
        return SX1280_ERR_PARAM;
    }

    ret = SX1280_Reset();
    if (ret != SX1280_OK) {
        return ret;
    }

    s_config.rf_freq_hz    = c->rf_freq_hz;
    s_config.sf            = (uint8_t)c->sf;
    s_config.bw            = (uint8_t)c->bw;
    s_config.cr            = (uint8_t)c->cr;
    s_config.header_type   = c->header_type;
    s_config.preamble_len  = c->preamble_len;
    s_config.payload_len   = c->payload_len;
    s_config.crc_en        = c->crc_en;
    s_config.iq_mode       = c->iq_mode;
    s_config.power_dbm     = c->power_dbm;
    s_config.ramp_time     = c->ramp_time;
    s_config.rx_continuous = 0U;

    /* LoRa 配置序列 */
    ret = SX1280_SetStandby(SX1280_STDBY_RC);
    if (ret != SX1280_OK) return ret;

    ret = SX1280_CmdOp(SX1280_CMD_SET_PACKET_TYPE,
                       &(uint8_t){SX1280_PKT_LORA}, 1U);
    if (ret != SX1280_OK) return ret;
    s_config.packet_type = SX1280_PKT_LORA;

    ret = SX1280_SetRfFrequency(c->rf_freq_hz);
    if (ret != SX1280_OK) return ret;

    ret = SX1280_SetBufferBaseAddress(0x00U, 0x00U);
    if (ret != SX1280_OK) return ret;

    ret = SX1280_SetLoRaModulation(c->sf, c->bw, c->cr);
    if (ret != SX1280_OK) return ret;

    /* update_cache = 1：初始化时同步到 s_config 缓存 */
    ret = SX1280_SetLoRaPacketParams(c->preamble_len, c->header_type,
                                     c->payload_len, c->crc_en,
                                     c->iq_mode, 1U);
    if (ret != SX1280_OK) return ret;

    if (c->syncword != SX1280_SYNCWORD_KEEP_DEFAULT) {
        ret = SX1280_SetLoRaSyncWord(c->syncword);
        if (ret != SX1280_OK) return ret;
    }

    ret = SX1280_SetTxParams(c->power_dbm, (SX1280_RampTime_t)c->ramp_time);
    if (ret != SX1280_OK) return ret;

    ret = SX1280_SetDioIrqParams(SX1280_IRQ_DRIVER_MASK,
                                 SX1280_IRQ_DRIVER_MASK,
                                 0x0000U,
                                 0x0000U);
    if (ret != SX1280_OK) return ret;

    ret = SX1280_ClearIrqStatus(SX1280_IRQ_DRIVER_MASK);
    if (ret != SX1280_OK) return ret;

    s_config.inited = 1U;
    s_state = SX1280_STATE_IDLE;
    s_irq_flag = 0U;

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

    ret = SX1280_SetStandby(SX1280_STDBY_RC);
    if (ret != SX1280_OK) return ret;

    ret = SX1280_CmdOp(SX1280_CMD_SET_SLEEP, &sleep_cfg, 1U);
    if (ret != SX1280_OK) return ret;

    s_config.sleeping = 1U;
    if ((sleep_cfg & SX1280_SLEEP_RAM_RET) == 0U) {
        s_config.inited = 0U;
    }
    return SX1280_OK;
}

int8_t SX1280_Wake(void)
{
    int8_t ret;

    if (s_config.sleeping == 0U) {
        return SX1280_OK;
    }

    ret = SX1280_SpiLock();
    if (ret != SX1280_OK) {
        return ret;
    }
    ret = SX1280_DoWake();
    SX1280_SpiUnlock();
    return ret;
}

/*============================================================================
 * 配置接口实现
 *==========================================================================*/

int8_t SX1280_SetRfFrequency(uint32_t freq_hz)
{
    uint8_t param[3];
    uint32_t freq_reg;

    if ((freq_hz < SX1280_FREQ_MIN_HZ) || (freq_hz > SX1280_FREQ_MAX_HZ)) {
        return SX1280_ERR_PARAM;
    }

    freq_reg = (uint32_t)(((uint64_t)freq_hz * SX1280_PLL_STEP_NUM) /
                          SX1280_PLL_STEP_DEN);
    param[0] = (uint8_t)((freq_reg >> 16) & 0xFFU);
    param[1] = (uint8_t)((freq_reg >> 8)  & 0xFFU);
    param[2] = (uint8_t)(freq_reg & 0xFFU);

    s_config.rf_freq_hz = freq_hz;
    return SX1280_CmdOp(SX1280_CMD_SET_RF_FREQUENCY, param, 3U);
}

int8_t SX1280_SetLoRaModulation(SX1280_LoRaSF_t sf, SX1280_LoRaBW_t bw,
                                SX1280_LoRaCR_t cr)
{
    uint8_t param[3];
    uint8_t sf_cfg;
    int8_t ret;

    if ((sf < SX1280_SF5) || (sf > SX1280_SF12) ||
        ((bw != SX1280_BW1600) && (bw != SX1280_BW800) &&
         (bw != SX1280_BW400) && (bw != SX1280_BW200)) ||
        ((cr != SX1280_CR45) && (cr != SX1280_CR46) &&
         (cr != SX1280_CR47) && (cr != SX1280_CR48))) {
        return SX1280_ERR_PARAM;
    }

    param[0] = (uint8_t)sf;
    param[1] = (uint8_t)bw;
    param[2] = (uint8_t)cr;

    ret = SX1280_SetStandby(SX1280_STDBY_RC);
    if (ret != SX1280_OK) return ret;

    ret = SX1280_CmdOp(SX1280_CMD_SET_MODULATION_PARAMS, param, 3U);
    if (ret != SX1280_OK) return ret;

    /* 依据手册 14.4.1 节配置 SF 附加寄存器 0x925 */
    if (((uint8_t)sf == SX1280_SF5) || ((uint8_t)sf == SX1280_SF6)) {
        sf_cfg = 0x1EU;
    } else if (((uint8_t)sf == SX1280_SF7) || ((uint8_t)sf == SX1280_SF8)) {
        sf_cfg = 0x37U;
    } else {
        sf_cfg = 0x32U;
    }
    ret = SX1280_WriteReg8(SX1280_REG_SF_ADD_CFG, sf_cfg);
    if (ret != SX1280_OK) return ret;

    ret = SX1280_WriteReg8(SX1280_REG_FREQ_ERR_COMP, 0x01U);
    if (ret != SX1280_OK) return ret;

    s_config.sf = (uint8_t)sf;
    s_config.bw = (uint8_t)bw;
    s_config.cr = (uint8_t)cr;

    return SX1280_OK;
}

int8_t SX1280_SetLoRaPacketParams(uint8_t preamble_len, uint8_t header_type,
                                  uint8_t payload_len, uint8_t crc_en,
                                  uint8_t iq_mode, uint8_t update_cache)
{
    uint8_t param[7];
    uint8_t preamble_code;
    int8_t ret;

    if ((preamble_len < 1U) || (payload_len < 1U) ||
        ((header_type != SX1280_HEADER_EXPLICIT) &&
         (header_type != SX1280_HEADER_IMPLICIT)) ||
        ((crc_en != SX1280_CRC_ON) && (crc_en != SX1280_CRC_OFF)) ||
        ((iq_mode != SX1280_IQ_STANDARD) && (iq_mode != SX1280_IQ_INVERTED))) {
        return SX1280_ERR_PARAM;
    }

    ret = SX1280_EncodePreamble(preamble_len, &preamble_code);
    if (ret != SX1280_OK) return ret;

    param[0] = preamble_code;
    param[1] = header_type;
    param[2] = payload_len;
    param[3] = crc_en;
    param[4] = iq_mode;
    param[5] = 0x00U;
    param[6] = 0x00U;

    ret = SX1280_CmdOp(SX1280_CMD_SET_PACKET_PARAMS, param, 7U);
    if (ret != SX1280_OK) {
        return ret;
    }

    /* 只有显式要求才更新缓存，避免 StartTransmit 覆盖接收端 payload_len */
    if (update_cache != 0U) {
        s_config.preamble_len = preamble_len;
        s_config.header_type  = header_type;
        s_config.payload_len  = payload_len;
        s_config.crc_en       = crc_en;
        s_config.iq_mode      = iq_mode;
    }
    return SX1280_OK;
}

int8_t SX1280_SetLoRaSyncWord(uint8_t syncword)
{
    uint8_t val;
    int8_t ret;

    /* 写 MSB nibble（显式加括号，避免依赖运算符优先级） */
    ret = SX1280_ReadReg8(SX1280_REG_LORA_SYNCWORD_MSB, &val);
    if (ret != SX1280_OK) return ret;
    val = (uint8_t)((val & 0x0FU) | (((syncword >> 4) & 0x0FU) << 4));
    ret = SX1280_WriteReg8(SX1280_REG_LORA_SYNCWORD_MSB, val);
    if (ret != SX1280_OK) return ret;

    /* 写 LSB nibble */
    ret = SX1280_ReadReg8(SX1280_REG_LORA_SYNCWORD_LSB, &val);
    if (ret != SX1280_OK) return ret;
    val = (uint8_t)((val & 0x0FU) | ((syncword & 0x0FU) << 4));
    ret = SX1280_WriteReg8(SX1280_REG_LORA_SYNCWORD_LSB, val);
    if (ret != SX1280_OK) return ret;

    /* 回读校验 */
    ret = SX1280_ReadReg8(SX1280_REG_LORA_SYNCWORD_MSB, &val);
    if (ret != SX1280_OK) return ret;
    if ((val >> 4) != ((syncword >> 4) & 0x0FU)) return SX1280_ERR_CMD;

    ret = SX1280_ReadReg8(SX1280_REG_LORA_SYNCWORD_LSB, &val);
    if (ret != SX1280_OK) return ret;
    if ((val >> 4) != (syncword & 0x0FU)) return SX1280_ERR_CMD;

    return SX1280_OK;
}

int8_t SX1280_SetTxParams(int8_t power_dbm, SX1280_RampTime_t ramp_time)
{
    uint8_t param[2];

    if ((power_dbm < -18) || (power_dbm > 13)) {
        return SX1280_ERR_PARAM;
    }
    if ((ramp_time != SX1280_RAMP_2US) && (ramp_time != SX1280_RAMP_4US) &&
        (ramp_time != SX1280_RAMP_6US) && (ramp_time != SX1280_RAMP_8US) &&
        (ramp_time != SX1280_RAMP_10US) && (ramp_time != SX1280_RAMP_12US) &&
        (ramp_time != SX1280_RAMP_16US) && (ramp_time != SX1280_RAMP_20US)) {
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
        s_config.tx_base = tx_base;
    }
    return ret;
}

int8_t SX1280_SetAutoFs(uint8_t enable)
{
    uint8_t param = (enable != 0U) ? 0x01U : 0x00U;
    return SX1280_CmdOp(SX1280_CMD_SET_AUTO_FS, &param, 1U);
}

int8_t SX1280_GetFrequencyError(int32_t *freq_err_hz)
{
    uint8_t raw[3];
    int8_t  ret;
    int32_t signed_val;

    if (freq_err_hz == NULL) {
        return SX1280_ERR_PARAM;
    }

    ret = SX1280_ReadRegister(SX1280_REG_FEI_MSB, raw, 3U);
    if (ret != SX1280_OK) {
        return ret;
    }

    /* FEI 为 20bit 二进制补码，存于 24bit 空间 */
    signed_val = ((int32_t)raw[0] << 16) |
                 ((int32_t)raw[1] << 8)  |
                 ((int32_t)raw[2]);

    /* 20bit 符号扩展 */
    if ((signed_val & 0x00080000) != 0) {
        signed_val |= (int32_t)0xFFF00000;
    }

    /* FrequencyError[Hz] = 1.55 × SignedFEI / 1600 */
    *freq_err_hz = (int32_t)((1.55f * (float)signed_val) / 1600.0f);
    return SX1280_OK;
}

/*============================================================================
 * 非阻塞收发接口实现
 *==========================================================================*/

int8_t SX1280_StartTransmit(const uint8_t *data, uint8_t len)
{
    int8_t ret;

    if ((data == NULL) || (len == 0U) || (len > SX1280_PAYLOAD_MAX)) {
        return SX1280_ERR_PARAM;
    }
    if (s_config.inited == 0U) {
        return SX1280_ERR_STATE;
    }
    if (s_state != SX1280_STATE_IDLE) {
        return SX1280_ERR_BUSY;
    }

    ret = SX1280_SetStandby(SX1280_STDBY_RC);
    if (ret != SX1280_OK) return ret;

    /* update_cache = 0：临时改 TX 长度但不污染接收端 payload_len 缓存。
     * 注意：本调用会把硬件寄存器中的 payload_len 字段改成 TX 长度。
     *       下一次 StartReceive 会用 s_config 缓存重写回接收端配置
     *       （见 SX1280_StartReceive 中的 V2.2 修复），无需调用方干预。 */
    ret = SX1280_SetLoRaPacketParams(s_config.preamble_len,
                                     s_config.header_type,
                                     len,
                                     s_config.crc_en,
                                     s_config.iq_mode,
                                     0U);
    if (ret != SX1280_OK) {
        return ret;
    }

    ret = SX1280_WriteBuffer(s_config.tx_base, data, len);
    if (ret != SX1280_OK) return ret;

    ret = SX1280_ClearIrqStatus(SX1280_IRQ_DRIVER_MASK);
    if (ret != SX1280_OK) return ret;

    {
        uint8_t param[3] = {0x00U, 0x00U, 0x00U};
        ret = SX1280_CmdOp(SX1280_CMD_SET_TX, param, 3U);
        if (ret != SX1280_OK) return ret;
    }

    s_irq_flag = 0U;
    s_config.rx_continuous = 0U;
    s_state = SX1280_STATE_TX;
    return SX1280_OK;
}

int8_t SX1280_StartReceive(uint32_t timeout_ms)
{
    uint8_t base;
    uint16_t count;
    int8_t ret;

    if (s_config.inited == 0U) {
        return SX1280_ERR_STATE;
    }
    if (s_state != SX1280_STATE_IDLE) {
        return SX1280_ERR_BUSY;
    }

    ret = SX1280_SetStandby(SX1280_STDBY_RC);
    if (ret != SX1280_OK) return ret;

    /* V2.2 修复：恢复接收端包参数。
     *
     * 背景：SX1280_StartTransmit 为了不污染 s_config.payload_len 缓存，
     *       以 update_cache=0 调用了 SetLoRaPacketParams，但这会把硬件寄存器
     *       中的 payload_len 字段改成 TX 长度。若此处不还原，下一轮 RX：
     *         - 隐式报头：RX 按 TX 长度做定长接收，长度不符则收不到；
     *         - 显式报头：该字段作为"最大可接受长度"过滤（手册 7.4.2），
     *                    长度超出 TX 长度的包会被静默丢弃。
     *       因此进入 SetRx 前统一用 s_config 缓存重写一次包参数，
     *       保证 RX 行为始终与用户配置一致。 */
    ret = SX1280_SetLoRaPacketParams(s_config.preamble_len,
                                     s_config.header_type,
                                     s_config.payload_len,
                                     s_config.crc_en,
                                     s_config.iq_mode,
                                     0U);
    if (ret != SX1280_OK) return ret;

    ret = SX1280_ClearIrqStatus(SX1280_IRQ_DRIVER_MASK);
    if (ret != SX1280_OK) return ret;

    SX1280_CalcTimeout(timeout_ms, &base, &count);
    {
        uint8_t param[3];
        param[0] = base;
        param[1] = (uint8_t)((count >> 8) & 0xFFU);
        param[2] = (uint8_t)(count & 0xFFU);
        ret = SX1280_CmdOp(SX1280_CMD_SET_RX, param, 3U);
        if (ret != SX1280_OK) return ret;
    }

    s_irq_flag = 0U;
    s_config.rx_continuous = (timeout_ms == SX1280_RX_TIMEOUT_CONTINUOUS) ? 1U : 0U;
    s_state = SX1280_STATE_RX;
    return SX1280_OK;
}

int8_t SX1280_Abort(void)
{
    int8_t ret;

    if (s_config.inited == 0U) {
        return SX1280_ERR_STATE;
    }

    ret = SX1280_SetStandby(SX1280_STDBY_RC);
    (void)SX1280_ClearIrqStatus(SX1280_IRQ_DRIVER_MASK);

    s_config.rx_continuous = 0U;
    s_irq_flag = 0U;
    s_state = SX1280_STATE_IDLE;

    return ret;
}

/**
 * @brief 重新进入连续接收模式（供 HEADER_ERROR/超时恢复使用）
 *
 * @note 这里 **不需要** 恢复包参数：
 *       - 连续模式只能由 SX1280_StartReceive 进入，那里已经用 s_config
 *         恢复过寄存器；
 *       - 进入连续模式后不会再调用 StartTransmit（发送必须先 Abort），
 *         因此寄存器里的 payload_len 保持接收端配置，未被污染。
 *       如果未来支持"连续 RX 中穿插 TX"，则需要在此处补一次恢复。
 */
static int8_t SX1280_RestartRxContinuous(void)
{
    uint8_t base;
    uint16_t count;
    uint8_t param[3];

    SX1280_CalcTimeout(SX1280_RX_TIMEOUT_CONTINUOUS, &base, &count);
    param[0] = base;
    param[1] = (uint8_t)((count >> 8) & 0xFFU);
    param[2] = (uint8_t)(count & 0xFFU);

    return SX1280_CmdOp(SX1280_CMD_SET_RX, param, 3U);
}

void SX1280_Process(void)
{
    uint16_t irq;
    int8_t ret;

    if (s_irq_flag == 0U) {
        return;
    }
    s_irq_flag = 0U;
    __DSB();   /* 确保标志清零对其他上下文可见 */

    ret = SX1280_GetIrqStatus(&irq);
    if (ret != SX1280_OK) {
        s_irq_flag = 1U;
        return;
    }

    if (s_state == SX1280_STATE_TX) {
        if (irq & SX1280_IRQ_TX_DONE) {
            SX1280_ClearIrqStatus(irq & SX1280_IRQ_DRIVER_MASK);
            (void)SX1280_SetStandby(SX1280_STDBY_RC);
            s_state = SX1280_STATE_IDLE;
            if (s_tx_done_cb) s_tx_done_cb();
        } else if (irq & SX1280_IRQ_RX_TX_TIMEOUT) {
            SX1280_ClearIrqStatus(irq & SX1280_IRQ_DRIVER_MASK);
            (void)SX1280_SetStandby(SX1280_STDBY_RC);
            s_state = SX1280_STATE_IDLE;
            if (s_error_cb) s_error_cb(SX1280_ERR_TIMEOUT);
        } else {
            SX1280_ClearIrqStatus(irq & SX1280_IRQ_DRIVER_MASK);
        }
    } else if (s_state == SX1280_STATE_RX) {

        /* ----- HEADER_ERROR 处理 ----- */
        if (irq & SX1280_IRQ_HEADER_ERROR) {
            SX1280_ClearIrqStatus(irq & SX1280_IRQ_DRIVER_MASK);

            if (s_config.rx_continuous != 0U) {
                /* 连续模式：重新进入 SetRx(0xFFFF) 而非停机 */
                if (SX1280_RestartRxContinuous() != SX1280_OK) {
                    (void)SX1280_SetStandby(SX1280_STDBY_RC);
                    s_config.rx_continuous = 0U;
                    s_state = SX1280_STATE_IDLE;
                }
                if (s_error_cb) s_error_cb(SX1280_ERR_CMD);
            } else {
                (void)SX1280_SetStandby(SX1280_STDBY_RC);
                s_state = SX1280_STATE_IDLE;
                if (s_error_cb) s_error_cb(SX1280_ERR_CMD);
            }
        }
        /* ----- RX/TX 超时 ----- */
        else if (irq & SX1280_IRQ_RX_TX_TIMEOUT) {
            SX1280_ClearIrqStatus(irq & SX1280_IRQ_DRIVER_MASK);

            if (s_config.rx_continuous != 0U) {
                /* 连续模式按手册不应触发超时；若意外触发则重入 */
                if (SX1280_RestartRxContinuous() != SX1280_OK) {
                    (void)SX1280_SetStandby(SX1280_STDBY_RC);
                    s_config.rx_continuous = 0U;
                    s_state = SX1280_STATE_IDLE;
                    if (s_error_cb) s_error_cb(SX1280_ERR_TIMEOUT);
                }
                /* 重入成功时不回调错误（V2.1 修复） */
            } else {
                (void)SX1280_SetStandby(SX1280_STDBY_RC);
                s_state = SX1280_STATE_IDLE;
                if (s_error_cb) s_error_cb(SX1280_ERR_TIMEOUT);
            }
        }
        /* ----- RX_DONE ----- */
        else if (irq & SX1280_IRQ_RX_DONE) {
            if (irq & SX1280_IRQ_CRC_ERROR) {
                SX1280_ClearIrqStatus(irq & SX1280_IRQ_DRIVER_MASK);
                if (s_config.rx_continuous == 0U) {
                    (void)SX1280_SetStandby(SX1280_STDBY_RC);
                    s_state = SX1280_STATE_IDLE;
                }
                if (s_error_cb) s_error_cb(SX1280_ERR_CRC);
            } else {
                uint8_t pkt[2];
                uint8_t plen = 0U;
                uint8_t ptr = 0U;
                SX1280_RxInfo_t info;

                ret = SX1280_GetData(SX1280_CMD_GET_RX_BUFFER_STATUS, pkt, 2U);
                if (ret != SX1280_OK) {
                    SX1280_ClearIrqStatus(irq & SX1280_IRQ_DRIVER_MASK);
                    if (s_config.rx_continuous == 0U) {
                        (void)SX1280_SetStandby(SX1280_STDBY_RC);
                        s_state = SX1280_STATE_IDLE;
                    }
                    if (s_error_cb) s_error_cb(ret);
                    return;
                }
                plen = pkt[0];
                ptr  = pkt[1];

                if ((plen == 0U) && (s_config.header_type == SX1280_HEADER_IMPLICIT)) {
                    plen = s_config.payload_len;
                }
                if (plen > SX1280_RX_BUFFER_SIZE) {
                    plen = SX1280_RX_BUFFER_SIZE;
                }

                if (plen > 0U) {
                    ret = SX1280_ReadBuffer(ptr, s_rx_data, plen);
                    if (ret != SX1280_OK) {
                        SX1280_ClearIrqStatus(irq & SX1280_IRQ_DRIVER_MASK);
                        if (s_config.rx_continuous == 0U) {
                            (void)SX1280_SetStandby(SX1280_STDBY_RC);
                            s_state = SX1280_STATE_IDLE;
                        }
                        if (s_error_cb) s_error_cb(ret);
                        return;
                    }
                }

                /* 读取 RSSI/SNR，SNR<=0 时按手册修正 RSSI */
                {
                    uint8_t ps[2];
                    if (SX1280_GetPacketStatus(ps, 2U) == SX1280_OK) {
                        info.rssi_dbm = (int16_t)(-((int16_t)ps[0] >> 1));
                        info.snr_q4   = (int8_t)ps[1];
                        info.snr_db   = (int8_t)(info.snr_q4 / 4);
                        if (info.snr_q4 <= 0) {
                            info.rssi_dbm -= info.snr_db;
                        }
                    } else {
                        info.rssi_dbm = 0;
                        info.snr_q4   = 0;
                        info.snr_db   = 0;
                    }
                }

                /* 先清 IRQ，再决定是否切 STDBY，最后回调，
                   确保用户能在回调中启动下一次收发 */
                SX1280_ClearIrqStatus(irq & SX1280_IRQ_DRIVER_MASK);
                if (s_config.rx_continuous == 0U) {
                    (void)SX1280_SetStandby(SX1280_STDBY_RC);
                    s_state = SX1280_STATE_IDLE;
                }
                if (s_rx_done_cb) {
                    s_rx_done_cb(s_rx_data, plen, &info);
                }
            }
        } else {
            SX1280_ClearIrqStatus(irq & SX1280_IRQ_DRIVER_MASK);
        }
    } else {
        /* IDLE 状态：仅清 DRIVER_MASK，避免误清用户自定义 IRQ（V2.1 修复） */
        SX1280_ClearIrqStatus(irq & SX1280_IRQ_DRIVER_MASK);
        if (s_error_cb) s_error_cb(SX1280_ERR_STATE);
    }
}

void SX1280_DIO1_IRQHandler(void)
{
    s_irq_flag = 1U;
}

void SX1280_DIO2_IRQHandler(void)
{
    s_irq_flag = 1U;
}

void SX1280_DIO3_IRQHandler(void)
{
    s_irq_flag = 1U;
}

void SX1280_RegisterCallbacks(SX1280_TxDoneCallback_t tx_done,
                              SX1280_RxDoneCallback_t rx_done,
                              SX1280_ErrorCallback_t error)
{
    s_tx_done_cb = tx_done;
    s_rx_done_cb = rx_done;
    s_error_cb   = error;
}

SX1280_State_t SX1280_GetState(void)
{
    return s_state;
}

/*============================================================================
 * 状态与诊断接口实现
 *==========================================================================*/

int8_t SX1280_GetStatus(uint8_t *status)
{
    uint8_t tx;
    int8_t ret;

    if (status == NULL) return SX1280_ERR_PARAM;

    tx = SX1280_CMD_GET_STATUS;
    ret = SX1280_SpiFrame(&tx, status, 1U);
    if (ret != SX1280_OK) return ret;
    return SX1280_OK;
}

int8_t SX1280_GetIrqStatus(uint16_t *irq)
{
    uint8_t raw[2];
    int8_t ret;

    if (irq == NULL) return SX1280_ERR_PARAM;

    ret = SX1280_GetData(SX1280_CMD_GET_IRQ_STATUS, raw, 2U);
    if (ret != SX1280_OK) return ret;

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
    if (packet_type == NULL) return SX1280_ERR_PARAM;
    ret = SX1280_GetData(SX1280_CMD_GET_PACKET_TYPE, packet_type, 1U);
    if (ret == SX1280_OK) {
        s_config.packet_type = *packet_type;
    }
    return ret;
}

int8_t SX1280_GetRssiInst(uint8_t *rssi_raw)
{
    if (rssi_raw == NULL) return SX1280_ERR_PARAM;
    return SX1280_GetData(SX1280_CMD_GET_RSSI_INST, rssi_raw, 1U);
}

int8_t SX1280_GetRxBufferStatus(uint8_t *payload_len, uint8_t *start_ptr)
{
    uint8_t raw[2];
    int8_t ret;

    if ((payload_len == NULL) || (start_ptr == NULL)) {
        return SX1280_ERR_PARAM;
    }

    ret = SX1280_GetData(SX1280_CMD_GET_RX_BUFFER_STATUS, raw, 2U);
    if (ret != SX1280_OK) return ret;

    *payload_len = raw[0];
    *start_ptr   = raw[1];
    return SX1280_OK;
}

int8_t SX1280_GetPacketStatus(uint8_t *status, uint8_t n)
{
    if ((status == NULL) || (n == 0U) || (n > SX1280_GETDATA_MAX_N)) {
        return SX1280_ERR_PARAM;
    }
    return SX1280_GetData(SX1280_CMD_GET_PACKET_STATUS, status, n);
}

/**
 * @brief 从数据缓冲区读取数据（全程持锁保护 s_txbuf/s_rxbuf）
 */
int8_t SX1280_ReadBuffer(uint8_t offset, uint8_t *buf, uint8_t len)
{
    uint16_t total;
    uint16_t i;
    int8_t ret;

    if ((buf == NULL) && (len > 0U)) return SX1280_ERR_PARAM;
    if (len == 0U) return SX1280_OK;

    total = (uint16_t)(3U + len);
    if (total > SX1280_SCRATCH_SIZE) return SX1280_ERR_PARAM;

    ret = SX1280_SpiLock();
    if (ret != SX1280_OK) return ret;

    s_txbuf[0] = SX1280_CMD_READ_BUFFER;
    s_txbuf[1] = offset;
    for (i = 2U; i < total; i++) {
        s_txbuf[i] = 0x00U;
    }

    ret = SX1280_SpiFrameLocked(s_txbuf, s_rxbuf, total);
    if (ret == SX1280_OK) {
        ret = SX1280_StatusCheck(s_rxbuf[0]);
    }
    if (ret == SX1280_OK) {
        for (i = 0U; i < len; i++) {
            buf[i] = s_rxbuf[3U + i];
        }
    }

    SX1280_SpiUnlock();
    return ret;
}

int8_t SX1280_WriteBuffer(uint8_t offset, const uint8_t *buf, uint8_t len)
{
    uint16_t total;
    uint16_t i;
    int8_t ret;

    if ((buf == NULL) || (len == 0U)) return SX1280_ERR_PARAM;

    total = (uint16_t)(2U + len);
    if (total > SX1280_SCRATCH_SIZE) return SX1280_ERR_PARAM;

    ret = SX1280_SpiLock();
    if (ret != SX1280_OK) return ret;

    s_txbuf[0] = SX1280_CMD_WRITE_BUFFER;
    s_txbuf[1] = offset;
    for (i = 0U; i < len; i++) {
        s_txbuf[2U + i] = buf[i];
    }

    ret = SX1280_SpiFrameLocked(s_txbuf, s_rxbuf, total);
    if (ret == SX1280_OK) {
        ret = SX1280_StatusCheck(s_rxbuf[0]);
    }

    SX1280_SpiUnlock();
    return ret;
}

int8_t SX1280_ReadRegister(uint16_t addr, uint8_t *buf, uint8_t len)
{
    uint16_t total;
    uint16_t i;
    int8_t ret;

    if ((buf == NULL) && (len > 0U)) return SX1280_ERR_PARAM;
    if (len == 0U) return SX1280_OK;

    total = (uint16_t)(3U + len);
    if (total > SX1280_SCRATCH_SIZE) return SX1280_ERR_PARAM;

    ret = SX1280_SpiLock();
    if (ret != SX1280_OK) return ret;

    s_txbuf[0] = SX1280_CMD_READ_REGISTER;
    s_txbuf[1] = (uint8_t)((addr >> 8) & 0xFFU);
    s_txbuf[2] = (uint8_t)(addr & 0xFFU);
    for (i = 3U; i < total; i++) {
        s_txbuf[i] = 0x00U;
    }

    ret = SX1280_SpiFrameLocked(s_txbuf, s_rxbuf, total);
    if (ret == SX1280_OK) {
        ret = SX1280_StatusCheck(s_rxbuf[0]);
    }
    if (ret == SX1280_OK) {
        for (i = 0U; i < len; i++) {
            buf[i] = s_rxbuf[3U + i];
        }
    }

    SX1280_SpiUnlock();
    return ret;
}

int8_t SX1280_WriteRegister(uint16_t addr, const uint8_t *buf, uint8_t len)
{
    uint16_t total;
    uint16_t i;
    int8_t ret;

    if ((buf == NULL) || (len == 0U)) return SX1280_ERR_PARAM;

    total = (uint16_t)(3U + len);
    if (total > SX1280_SCRATCH_SIZE) return SX1280_ERR_PARAM;

    ret = SX1280_SpiLock();
    if (ret != SX1280_OK) return ret;

    s_txbuf[0] = SX1280_CMD_WRITE_REGISTER;
    s_txbuf[1] = (uint8_t)((addr >> 8) & 0xFFU);
    s_txbuf[2] = (uint8_t)(addr & 0xFFU);
    for (i = 0U; i < len; i++) {
        s_txbuf[3U + i] = buf[i];
    }

    ret = SX1280_SpiFrameLocked(s_txbuf, s_rxbuf, total);
    if (ret == SX1280_OK) {
        ret = SX1280_StatusCheck(s_rxbuf[0]);
    }

    SX1280_SpiUnlock();
    return ret;
}

int8_t MYSX1280_Init(void)
{
    SX1280_Config_t cfg = SX1280_CONFIG_DEFAULT;

    cfg.rf_freq_hz   = 2400000000UL;
    cfg.sf           = SX1280_SF7;
    cfg.bw           = SX1280_BW800;
    cfg.cr           = SX1280_CR45;
    cfg.preamble_len = 12;
    cfg.header_type  = SX1280_HEADER_EXPLICIT;
    cfg.payload_len  = 64;
    cfg.crc_en       = SX1280_CRC_ON;
    cfg.iq_mode      = SX1280_IQ_STANDARD;
    cfg.syncword     = SX1280_SYNCWORD_KEEP_DEFAULT;
    cfg.power_dbm    = 6;
    cfg.ramp_time    = SX1280_RAMP_20US;

    if (SX1280_Init(&cfg) != SX1280_OK)
    {
        /* 初始化失败处理 */
    }
    return SX1280_OK;
}

/* 文件结束 */