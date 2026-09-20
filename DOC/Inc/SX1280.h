//
// Created by WIN on 2026/9/21.
//

#ifndef INC_2_4G_SX1280_H
#define INC_2_4G_SX1280_H

/**
 ******************************************************************************
 * @file    sx1280.h
 * @brief   Semtech SX1280/SX1281 2.4GHz 收发芯片驱动 —— 头文件（LoRa 专用，非阻塞）
 * @version V2.2
 * @details 基于 SX1280/SX1281 数据手册 Rev 3.2 (DS.SX1280-1.W.APP, 2020-03)
 *          - 仅深度封装 LoRa 模式。
 *          - 非阻塞设计：StartTransmit/StartReceive 启动后立即返回，
 *            由 DIO1~3 中断触发 SX1280_Process() 在主循环中完成状态处理。
 *          - SPI / BUSY / NRESET / 延时全部直接使用 STM32 HAL 库。
 *
 * @note    V2.2 修复清单（相对 V2.1）：
 *          (1) SX1280_StartReceive 在进入 SetRx 前，用 s_config 缓存重写一次
 *              包参数（前导码/报头/载荷长度/CRC/IQ），消除上一次
 *              SX1280_StartTransmit 留下的 TX 长度污染问题：
 *                - 隐式报头：避免 RX 按 TX 长度做定长接收导致长度失配；
 *                - 显式报头：避免 payload_len 作为最大长度过滤门限
 *                            导致超长包被静默丢弃。
 *              用户不再需要在 TX 后手动重设 payload_len。
 *
 * @note    V2.1 修复清单（相对 V2.0）：
 *          (1) EncodePreamble 改为向上取整到最近的 m·2^n，接受 1~255；
 *          (2) 连续接收模式下 HEADER_ERROR 会重新进入 SetRx(0xFFFF)；
 *          (3) 连续模式超时成功重入后不再误报错误回调；
 *          (4) SetLoRaPacketParams 新增 update_cache 参数，
 *              显式区分"仅写寄存器"与"同时更新缓存"；
 *          (5) IDLE 分支仅清 DRIVER_MASK，不再误清用户自定义 IRQ；
 *          (6) 新增 SpiLock/SpiUnlock/SpiFrameLocked，
 *              Buffer/Register API 全程持锁保护共享 s_txbuf/s_rxbuf；
 *          (7) SX1280_Reset 显式清零 s_spi_in_progress；
 *          (8) 新增 SX1280_SetAutoFs / SX1280_GetFrequencyError。
 ******************************************************************************
 */
#ifndef SX1280_H
#define SX1280_H

#include <stdint.h>
#include <stddef.h>
#include "main.h"
#include "spi.h"
#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * 0. 硬件抽象层（HAL）配置 —— 用户必须按实际硬件修改
 *==========================================================================*/
#ifndef SX1280_SPI_HANDLE
#define SX1280_SPI_HANDLE        (&hspi1)
#endif
#ifndef SX1280_NSS_GPIO_Port
#define SX1280_NSS_GPIO_Port     GPIOA
#endif
#ifndef SX1280_NSS_Pin
#define SX1280_NSS_Pin           GPIO_PIN_4
#endif
#ifndef SX1280_BUSY_GPIO_Port
#define SX1280_BUSY_GPIO_Port    GPIOB
#endif
#ifndef SX1280_BUSY_Pin
#define SX1280_BUSY_Pin          GPIO_PIN_10
#endif
#ifndef SX1280_RESET_GPIO_Port
#define SX1280_RESET_GPIO_Port   GPIOB
#endif
#ifndef SX1280_RESET_Pin
#define SX1280_RESET_Pin         GPIO_PIN_12
#endif
#ifndef SX1280_SPI_TIMEOUT_MS
#define SX1280_SPI_TIMEOUT_MS    (100U)
#endif
#ifndef SX1280_CPU_MHZ
#define SX1280_CPU_MHZ           (100U)
#endif

/*============================================================================
 * 1. 通用常量
 *==========================================================================*/
#define SX1280_SPI_MAX_FREQ_HZ  (18000000UL)
#define SX1280_PAYLOAD_MAX      (255U)
#define SX1280_RX_BUFFER_SIZE   (255U)

/* 频率合成：PLL 步进 = 52MHz / 2^18 ≈ 198.36Hz */
#define SX1280_PLL_STEP_NUM     (262144ULL)
#define SX1280_PLL_STEP_DEN     (52000000ULL)

#define SX1280_FREQ_MIN_HZ      (2400000000UL)
#define SX1280_FREQ_MAX_HZ      (2500000000UL)

/* RTC 超时上限：4ms × 65534 ≈ 262.13s */
#define SX1280_RX_TIMEOUT_MAX_MS (262000UL)

/* 接收超时特殊值 */
#define SX1280_RX_TIMEOUT_SINGLE      (0UL)
#define SX1280_RX_TIMEOUT_CONTINUOUS  (0xFFFFFFFFUL)

/* 默认等待超时（保守值） */
#define SX1280_RESET_WAIT_MS     (200U)
#define SX1280_RESET_LOW_MS      (1U)
#define SX1280_WAKE_WAIT_MS      (50U)
#define SX1280_BUSY_WAIT_MS      (100U)

/*============================================================================
 * 2. SPI 命令操作码（手册第 12 章）
 *==========================================================================*/
#define SX1280_CMD_GET_STATUS          (0xC0U)
#define SX1280_CMD_GET_PACKET_TYPE     (0x03U)
#define SX1280_CMD_WRITE_REGISTER      (0x18U)
#define SX1280_CMD_READ_REGISTER       (0x19U)
#define SX1280_CMD_WRITE_BUFFER        (0x1AU)
#define SX1280_CMD_READ_BUFFER         (0x1BU)
#define SX1280_CMD_SET_SLEEP           (0x84U)
#define SX1280_CMD_SET_STANDBY         (0x80U)
#define SX1280_CMD_SET_FS              (0xC1U)
#define SX1280_CMD_SET_TX              (0x83U)
#define SX1280_CMD_SET_RX              (0x82U)
#define SX1280_CMD_SET_RX_DUTY_CYCLE   (0x94U)
#define SX1280_CMD_SET_CAD             (0xC5U)
#define SX1280_CMD_SET_TX_CONT_WAVE    (0xD1U)
#define SX1280_CMD_SET_TX_CONT_PREAMBLE (0xD2U)
#define SX1280_CMD_SET_AUTO_FS         (0x9EU)
#define SX1280_CMD_SET_AUTO_TX         (0x98U)
#define SX1280_CMD_SET_LONG_PREAMBLE   (0x9BU)
#define SX1280_CMD_SET_PACKET_TYPE     (0x8AU)
#define SX1280_CMD_SET_MODULATION_PARAMS (0x8BU)
#define SX1280_CMD_SET_PACKET_PARAMS   (0x8CU)
#define SX1280_CMD_SET_RF_FREQUENCY    (0x86U)
#define SX1280_CMD_SET_TX_PARAMS       (0x8EU)
#define SX1280_CMD_SET_CAD_PARAMS      (0x88U)
#define SX1280_CMD_SET_BUFFER_BASE_ADDR (0x8FU)
#define SX1280_CMD_SET_DIO_IRQ_PARAMS  (0x8DU)
#define SX1280_CMD_GET_IRQ_STATUS      (0x15U)
#define SX1280_CMD_CLEAR_IRQ_STATUS    (0x97U)
#define SX1280_CMD_GET_RX_BUFFER_STATUS (0x17U)
#define SX1280_CMD_GET_PACKET_STATUS   (0x1DU)
#define SX1280_CMD_GET_RSSI_INST       (0x1FU)
#define SX1280_CMD_SET_REGULATOR_MODE  (0x96U)

/*============================================================================
 * 3. 状态字节定义（手册 Table 11-5）
 *==========================================================================*/
#define SX1280_STATUS_MODE_MASK   (0xE0U)
#define SX1280_STATUS_CMD_MASK    (0x1CU)
#define SX1280_STATUS_MODE(s)     ((uint8_t)(((s) & SX1280_STATUS_MODE_MASK) >> 5))
#define SX1280_STATUS_CMD(s)      ((uint8_t)(((s) & SX1280_STATUS_CMD_MASK) >> 2))

#define SX1280_MODE_STDBY_RC      (0x02U)
#define SX1280_MODE_STDBY_XOSC    (0x03U)
#define SX1280_MODE_FS            (0x04U)
#define SX1280_MODE_RX            (0x05U)
#define SX1280_MODE_TX            (0x06U)

#define SX1280_CMDST_OK           (0x01U)
#define SX1280_CMDST_DATA_READY   (0x02U)
#define SX1280_CMDST_TIMEOUT      (0x03U)
#define SX1280_CMDST_PROC_ERR     (0x04U)
#define SX1280_CMDST_FAIL         (0x05U)
#define SX1280_CMDST_TX_DONE      (0x06U)

/*============================================================================
 * 4. IRQ 寄存器位定义（手册 Table 11-73）
 *==========================================================================*/
#define SX1280_IRQ_TX_DONE              (0x0001U)
#define SX1280_IRQ_RX_DONE              (0x0002U)
#define SX1280_IRQ_SYNC_WORD_VALID      (0x0004U)
#define SX1280_IRQ_SYNC_WORD_ERROR      (0x0008U)
#define SX1280_IRQ_HEADER_VALID         (0x0010U)
#define SX1280_IRQ_HEADER_ERROR         (0x0020U)
#define SX1280_IRQ_CRC_ERROR            (0x0040U)
#define SX1280_IRQ_RANGING_SLAVE_DONE   (0x0080U)
#define SX1280_IRQ_RANGING_SLAVE_DISCARD (0x0100U)
#define SX1280_IRQ_RANGING_MASTER_VALID (0x0200U)
#define SX1280_IRQ_RANGING_MASTER_TIMEOUT (0x0400U)
#define SX1280_IRQ_RANGING_SLAVE_VALID  (0x0800U)
#define SX1280_IRQ_CAD_DONE             (0x1000U)
#define SX1280_IRQ_CAD_DETECTED         (0x2000U)
#define SX1280_IRQ_RX_TX_TIMEOUT        (0x4000U)
#define SX1280_IRQ_PREAMBLE_DETECTED    (0x8000U)
#define SX1280_IRQ_ALL                  (0xFFFFU)

/* 本驱动关心的 IRQ 位 */
#define SX1280_IRQ_DRIVER_MASK          (SX1280_IRQ_TX_DONE | \
                                         SX1280_IRQ_RX_DONE | \
                                         SX1280_IRQ_RX_TX_TIMEOUT | \
                                         SX1280_IRQ_HEADER_ERROR | \
                                         SX1280_IRQ_CRC_ERROR)

/*============================================================================
 * 5. 寄存器地址（手册第 13 章）
 *==========================================================================*/
#define SX1280_REG_SF_ADD_CFG        (0x0925U)
#define SX1280_REG_FEI_MSB           (0x0954U)   /*!< FEI[23:16]（20bit 有效） */
#define SX1280_REG_FEI_MID           (0x0955U)   /*!< FEI[15:8] */
#define SX1280_REG_FEI_LSB           (0x0956U)   /*!< FEI[7:0] */
#define SX1280_REG_FREQ_ERR_COMP     (0x093CU)
#define SX1280_REG_LORA_SYNCWORD_MSB (0x0944U)
#define SX1280_REG_LORA_SYNCWORD_LSB (0x0945U)
#define SX1280_REG_RX_PAYLOAD_LEN    (0x0901U)

/*============================================================================
 * 6. 枚举与位域（仅 LoRa）
 *==========================================================================*/

/* --- 包类型（仅用于兼容，实际只用 LoRa） --- */
typedef enum {
    SX1280_PKT_LORA    = 0x01,  /*!< LoRa 模式 */
} SX1280_PacketType_t;

/* --- LoRa 扩频因子（手册 Table 14-47） --- */
typedef enum {
    SX1280_SF5  = 0x50,
    SX1280_SF6  = 0x60,
    SX1280_SF7  = 0x70,   /*!< 默认 */
    SX1280_SF8  = 0x80,
    SX1280_SF9  = 0x90,
    SX1280_SF10 = 0xA0,
    SX1280_SF11 = 0xB0,
    SX1280_SF12 = 0xC0
} SX1280_LoRaSF_t;

/* --- LoRa 带宽（手册 Table 14-48） --- */
typedef enum {
    SX1280_BW1600 = 0x0A,  /*!< 1625 kHz */
    SX1280_BW800  = 0x18,  /*!< 812.5 kHz（默认） */
    SX1280_BW400  = 0x26,  /*!< 406.25 kHz */
    SX1280_BW200  = 0x34   /*!< 203.125 kHz */
} SX1280_LoRaBW_t;

/* --- LoRa 编码率（手册 Table 14-49） --- */
typedef enum {
    SX1280_CR45 = 0x01,  /*!< 4/5（默认） */
    SX1280_CR46 = 0x02,  /*!< 4/6 */
    SX1280_CR47 = 0x03,  /*!< 4/7 */
    SX1280_CR48 = 0x04   /*!< 4/8 */
} SX1280_LoRaCR_t;

/* --- LoRa 报头类型（手册 Table 14-51） --- */
typedef enum {
    SX1280_HEADER_EXPLICIT = 0x00,  /*!< 显式（变长） */
    SX1280_HEADER_IMPLICIT = 0x80   /*!< 隐式（定长） */
} SX1280_LoRaHeader_t;

/* --- LoRa CRC（手册 Table 14-53） --- */
#define SX1280_CRC_OFF        (0x00U)
#define SX1280_CRC_ON         (0x20U)

/* --- LoRa IQ 极性（手册 Table 14-54） --- */
#define SX1280_IQ_STANDARD    (0x40U)
#define SX1280_IQ_INVERTED    (0x00U)

/* --- 同步字特殊值 --- */
#define SX1280_SYNCWORD_KEEP_DEFAULT  (0xFFU)  /*!< 不修改同步字（POR 默认 0x12） */

/* --- 待机时钟源（手册 Table 11-20） --- */
typedef enum {
    SX1280_STDBY_RC   = 0x00,
    SX1280_STDBY_XOSC = 0x01
} SX1280_StandbyMode_t;

/* --- 睡眠保留配置（手册 Table 11-17） --- */
#define SX1280_SLEEP_NONE        (0x00U)
#define SX1280_SLEEP_RAM_RET     (0x01U)
#define SX1280_SLEEP_BUF_RET     (0x02U)
#define SX1280_SLEEP_FULL_RET    (0x03U)

/* --- 发射斜坡时间（手册 Table 11-49） --- */
typedef enum {
    SX1280_RAMP_2US  = 0x00,
    SX1280_RAMP_4US  = 0x20,
    SX1280_RAMP_6US  = 0x40,
    SX1280_RAMP_8US  = 0x60,
    SX1280_RAMP_10US = 0x80,
    SX1280_RAMP_12US = 0xA0,
    SX1280_RAMP_16US = 0xC0,
    SX1280_RAMP_20US = 0xE0   /*!< 默认 */
} SX1280_RampTime_t;

/* --- Rx/Tx 超时时间基准（手册 Table 11-24） --- */
typedef enum {
    SX1280_TIMEBASE_15_625US = 0x00,
    SX1280_TIMEBASE_62_5US   = 0x01,
    SX1280_TIMEBASE_1MS      = 0x02,
    SX1280_TIMEBASE_4MS      = 0x03
} SX1280_TimeBase_t;

/*============================================================================
 * 7. 返回错误码
 *==========================================================================*/
typedef enum {
    SX1280_OK         =  0,
    SX1280_ERR_PARAM  = -1,
    SX1280_ERR_SPI    = -2,
    SX1280_ERR_TIMEOUT= -3,
    SX1280_ERR_CMD    = -4,
    SX1280_ERR_STATE  = -5,
    SX1280_ERR_CRC    = -6,
    SX1280_ERR_DEVICE = -7,
    SX1280_ERR_SIZE   = -8,
    SX1280_ERR_BUSY   = -9
} SX1280_Status_t;

/*============================================================================
 * 8. 驱动配置结构体
 *==========================================================================*/
typedef struct {
    uint32_t rf_freq_hz;              /*!< 射频中心频率 Hz (2.4G~2.5G) */
    SX1280_LoRaSF_t    sf;            /*!< 扩频因子 */
    SX1280_LoRaBW_t    bw;            /*!< 带宽 */
    SX1280_LoRaCR_t    cr;            /*!< 编码率 */
    /**
     * @brief 前导码长度（符号数 1~255）
     * @note  硬件仅支持 m·2^n（m∈[1,15]，n∈[0,15]）形式，
     *        本驱动会将输入向上取整到最近的合法值。
     *        例如 17→18，255→256（即 1·2^8，编码后按 256 生效）。
     */
    uint8_t  preamble_len;
    uint8_t  header_type;             /*!< 显式/隐式报头 */
    uint8_t  payload_len;             /*!< 载荷长度（隐式报头固定长度，1~255） */
    uint8_t  crc_en;                  /*!< SX1280_CRC_ON / CRC_OFF */
    uint8_t  iq_mode;                 /*!< SX1280_IQ_STANDARD / IQ_INVERTED */
    uint8_t  syncword;                /*!< LoRa 同步字 0xXY；
                                          设为 SX1280_SYNCWORD_KEEP_DEFAULT 表示保持默认 */
    int8_t   power_dbm;               /*!< 发射功率 -18~+13 dBm */
    uint8_t  ramp_time;               /*!< PA 斜坡时间（SX1280_RampTime_t） */
} SX1280_Config_t;

#define SX1280_CONFIG_DEFAULT                                        \
    {                                                                \
        SX1280_FREQ_MIN_HZ,       /* rf_freq_hz = 2400MHz */         \
        SX1280_SF7,               /* sf      */                      \
        SX1280_BW800,             /* bw      */                      \
        SX1280_CR45,              /* cr      */                      \
        12U,                      /* preamble_len */                 \
        SX1280_HEADER_EXPLICIT,   /* header_type */                  \
        255U,                     /* payload_len */                  \
        SX1280_CRC_ON,            /* crc_en  */                      \
        SX1280_IQ_STANDARD,       /* iq_mode */                      \
        SX1280_SYNCWORD_KEEP_DEFAULT, /* syncword */                 \
        13,                       /* power_dbm */                    \
        SX1280_RAMP_20US,         /* ramp_time */                    \
    }

/*============================================================================
 * 9. 收包附加信息
 *==========================================================================*/
typedef struct {
    int16_t rssi_dbm;   /*!< 包 RSSI（dBm），已按 SNR<=0 修正 */
    int8_t  snr_q4;     /*!< 包 SNR 原始值（补码，×4） */
    int8_t  snr_db;     /*!< 包 SNR（dB）= snr_q4 / 4 */
} SX1280_RxInfo_t;

/*============================================================================
 * 10. 驱动状态与回调类型
 *==========================================================================*/
typedef enum {
    SX1280_STATE_IDLE = 0,
    SX1280_STATE_TX,
    SX1280_STATE_RX,
} SX1280_State_t;

typedef void (*SX1280_TxDoneCallback_t)(void);
typedef void (*SX1280_RxDoneCallback_t)(const uint8_t *data, uint8_t len,
                                        const SX1280_RxInfo_t *info);
typedef void (*SX1280_ErrorCallback_t)(int8_t error);

/*============================================================================
 * 11. API 函数声明
 *==========================================================================*/

/* ---------------------- 初始化 / 复位 / 电源模式 ---------------------- */
int8_t SX1280_Init(const SX1280_Config_t *cfg);
int8_t SX1280_Reset(void);
int8_t SX1280_SetStandby(SX1280_StandbyMode_t mode);
int8_t SX1280_SetSleep(uint8_t sleep_cfg);
int8_t SX1280_Wake(void);
int8_t MYSX1280_Init(void);

/* ---------------------- 非阻塞收发接口 ---------------------- */
int8_t SX1280_StartTransmit(const uint8_t *data, uint8_t len);

/**
 * @brief 启动非阻塞接收
 * @param timeout_ms 超时语义：
 *        - SX1280_RX_TIMEOUT_SINGLE (0)              : 无超时，收一包后回 STDBY
 *        - SX1280_RX_TIMEOUT_CONTINUOUS (0xFFFFFFFF) : 连续接收，需 Abort 停止
 *        - 其他值 (1 ~ SX1280_RX_TIMEOUT_MAX_MS)     : 带超时的单次接收
 * @return SX1280_OK 或错误码
 *
 * @note V2.2 起，本函数会自动用 s_config 缓存中的包参数
 *       （前导码 / 报头 / 载荷长度 / CRC / IQ）重写硬件寄存器，
 *       以消除上一次 SX1280_StartTransmit 留下的 TX 长度污染。
 *       因此 **不需要** 用户在 TX 后手动重设 payload_len。
 */
int8_t SX1280_StartReceive(uint32_t timeout_ms);

int8_t SX1280_Abort(void);
void    SX1280_Process(void);

void SX1280_DIO1_IRQHandler(void);
void SX1280_DIO2_IRQHandler(void);
void SX1280_DIO3_IRQHandler(void);

void SX1280_RegisterCallbacks(SX1280_TxDoneCallback_t tx_done,
                              SX1280_RxDoneCallback_t rx_done,
                              SX1280_ErrorCallback_t error);

SX1280_State_t SX1280_GetState(void);

/* ---------------------- 配置接口 ---------------------- */
int8_t SX1280_SetRfFrequency(uint32_t freq_hz);
int8_t SX1280_SetLoRaModulation(SX1280_LoRaSF_t sf, SX1280_LoRaBW_t bw,
                                SX1280_LoRaCR_t cr);

/**
 * @brief 设置 LoRa 包参数
 * @param preamble_len 前导码长度（1~255，硬件向上取整到 m·2^n）
 * @param header_type  显式/隐式报头
 * @param payload_len  载荷长度（隐式报头固定长度；显式报头下为最大值过滤门限）
 * @param crc_en       SX1280_CRC_ON / CRC_OFF
 * @param iq_mode      SX1280_IQ_STANDARD / IQ_INVERTED
 * @param update_cache 1=同时更新 s_config 缓存（用于 Init/静态配置）；
 *                     0=仅写硬件寄存器，不污染缓存（用于 StartTransmit 临时改长度）
 * @return SX1280_OK 或错误码
 *
 * @note V2.1 新增 update_cache 参数。显式传 0 可避免在发送时覆盖
 *       接收端已配置的 payload_len（隐式报头模式下尤为重要）。
 * @note 应用层一般无需直接调用；用户意图配置通过 SX1280_Config_t 传入
 *       SX1280_Init，之后由 StartReceive/StartTransmit 内部按需重写。
 */
int8_t SX1280_SetLoRaPacketParams(uint8_t preamble_len, uint8_t header_type,
                                  uint8_t payload_len, uint8_t crc_en,
                                  uint8_t iq_mode, uint8_t update_cache);

int8_t SX1280_SetLoRaSyncWord(uint8_t syncword);
int8_t SX1280_SetTxParams(int8_t power_dbm, SX1280_RampTime_t ramp_time);
int8_t SX1280_SetDioIrqParams(uint16_t irq_mask, uint16_t dio1_mask,
                              uint16_t dio2_mask, uint16_t dio3_mask);
int8_t SX1280_SetBufferBaseAddress(uint8_t tx_base, uint8_t rx_base);

/**
 * @brief 启用/禁用 AutoFs（Rx/Tx 完成后自动进入 FS 而非 STDBY）
 * @param enable 0=禁用，非 0=启用
 * @note 用于减少连续 Rx/Tx 切换开销，详见手册 11.6.12
 */
int8_t SX1280_SetAutoFs(uint8_t enable);

/**
 * @brief 读取 LoRa 频率误差指示（FEI）
 * @param freq_err_hz 输出：频率误差（Hz，有符号）
 * @return SX1280_OK 或错误码
 * @note 转换公式：FrequencyError[Hz] = 1.55 × SignedFEI / 1600，
 *       仅对 SNR > 0 的包可靠。
 */
int8_t SX1280_GetFrequencyError(int32_t *freq_err_hz);

/* ---------------------- 状态与诊断接口 ---------------------- */
int8_t SX1280_GetStatus(uint8_t *status);
int8_t SX1280_GetIrqStatus(uint16_t *irq);
int8_t SX1280_ClearIrqStatus(uint16_t irq_mask);
int8_t SX1280_GetPacketType(uint8_t *packet_type);
int8_t SX1280_GetRssiInst(uint8_t *rssi_raw);
int8_t SX1280_GetRxBufferStatus(uint8_t *payload_len, uint8_t *start_ptr);
int8_t SX1280_GetPacketStatus(uint8_t *status, uint8_t n);

int8_t SX1280_ReadBuffer(uint8_t offset, uint8_t *buf, uint8_t len);
int8_t SX1280_WriteBuffer(uint8_t offset, const uint8_t *buf, uint8_t len);
int8_t SX1280_ReadRegister(uint16_t addr, uint8_t *buf, uint8_t len);
int8_t SX1280_WriteRegister(uint16_t addr, const uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* SX1280_H */

#endif //INC_2_4G_SX1280_H
