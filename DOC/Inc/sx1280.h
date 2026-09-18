/**
 ******************************************************************************
 * @file    sx1280.h
 * @brief   Semtech SX1280/SX1281 2.4GHz 收发芯片驱动 —— 头文件
 * @details 基于 SX1280/SX1281 数据手册 Rev 3.2 (DS.SX1280-1.W.APP, 2020-03)
 *          - SPI 从机接口：CPOL=0 / CPHA=0，SCK 最高 18MHz，MSB 优先
 *            （手册 9.3 节，SCK 周期最短 55ns）
 *          - 支持 LoRa® / FLRC / GFSK / BLE 四种包类型；本驱动默认并深度封装
 *            LoRa 模式（手册 14.4 节），其余类型仅保留命令级接口
 *          - BUSY 引脚：低电平表示可接收新命令，驱动在每次 SPI 事务前后等待
 *          - NRESET：低有效硬件复位（手册 10.1 节，复位后等待 BUSY 变低）
 *          - 睡眠唤醒：NSS 下降沿触发，唤醒期间 BUSY 为高，就绪后拉低
 *            （手册 9.3.3 节，本驱动通过“探测帧 + 等待 BUSY 变低”实现自动唤醒）
 *
 * @note    平台相关硬件（SPI 收发 / BUSY / NRESET / 延时）全部抽象为弱函数，
 *          用户在自己的工程中定义同名非弱函数即可完成移植，无需修改本文件；
 *          NSS 片选由 SX1280_SPI_Transceive() 内部自行管理（可由硬件 SPI 负责）。
 *          详见 README.md。
 ******************************************************************************
 */
#ifndef SX1280_H
#define SX1280_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * 1. 器件基本信息与通用常量
 *==========================================================================*/
#define SX1280_SPI_MAX_FREQ_HZ  (18000000UL)  /*!< SPI 最大时钟 18MHz（手册 9.3 节） */
#define SX1280_PAYLOAD_MAX      (255U)        /*!< 数据缓冲/单包最大长度（字节）    */

/* 频率合成（手册 11.7.3 SetRfFrequency）：PLL 步进 = 52MHz / 2^18 ≈ 198.36Hz
 * 频率寄存器值 FREQ = f(Hz) × 2^18 / 52e6，24bit */
#define SX1280_PLL_STEP_NUM     (262144ULL)   /*!< 2^18                          */
#define SX1280_PLL_STEP_DEN      (52000000ULL) /*!< 晶振频率 52MHz（手册 3.4 节）  */

/* 频率合法范围（手册 3.4 节：FR = 2400 ~ 2500 MHz） */
#define SX1280_FREQ_MIN_HZ      (2400000000UL)
#define SX1280_FREQ_MAX_HZ      (2500000000UL)

/* 收发超时上限：RTC 时间基准最大 4ms × 16bit 计数 ≈ 262.14s（手册 11.6.5） */
#define SX1280_RX_TIMEOUT_MAX_MS (262000UL)

/* 阻塞等待相关默认超时（保守取值，非手册规格） */
#define SX1280_RESET_WAIT_MS     (200U)  /*!< NRESET 释放后等待 BUSY 变低的最长时间 */
#define SX1280_RESET_LOW_MS      (1U)    /*!< NRESET 拉低保持时间                    */
#define SX1280_WAKE_WAIT_MS      (50U)   /*!< 唤醒后等待 BUSY 变低的最长时间         */
#define SX1280_BUSY_WAIT_MS      (100U)  /*!< 常规命令前等待 BUSY 变低的最长时间     */

/*============================================================================
 * 2. SPI 命令操作码（手册第 12 章 Table 12-1：Transceiver Available Commands）
 *==========================================================================*/
#define SX1280_CMD_GET_STATUS          (0xC0U)  /*!< 读状态（SPI 单字节帧）          */
#define SX1280_CMD_GET_PACKET_TYPE     (0x03U)  /*!< 读当前包类型                    */
#define SX1280_CMD_WRITE_REGISTER      (0x18U)  /*!< 写寄存器：+2 字节地址 + 数据     */
#define SX1280_CMD_READ_REGISTER       (0x19U)  /*!< 读寄存器：+2 字节地址            */
#define SX1280_CMD_WRITE_BUFFER        (0x1AU)  /*!< 写数据缓冲：+1 字节偏移 + 数据   */
#define SX1280_CMD_READ_BUFFER         (0x1BU)  /*!< 读数据缓冲：+1 字节偏移          */
#define SX1280_CMD_SET_SLEEP           (0x84U)  /*!< 睡眠                            */
#define SX1280_CMD_SET_STANDBY         (0x80U)  /*!< 待机                            */
#define SX1280_CMD_SET_FS              (0xC1U)  /*!< 频率合成模式（测试用）           */
#define SX1280_CMD_SET_TX              (0x83U)  /*!< 发射                            */
#define SX1280_CMD_SET_RX              (0x82U)  /*!< 接收                            */
#define SX1280_CMD_SET_RX_DUTY_CYCLE   (0x94U)  /*!< 占空比接收                      */
#define SX1280_CMD_SET_CAD             (0xC5U)  /*!< LoRa 信道检测                   */
#define SX1280_CMD_SET_TX_CONT_WAVE    (0xD1U)  /*!< 连续载波（测试）                */
#define SX1280_CMD_SET_TX_CONT_PREAMBLE (0xD2U) /*!< 连续前导（测试）                */
#define SX1280_CMD_SET_AUTO_FS         (0x9EU)  /*!< Rx/Tx 结束后自动进 FS           */
#define SX1280_CMD_SET_AUTO_TX         (0x98U)  /*!< 自动应答（BLE）                 */
#define SX1280_CMD_SET_LONG_PREAMBLE   (0x9BU)  /*!< 长前导模式                      */
#define SX1280_CMD_SET_PACKET_TYPE     (0x8AU)  /*!< 设置包类型（先于其它配置）      */
#define SX1280_CMD_SET_MODULATION_PARAMS (0x8BU) /*!< 调制参数（3 字节）             */
#define SX1280_CMD_SET_PACKET_PARAMS   (0x8CU)  /*!< 包参数（7 字节）                */
#define SX1280_CMD_SET_RF_FREQUENCY    (0x86U)  /*!< 射频频率（3 字节 24bit）        */
#define SX1280_CMD_SET_TX_PARAMS       (0x8EU)  /*!< 发射功率 + 斜坡时间             */
#define SX1280_CMD_SET_CAD_PARAMS      (0x88U)  /*!< CAD 符号数                     */
#define SX1280_CMD_SET_BUFFER_BASE_ADDR (0x8FU) /*!< Tx/Rx 缓冲基地址                */
#define SX1280_CMD_SET_DIO_IRQ_PARAMS  (0x8DU)  /*!< IRQ 使能与 DIO 映射             */
#define SX1280_CMD_GET_IRQ_STATUS      (0x15U)  /*!< 读 IRQ 状态（2 字节）           */
#define SX1280_CMD_CLEAR_IRQ_STATUS    (0x97U)  /*!< 清 IRQ（2 字节掩码）            */
#define SX1280_CMD_GET_RX_BUFFER_STATUS (0x17U) /*!< 读 Rx 长度与起始指针            */
#define SX1280_CMD_GET_PACKET_STATUS   (0x1DU)  /*!< 读包状态（RSSI/SNR 等）         */
#define SX1280_CMD_GET_RSSI_INST       (0x1FU)  /*!< 读瞬时 RSSI                     */
#define SX1280_CMD_SET_REGULATOR_MODE  (0x96U)  /*!< LDO/DC-DC 选择                  */

/*============================================================================
 * 3. 状态字节定义（手册 Table 11-5，SPI 每帧 MISO 首字节即状态）
 *==========================================================================*/
#define SX1280_STATUS_MODE_MASK   (0xE0U)  /*!< bit[7:5] 电路模式     */
#define SX1280_STATUS_CMD_MASK    (0x1CU)  /*!< bit[4:2] 命令状态     */
#define SX1280_STATUS_MODE(s)     ((uint8_t)(((s) & SX1280_STATUS_MODE_MASK) >> 5))
#define SX1280_STATUS_CMD(s)      ((uint8_t)(((s) & SX1280_STATUS_CMD_MASK) >> 2))

/* 电路模式（模式值 0x0/0x1 保留） */
#define SX1280_MODE_STDBY_RC      (0x02U)  /*!< RC 13MHz 待机          */
#define SX1280_MODE_STDBY_XOSC    (0x03U)  /*!< 晶振 52MHz 待机        */
#define SX1280_MODE_FS            (0x04U)  /*!< 频率合成              */
#define SX1280_MODE_RX            (0x05U)  /*!< 接收                  */
#define SX1280_MODE_TX            (0x06U)  /*!< 发射                  */

/* 命令处理状态（0x0 保留） */
#define SX1280_CMDST_OK           (0x01U)  /*!< 命令成功处理          */
#define SX1280_CMDST_DATA_READY   (0x02U)  /*!< 数据可读（收包完成）  */
#define SX1280_CMDST_TIMEOUT      (0x03U)  /*!< 命令超时（需重发）    */
#define SX1280_CMDST_PROC_ERR     (0x04U)  /*!< 处理错误（非法操作码）*/
#define SX1280_CMDST_FAIL         (0x05U)  /*!< 执行失败              */
#define SX1280_CMDST_TX_DONE      (0x06U)  /*!< 发射完成              */

/*============================================================================
 * 4. IRQ 寄存器位定义（手册 Table 11-73，16bit 由 GetIrqStatus 读回）
 *==========================================================================*/
#define SX1280_IRQ_TX_DONE              (0x0001U) /*!< 发射完成（全部模式）    */
#define SX1280_IRQ_RX_DONE              (0x0002U) /*!< 接收完成（全部模式）    */
#define SX1280_IRQ_SYNC_WORD_VALID      (0x0004U) /*!< 同步字有效（FSK/BLE/FLRC） */
#define SX1280_IRQ_SYNC_WORD_ERROR      (0x0008U) /*!< 同步字错误（FLRC）      */
#define SX1280_IRQ_HEADER_VALID         (0x0010U) /*!< 报头有效（LoRa）        */
#define SX1280_IRQ_HEADER_ERROR         (0x0020U) /*!< 报头错误（LoRa）        */
#define SX1280_IRQ_CRC_ERROR            (0x0040U) /*!< CRC 错误（全部模式）    */
#define SX1280_IRQ_RANGING_SLAVE_DONE   (0x0080U) /*!< 测距（从）完成          */
#define SX1280_IRQ_RANGING_SLAVE_DISCARD (0x0100U) /*!< 测距（从）丢弃          */
#define SX1280_IRQ_RANGING_MASTER_VALID (0x0200U) /*!< 测距（主）结果有效      */
#define SX1280_IRQ_RANGING_MASTER_TIMEOUT (0x0400U) /*!< 测距（主）超时         */
#define SX1280_IRQ_RANGING_SLAVE_VALID  (0x0800U) /*!< 测距（从）请求有效      */
#define SX1280_IRQ_CAD_DONE             (0x1000U) /*!< 信道检测完成（LoRa）    */
#define SX1280_IRQ_CAD_DETECTED         (0x2000U) /*!< 检测到 LoRa 信号        */
#define SX1280_IRQ_RX_TX_TIMEOUT        (0x4000U) /*!< Rx/Tx 超时（全部模式）  */
#define SX1280_IRQ_PREAMBLE_DETECTED    (0x8000U) /*!< 前导检测/高级测距完成   */
#define SX1280_IRQ_ALL                  (0xFFFFU) /*!< 全部 IRQ                */

/*============================================================================
 * 5. 寄存器地址（手册第 13 章 Register Map；读/写命令均携带 2 字节地址）
 *==========================================================================*/
#define SX1280_REG_SF_ADD_CFG        (0x0925U) /*!< SF 附加配置（LoRa，按 SF 取值 0x1E/0x37/0x32） */
#define SX1280_REG_FREQ_ERR_COMP     (0x093CU) /*!< 频率误差补偿模式（LoRa 必须写 0x01）           */
#define SX1280_REG_LORA_SYNCWORD_MSB (0x0944U) /*!< LoRa 同步字高半字节位 [7:4]                    */
#define SX1280_REG_LORA_SYNCWORD_LSB (0x0945U) /*!< LoRa 同步字低半字节位 [7:4]                    */
#define SX1280_REG_RX_PAYLOAD_LEN    (0x0901U) /*!< 收到包长度（固定报头模式读此寄存器）           */

/*============================================================================
 * 6. 枚举与位域（数值即命令字节值，见手册 Table 11-42/14-47~14-54 等）
 *==========================================================================*/

/* --- 包类型（手册 Table 11-42：SetPacketType） --- */
typedef enum {
    SX1280_PKT_GFSK    = 0x00,  /*!< GFSK 模式（POR 默认） */
    SX1280_PKT_LORA    = 0x01,  /*!< LoRa 模式（本驱动默认）*/
    SX1280_PKT_RANGING = 0x02,  /*!< 测距模式             */
    SX1280_PKT_FLRC    = 0x03,  /*!< FLRC 模式            */
    SX1280_PKT_BLE     = 0x04   /*!< BLE 模式             */
} SX1280_PacketType_t;

/* --- LoRa 扩频因子（手册 Table 14-47，命令字节值） --- */
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

/* --- LoRa 带宽（手册 Table 14-48，命令字节值） --- */
typedef enum {
    SX1280_BW1600 = 0x0A,  /*!< 1625 kHz */
    SX1280_BW800  = 0x18,  /*!< 812.5 kHz（默认） */
    SX1280_BW400  = 0x26,  /*!< 406.25 kHz */
    SX1280_BW200  = 0x34   /*!< 203.125 kHz */
} SX1280_LoRaBW_t;

/* --- LoRa 编码率（手册 Table 14-49，命令字节值；长交织码率见 README） --- */
typedef enum {
    SX1280_CR45 = 0x01,  /*!< 4/5（默认） */
    SX1280_CR46 = 0x02,  /*!< 4/6 */
    SX1280_CR47 = 0x03,  /*!< 4/7 */
    SX1280_CR48 = 0x04   /*!< 4/8 */
} SX1280_LoRaCR_t;

/* --- LoRa 报头/长度类型（手册 Table 14-51） --- */
typedef enum {
    SX1280_HEADER_EXPLICIT = 0x00,  /*!< 显式（变长）报头，接收端长度来自报头 */
    SX1280_HEADER_IMPLICIT = 0x80   /*!< 隐式（定长）报头，收发长度必须一致   */
} SX1280_LoRaHeader_t;

/* --- LoRa CRC（手册 Table 14-53） --- */
#define SX1280_CRC_OFF        (0x00U)
#define SX1280_CRC_ON         (0x20U)

/* --- LoRa IQ 极性（手册 Table 14-54） --- */
#define SX1280_IQ_STANDARD    (0x40U)  /*!< 常规 IQ（默认） */
#define SX1280_IQ_INVERTED    (0x00U)  /*!< IQ 交换        */

/* --- 待机时钟源（手册 Table 11-20：SetStandby） --- */
typedef enum {
    SX1280_STDBY_RC   = 0x00,  /*!< RC 13MHz（低功耗，默认） */
    SX1280_STDBY_XOSC = 0x01   /*!< 52MHz 晶振（低时延）    */
} SX1280_StandbyMode_t;

/* --- 睡眠保留配置（手册 Table 11-17：SetSleep sleepConfig） --- */
#define SX1280_SLEEP_NONE        (0x00U)  /*!< 不保留任何数据（唤醒需重配置）   */
#define SX1280_SLEEP_RAM_RET     (0x01U)  /*!< bit0：数据 RAM 保留（配置/校准） */
#define SX1280_SLEEP_BUF_RET     (0x02U)  /*!< bit1：数据缓冲保留               */
#define SX1280_SLEEP_FULL_RET    (0x03U)  /*!< RAM + 缓冲都保留                */

/* --- 发射斜坡时间（手册 Table 11-49：SetTxParams rampTime） --- */
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
    SX1280_TIMEBASE_15_625US = 0x00,  /*!< 15.625 µs */
    SX1280_TIMEBASE_62_5US   = 0x01,  /*!< 62.5 µs   */
    SX1280_TIMEBASE_1MS      = 0x02,  /*!< 1 ms      */
    SX1280_TIMEBASE_4MS      = 0x03   /*!< 4 ms      */
} SX1280_TimeBase_t;

/*============================================================================
 * 7. 返回错误码
 *==========================================================================*/
typedef enum {
    SX1280_OK         =  0,   /*!< 操作成功                            */
    SX1280_ERR_PARAM  = -1,   /*!< 参数错误（空指针/越界/非法取值等）  */
    SX1280_ERR_SPI    = -2,   /*!< SPI 通信失败（由移植层返回）        */
    SX1280_ERR_TIMEOUT= -3,   /*!< 等待超时（BUSY/收包/发包）          */
    SX1280_ERR_CMD    = -4,   /*!< 芯片返回命令处理错误/失败           */
    SX1280_ERR_STATE  = -5,   /*!< 芯片模式与操作不匹配                */
    SX1280_ERR_CRC    = -6,   /*!< 收到数据 CRC 校验失败               */
    SX1280_ERR_DEVICE = -7,   /*!< 器件未就绪/不在位（无法通信）       */
    SX1280_ERR_SIZE   = -8    /*!< 接收数据长度超出用户缓冲区           */
} SX1280_Status_t;

/*============================================================================
 * 8. 平台弱函数宏（用户定义同名非弱函数即可覆盖占位实现）
 *==========================================================================*/
#if defined(__GNUC__) || defined(__clang__)
  #define SX1280_WEAK __attribute__((weak))          /* GCC / Clang          */
#elif defined(__ICCARM__) || defined(__CC_ARM) || defined(__ARMCC_VERSION)
  #define SX1280_WEAK __weak                          /* IAR / Keil AC5 / AC6 */
#else
  #define SX1280_WEAK                                 /* 其他编译器：直接改 .c */
#endif

/*============================================================================
 * 9. 底层硬件移植接口（弱函数，占位实现见 sx1280.c，用户必须覆盖）
 *==========================================================================*/
/**
 * @brief 执行一次完整的 SPI 全双工事务（用户必须实现）
 * @note   协议要求：
 *         - SPI 模式 CPOL=0 / CPHA=0，MSB 优先，时钟 ≤ 18MHz；
 *         - 本函数内部负责 NSS 片选拉低 → 收发 len 字节 → NSS 拉高
 *           （NSS 可由硬件 SPI 外设管理，也可在函数内手动翻转 GPIO）；
 *         - 调用前驱动已保证 BUSY 为低，函数返回后驱动会等待 BUSY 释放；
 *         - 帧最大长度不超过 259 字节（读缓冲 255 + 4），单次事务完成即可。
 * @param tx 待发送数据（含操作码与参数；全双工时仍须提供）
 * @param rx 接收数据缓冲区（MISO 数据写入此处，可传 NULL 忽略）
 * @param len 传输字节数（≥1）
 * @return SX1280_OK 成功；其余返回值视为通信失败（驱动返回 SX1280_ERR_SPI）
 */
SX1280_WEAK int8_t SX1280_SPI_Transceive(const uint8_t *tx, uint8_t *rx,
                                         uint16_t len);

/**
 * @brief 读取 BUSY 引脚电平（用户必须实现）
 * @return 1 = 芯片忙（BUSY 高），不可发新命令；0 = 就绪（BUSY 低）
 */
SX1280_WEAK uint8_t SX1280_ReadBusy(void);

/**
 * @brief 控制 NRESET 引脚（用户必须实现；NRESET 低有效）
 * @param level 0 = 复位（拉低）；1 = 释放复位（拉高）
 */
SX1280_WEAK void SX1280_SetResetPin(uint8_t level);

/**
 * @brief 微秒延时（用户必须实现，供 BUSY 轮询节奏使用）
 * @param us 延时微秒数
 */
SX1280_WEAK void SX1280_DelayUs(uint32_t us);

/**
 * @brief 毫秒延时（用户必须实现）
 * @param ms 延时毫秒数
 */
SX1280_WEAK void SX1280_DelayMs(uint32_t ms);

/*============================================================================
 * 10. 驱动配置结构体（SX1280_Init 使用；传 NULL 时套用默认宏）
 *==========================================================================*/
/**
 * @brief SX1280 射频配置
 * @note  本驱动深度封装 LoRa 模式：Init 时按此结构完成
 *        Standby → SetPacketType(LoRa) → 频率 → 调制/包参数 → 发射参数
 *        → IRQ 映射 的完整配置序列（手册 14.4.1 节）。
 *        packet_type 仅用于读取语义提示，请保持 SX1280_PKT_LORA。
 */
typedef struct {
    SX1280_PacketType_t packet_type;  /*!< 包类型：保持 LoRa                */

    uint32_t rf_freq_hz;              /*!< 射频中心频率 Hz（2.4G~2.5G）     */
    SX1280_LoRaSF_t    sf;            /*!< LoRa 扩频因子                   */
    SX1280_LoRaBW_t    bw;            /*!< LoRa 带宽                       */
    SX1280_LoRaCR_t    cr;            /*!< LoRa 编码率                     */
    uint8_t  preamble_len;            /*!< 前导码长度（符号数 1~255）      */
    uint8_t  header_type;             /*!< 显式/隐式报头（SX1280_LoRaHeader_t） */
    uint8_t  payload_len;             /*!< 载荷长度（隐式报头固定长度，1~255） */
    uint8_t  crc_en;                  /*!< CRC：SX1280_CRC_ON / CRC_OFF    */
    uint8_t  iq_mode;                 /*!< IQ：SX1280_IQ_STANDARD / IQ_INVERTED */
    uint8_t  syncword;                /*!< LoRa 同步字 0xXY（0 = 保持 POR 默认） */
    int8_t   power_dbm;               /*!< 发射功率 dBm：-18 ~ +13         */
    uint8_t  ramp_time;               /*!< PA 斜坡时间（SX1280_RampTime_t） */

    uint16_t irq_mask;                /*!< IRQ 使能掩码（0=默认使能集）    */
    uint16_t dio1_mask;               /*!< 映射到 DIO1 的 IRQ（0=默认）    */
    uint16_t dio2_mask;               /*!< 映射到 DIO2 的 IRQ（0=默认）    */
    uint16_t dio3_mask;               /*!< 映射到 DIO3 的 IRQ（0=默认）    */
} SX1280_Config_t;

/* 默认配置：2400.0MHz / SF7 / BW 800kHz / CR 4/5 / 12 符号前导 / 显式报头 /
 * CRC 开 / 常规 IQ / 同步字 0x12（POR 默认）/ +13dBm / 20µs 斜坡 /
 * IRQ：TxDone|RxDone|RxTxTimeout 全部映射到 DIO1，CRC/报头错误映射 DIO2 */
#define SX1280_CONFIG_DEFAULT                                        \
    {                                                                \
        SX1280_PKT_LORA,          /* packet_type */                  \
        SX1280_FREQ_MIN_HZ,       /* rf_freq_hz = 2400MHz */         \
        SX1280_SF7,               /* sf      */                      \
        SX1280_BW800,             /* bw      */                      \
        SX1280_CR45,              /* cr      */                      \
        12U,                      /* preamble_len */                 \
        SX1280_HEADER_EXPLICIT,   /* header_type */                  \
        255U,                     /* payload_len */                  \
        SX1280_CRC_ON,            /* crc_en  */                      \
        SX1280_IQ_STANDARD,       /* iq_mode */                      \
        0x12U,                    /* syncword（POR 默认值）*/        \
        13,                       /* power_dbm */                    \
        SX1280_RAMP_20US,         /* ramp_time */                    \
        (SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE |                   \
         SX1280_IRQ_RX_TX_TIMEOUT | SX1280_IRQ_HEADER_ERROR |        \
         SX1280_IRQ_CRC_ERROR),   /* irq_mask（默认使能集）*/        \
        (SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE |                   \
         SX1280_IRQ_RX_TX_TIMEOUT),/* dio1_mask */                    \
        (SX1280_IRQ_HEADER_ERROR | SX1280_IRQ_CRC_ERROR),/* dio2_mask */ \
        0x0000U,                  /* dio3_mask */                    \
    }

/*============================================================================
 * 11. 收包附加信息（手册 Table 11-65/11-66，仅 LoRa 有效字段）
 *==========================================================================*/
typedef struct {
    int16_t rssi_dbm;   /*!< 包 RSSI（dBm）。LoRa：RSSI = -rssiSync/2；
                             若 SNR≤0 且未修正，实际值 = RSSI - SNR */
    int8_t  snr_q4;     /*!< 包 SNR 原始值（补码，×4，0.25dB 分辨率）   */
    int8_t  snr_db;     /*!< 包 SNR（dB）= snr_q4 / 4（LoRa 才有）     */
} SX1280_RxInfo_t;

/*============================================================================
 * 12. 驱动 API 函数声明
 *==========================================================================*/

/* ---------------------- 初始化 / 复位 / 电源模式 ---------------------- */

/**
 * @brief 初始化 SX1280（硬件复位 + LoRa 全配置）
 * @note  流程：NRESET 硬件复位 → 等 BUSY 低 → SetStandby(RC) →
 *        SetPacketType(LoRa) → SetBufferBaseAddress → SetModulationParams
 *        （含 SF 附加配置寄存器 0x925/0x93C，手册 14.4.1）→ SetPacketParams
 *        → LoRa 同步字 → SetRfFrequency → SetTxParams → SetDioIrqParams
 *        → ClearIrqStatus(全部)。复位后首次上电也适用。
 * @param cfg 配置结构体指针；NULL 表示使用 SX1280_CONFIG_DEFAULT
 * @return SX1280_OK 成功；否则返回对应错误码
 */
int8_t SX1280_Init(const SX1280_Config_t *cfg);

/**
 * @brief 硬件复位（NRESET 拉低 ≥1ms → 释放 → 等待 BUSY 变低）
 * @note  复位后芯片恢复 POR 状态（包类型 GFSK、全部寄存器默认），
 *        必须重新调用 SX1280_Init() 配置后才能收发。
 * @return SX1280_OK 成功；SX1280_ERR_TIMEOUT BUSY 未在期限内就绪
 */
int8_t SX1280_Reset(void);

/**
 * @brief 进入待机模式（手册 11.6.2 SetStandby）
 * @param mode SX1280_STDBY_RC（默认，RC 13MHz）/ SX1280_STDBY_XOSC（晶振）
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_SetStandby(SX1280_StandbyMode_t mode);

/**
 * @brief 进入睡眠模式（手册 11.6.1 SetSleep）
 * @param sleep_cfg 保留配置：SX1280_SLEEP_NONE / RAM_RET / BUF_RET / FULL_RET
 * @note  1) 睡眠前自动先回 STDBY_RC；
 *        2) 不保留数据 RAM 时（NONE），唤醒后所有配置丢失，需重新 Init；
 *        3) 唤醒由下一次调用任意驱动函数自动完成（NSS 下降沿唤醒 +
 *           探测帧 + 等 BUSY 低），或显式调用 SX1280_Wake()。
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_SetSleep(uint8_t sleep_cfg);

/**
 * @brief 显式唤醒睡眠中的芯片（手册 9.3.3 节）
 * @note  唤醒方式：发一个探测帧（触发 NSS 下降沿唤醒）→ 等待 BUSY 变低
 *        （芯片完成启动进入 STDBY_RC）。芯片未睡眠时调用无副作用。
 * @return SX1280_OK 成功；SX1280_ERR_TIMEOUT 唤醒超时
 */
int8_t SX1280_Wake(void);

/* ---------------------- 配置接口（LoRa 各参数） ---------------------- */

/**
 * @brief 设置包类型（手册 11.7.1 SetPacketType）
 * @note  切换前驱动自动回 STDBY_RC；切换后旧类型的调制/包参数不保留。
 *        本驱动只深度封装 LoRa（SX1280_PKT_LORA）；切换到 FLRC/GFSK/BLE
 *        后相关参数需按手册 14.1~14.3 节自行下发（命令级接口已开放）。
 * @param type 包类型（SX1280_PKT_xxx）
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_SetPacketType(SX1280_PacketType_t type);

/**
 * @brief 设置射频频率（手册 11.7.3 SetRfFrequency，命令须在 SetPacketType 之后）
 * @note  Tx/Rx 频率差（1.3MHz IF）由芯片内部处理，无需用户补偿。
 *        频率寄存器值 = f(Hz) × 2^18 / 52e6（24bit）。
 * @param freq_hz 频率 Hz，范围 2400MHz ~ 2500MHz
 * @return SX1280_OK 成功；SX1280_ERR_PARAM 频率越界
 */
int8_t SX1280_SetRfFrequency(uint32_t freq_hz);

/**
 * @brief 设置 LoRa 调制参数（手册 11.7.7 SetModulationParams + 14.4.1）
 * @note  需先处于 LoRa 包类型（Init 后即是）。内部自动按 SF 写附加配置
 *        寄存器 0x925（SF5/6→0x1E，SF7/8→0x37，SF9~12→0x32）及
 *        频率误差补偿寄存器 0x93C=0x01。
 * @param sf 扩频因子（SX1280_LoRaSF_t）
 * @param bw 带宽（SX1280_LoRaBW_t）
 * @param cr 编码率（SX1280_LoRaCR_t）
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_SetLoRaModulation(SX1280_LoRaSF_t sf, SX1280_LoRaBW_t bw,
                                SX1280_LoRaCR_t cr);

/**
 * @brief 设置 LoRa 包参数（手册 11.7.8 SetPacketParams + Table 14-50~54）
 * @param preamble_len 前导码符号数（1~255；建议 ≥6，经典值 12）
 * @param header_type  SX1280_HEADER_EXPLICIT（变长，推荐）/ IMPLICIT（定长）
 * @param payload_len  载荷长度字节数 1~255；显式报头时仅作上限/预留，
 *                     隐式报头时收发两端必须一致（固定长度）
 * @param crc_en       SX1280_CRC_ON / SX1280_CRC_OFF
 * @param iq_mode      SX1280_IQ_STANDARD / SX1280_IQ_INVERTED
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_SetLoRaPacketParams(uint8_t preamble_len, uint8_t header_type,
                                  uint8_t payload_len, uint8_t crc_en,
                                  uint8_t iq_mode);

/**
 * @brief 设置 LoRa 同步字（手册 14.4.1：写寄存器 0x944/0x945 的高半字节）
 * @param syncword 8bit 同步字 0xXY：X→0x944[7:4]，Y→0x945[7:4]
 * @note  POR 默认同步字为 0x12（Semtech 私有网络）；LoRaWAN 公共网络为 0x34。
 *        通信双方必须一致；不同同步字的节点互不可见。
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_SetLoRaSyncWord(uint8_t syncword);

/**
 * @brief 设置发射功率与 PA 斜坡时间（手册 11.7.4 SetTxParams）
 * @param power_dbm 输出功率 dBm：-18 ~ +13（芯片内部 = power 码 -18）
 * @param ramp_time PA 斜坡时间（SX1280_RampTime_t，默认 20µs）
 * @return SX1280_OK 成功；SX1280_ERR_PARAM 功率越界
 */
int8_t SX1280_SetTxParams(int8_t power_dbm, SX1280_RampTime_t ramp_time);

/**
 * @brief 配置 IRQ 使能及各 DIO 引脚映射（手册 11.9.1 SetDioIrqParams）
 * @note  驱动内部收发靠 SPI 轮询 IRQ 寄存器完成，不依赖 DIO；
 *        但保留此接口并把 IRQ 正确映射到 DIO1~3，方便用户用外部中断。
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_SetDioIrqParams(uint16_t irq_mask, uint16_t dio1_mask,
                              uint16_t dio2_mask, uint16_t dio3_mask);

/**
 * @brief 设置数据缓冲基地址（手册 11.7.6 SetBufferBaseAddress）
 * @param tx_base Tx 缓冲起始偏移（WriteBuffer 写数据的位置，默认 0x00）
 * @param rx_base Rx 缓冲起始偏移（默认 0x00）
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_SetBufferBaseAddress(uint8_t tx_base, uint8_t rx_base);

/* ---------------------- 数据传输接口（带返回值，阻塞式） -------------- */

/**
 * @brief 发送一包数据（阻塞直到发射完成或超时）
 * @note  流程：回 STDBY_RC → WriteBuffer(TxBase) → ClearIrq →
 *        SetTx（无内部超时）→ 轮询 TxDone / RxTxTimeout IRQ。
 *        发射完成或超时后芯片自动回 STDBY_RC（手册 14.4.2）。
 * @param data      待发送数据（非空）
 * @param len       数据长度 1~255（当前包类型下超长报错）
 * @param timeout_ms 阻塞超时 ms；0 表示无限等待（慎用）。建议 ≥ 300ms
 * @return SX1280_OK 成功；SX1280_ERR_PARAM/ERR_SPI/ERR_TIMEOUT/ERR_CMD
 */
int8_t SX1280_Transmit(const uint8_t *data, uint8_t len, uint32_t timeout_ms);

/**
 * @brief 接收一包数据（阻塞直到收到有效包或超时）
 * @note  流程：回 STDBY_RC → ClearIrq → SetRx（按 timeout_ms 折算芯片
 *        内部 RTC 超时，单包模式）→ 轮询 RxDone / RxTxTimeout /
 *        HeaderError / CrcError IRQ → 通过后 GetRxBufferStatus 取长度与
 *        起始指针 → ReadBuffer 读出；接收完成后芯片自动回 STDBY_RC。
 *        显式报头模式长度取自报头；隐式报头模式自动使用配置的固定长度。
 *        单包接收成功后仅返回一包数据，如需连续接收请循环调用。
 * @param data      接收缓冲区（≥ max_len）
 * @param max_len   缓冲区容量（1~255）；收到的包更长返回 SX1280_ERR_SIZE
 * @param rx_len    输出实际接收字节数（可为 NULL）
 * @param info      输出包 RSSI/SNR（可为 NULL，仅 LoRa 提供有效值）
 * @param timeout_ms 等待超时 ms（0=无限等待）；最大 262000ms，
 *                   超过按最大值处理（芯片 RTC 限制，手册 11.6.5）
 * @return SX1280_OK 成功；SX1280_ERR_TIMEOUT 未收到包；
 *         SX1280_ERR_CRC 收包但 CRC 错误；其余为通信/参数错误
 */
int8_t SX1280_Receive(uint8_t *data, uint8_t max_len, uint8_t *rx_len,
                      SX1280_RxInfo_t *info, uint32_t timeout_ms);

/* ---------------------- 状态与诊断接口 ---------------------- */

/**
 * @brief 读取状态字节（手册 11.3 GetStatus；SPI 单字节帧）
 * @note  注意：手册要求 GetStatus 不能是接口上第一条命令（复位后应先
 *        由 SX1280_Init 发出的配置命令建立接口），正常使用时无碍。
 * @param status 输出状态字节；可用 SX1280_STATUS_MODE(s)/STATUS_CMD(s) 解析
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_GetStatus(uint8_t *status);

/**
 * @brief 读取 IRQ 状态寄存器（手册 11.9.2 GetIrqStatus）
 * @param irq 输出 16bit IRQ 位（SX1280_IRQ_xxx）
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_GetIrqStatus(uint16_t *irq);

/**
 * @brief 清除指定 IRQ 标志（手册 11.9.3 ClearIrqStatus；位写 1 对应清除）
 * @param irq_mask 要清除的 IRQ 位掩码；SX1280_IRQ_ALL 清除全部
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_ClearIrqStatus(uint16_t irq_mask);

/**
 * @brief 读取当前包类型（手册 11.7.2 GetPacketType）
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_GetPacketType(uint8_t *packet_type);

/**
 * @brief 读取瞬时 RSSI（手册 11.8.3 GetRssiInst；仅接收进行中有意义）
 * @note  LoRa 模式下实际信号功率 RSSI(dBm) = -rssi_raw / 2
 * @param rssi_raw 输出芯片原始值
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_GetRssiInst(uint8_t *rssi_raw);

/**
 * @brief 读取数据缓冲（手册 11.5.2 ReadBuffer）
 * @param offset 缓冲内起始偏移（Rx 时应使用收包起始指针）
 * @param buf 输出缓冲区
 * @param len 读取字节数 1~255
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_ReadBuffer(uint8_t offset, uint8_t *buf, uint8_t len);

/**
 * @brief 写入数据缓冲（手册 11.5.1 WriteBuffer）
 * @param offset 缓冲内起始偏移（Tx 时应使用 Tx 基地址）
 * @param buf 数据源
 * @param len 字节数 1~255
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_WriteBuffer(uint8_t offset, const uint8_t *buf, uint8_t len);

/**
 * @brief 读芯片寄存器（手册 11.4.2 ReadRegister）
 * @param addr 16bit 寄存器地址（手册第 13 章）
 * @param buf 输出缓冲区（可跨多字节连续读）
 * @param len 字节数（建议 ≤64，栈开销考虑）
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_ReadRegister(uint16_t addr, uint8_t *buf, uint8_t len);

/**
 * @brief 写芯片寄存器（手册 11.4.1 WriteRegister）
 * @param addr 16bit 寄存器地址
 * @param buf 数据源
 * @param len 字节数
 * @return SX1280_OK 成功；否则错误码
 */
int8_t SX1280_WriteRegister(uint16_t addr, const uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* SX1280_H */
