//
// Created by WIN on 2026/9/19.
//

#ifndef INC_2_4G_HDC1080_H
#define INC_2_4G_HDC1080_H


/**
 ******************************************************************************
 * @file    HDC1080.h
 * @brief   HDC1080 数字温湿度传感器驱动 —— 头文件
 * @details 基于 TI HDC1080 数据手册 SNAS672 (ZHCSET2A, 2015-11 ~ 2016-01 修订)
 *          - I2C 从机接口，7-bit 地址 0x40（写 0x80 / 读 0x81）
 *          - 所有数据字节 MSB 优先，8-bit 指针寄存器寻址
 *          - 上电后最多 15ms 就绪；温湿度转换时间取决于分辨率（见 7.5 节）
 *          - 相对湿度精度 ±2% (典型)，温度精度 ±0.2°C (典型)
 *
* @note    本驱动默认使用 STM32 HAL 硬件 I2C 阻塞式读写，
 *          通过 HDC1080_I2C_HANDLE 宏指定 I2C 句柄（默认 hi2c1），
 *          HDC1080_I2C_TIMEOUT 指定超时。非 HAL 平台可直接修改
 *          hdc1080.c 中 HDC1080_I2C_Write / HDC1080_I2C_Read /
 *          HDC1080_DelayMs 三个函数。详见 README.md。
 ******************************************************************************
 */
#ifndef HDC1080_H
#define HDC1080_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * 1. 器件基本信息
 *==========================================================================*/
#define HDC1080_I2C_ADDR         (0x40U)   /*!< 7-bit 从机地址 (1000000b)      */
#define HDC1080_WR_ADDR          ((HDC1080_I2C_ADDR << 1) | 0U) /*!< 0x80 写  */
#define HDC1080_RD_ADDR          ((HDC1080_I2C_ADDR << 1) | 1U) /*!< 0x81 读  */

#define HDC1080_MANUFACTURER_ID  (0x5449U) /*!< 厂商 ID（Texas Instruments）  */
#define HDC1080_DEVICE_ID        (0x1050U) /*!< 器件 ID（HDC1080）             */

/*============================================================================
 * 2. 寄存器地址（8-bit 指针寄存器值）
 *==========================================================================*/
#define HDC1080_REG_TEMPERATURE  (0x00U)   /*!< 温度寄存器（只读，16bit）      */
#define HDC1080_REG_HUMIDITY     (0x01U)   /*!< 湿度寄存器（只读，16bit）      */
#define HDC1080_REG_CONFIG       (0x02U)   /*!< 配置寄存器（可读写，16bit）    */
#define HDC1080_REG_SERIAL_ID_1  (0xFBU)   /*!< 序列号高 16 位 (bit40:25)      */
#define HDC1080_REG_SERIAL_ID_2  (0xFCU)   /*!< 序列号中 16 位 (bit24:9)       */
#define HDC1080_REG_SERIAL_ID_3  (0xFDU)   /*!< 序列号低 9 位 (bit8:0, bit15:7) */
#define HDC1080_REG_MANUF_ID     (0xFEU)   /*!< 厂商 ID 寄存器（只读）         */
#define HDC1080_REG_DEVICE_ID    (0xFFU)   /*!< 器件 ID 寄存器（只读）         */

/*============================================================================
 * 3. 配置寄存器 (0x02) 位定义
 *==========================================================================*/
#define HDC1080_CFG_RST          (0x8000U) /*!< bit15  软复位，写 1 触发，自清零 */
#define HDC1080_CFG_HEAT         (0x2000U) /*!< bit13  加热器使能（1=开启）     */
#define HDC1080_CFG_MODE         (0x1000U) /*!< bit12  采集模式：1=温湿度顺序采集
                                                0=仅温度或仅湿度                 */
#define HDC1080_CFG_BTST         (0x0800U) /*!< bit11  电池状态（只读）：1=电压<2.8V */
#define HDC1080_CFG_TRES_MASK    (0x0400U) /*!< bit10  温度分辨率：0=14bit / 1=11bit */
#define HDC1080_CFG_TRES_14BIT   (0x0000U) /*!< 温度 14bit 分辨率               */
#define HDC1080_CFG_TRES_11BIT   (0x0400U) /*!< 温度 11bit 分辨率               */
#define HDC1080_CFG_HRES_MASK    (0x0300U) /*!< bit[9:8] 湿度分辨率             */
#define HDC1080_CFG_HRES_14BIT   (0x0000U) /*!< 湿度 14bit 分辨率               */
#define HDC1080_CFG_HRES_11BIT   (0x0100U) /*!< 湿度 11bit 分辨率               */
#define HDC1080_CFG_HRES_8BIT    (0x0200U) /*!< 湿度 8bit 分辨率                */
#define HDC1080_CFG_RESERVED     (0x407FU) /*!< bit14 与 bit[7:0] 保留位，必须写 0 */
#define HDC1080_CFG_DEFAULT      (0x1000U) /*!< POR 复位默认值（MODE=1 温湿度顺序采集） */

/*============================================================================
 * 4. 时序参数（单位：ms，取值 ≥ 手册最大值，留有余量）
 *    手册 7.5 节：温度 14bit=6.35ms / 11bit=3.65ms；
 *                湿度 14bit=6.50ms / 11bit=3.85ms / 8bit=2.50ms
 *==========================================================================*/
#define HDC1080_POWERUP_TIME_MS   (15U)    /*!< 上电就绪时间最大值              */
#define HDC1080_CONV_TEMP_14BIT_MS (7U)    /*!< 温度 14bit 转换时间（取整）     */
#define HDC1080_CONV_TEMP_11BIT_MS (4U)    /*!< 温度 11bit 转换时间（取整）     */
#define HDC1080_CONV_HUM_14BIT_MS  (7U)    /*!< 湿度 14bit 转换时间（取整）     */
#define HDC1080_CONV_HUM_11BIT_MS  (4U)    /*!< 湿度 11bit 转换时间（取整）     */
#define HDC1080_CONV_HUM_8BIT_MS   (3U)    /*!< 湿度 8bit 转换时间（取整）      */

/*============================================================================
 * 5. 分辨率枚举
 *==========================================================================*/
typedef enum {
    HDC1080_RES_14BIT = 0,   /*!< 14bit（温度/湿度均支持，默认） */
    HDC1080_RES_11BIT = 1,   /*!< 11bit（温度/湿度均支持）       */
    HDC1080_RES_8BIT  = 2    /*!< 8bit（仅湿度支持）             */
} HDC1080_Resolution_t;

/*============================================================================
 * 6. 返回错误码
 *==========================================================================*/
typedef enum {
    HDC1080_OK          =  0,  /*!< 操作成功                         */
    HDC1080_ERR_I2C     = -1,  /*!< I2C 通信失败（无应答/总线错误等） */
    HDC1080_ERR_CRC     = -2,  /*!< CRC 校验失败                     */
    HDC1080_ERR_PARAM   = -3,  /*!< 参数错误（空指针/非法参数等）     */
    HDC1080_ERR_TIMEOUT = -4,  /*!< 等待转换完成超时                 */
    HDC1080_ERR_DEVICE  = -5   /*!< 器件 ID 校验失败（器件不在位）    */
} HDC1080_Status_t;

/*============================================================================
 * 7. 数据结构
 *==========================================================================*/
/**
 * @brief 温湿度测量结果
 * @note  raw 为 16 位寄存器原始值（14bit 有效，低 2 位恒为 0）；
 *        temp_c / hum_rh 为换算后的实际值（float）。
 */
typedef struct {
    uint16_t temp_raw;   /*!< 温度原始值（ADC 输出，14bit 有效）   */
    uint16_t hum_raw;    /*!< 湿度原始值（ADC 输出，14bit 有效）   */
    float    temp_c;     /*!< 温度实际值，单位 °C                  */
    float    hum_rh;     /*!< 湿度实际值，单位 %RH                 */
} HDC1080_data_t;

/*============================================================================
 * 9. 硬件 I2C 配置（默认使用 STM32 HAL，用户可覆盖）
 *==========================================================================*/
/* 指定使用的 I2C 句柄，默认 hi2c1。可在编译选项中定义覆盖，例如：
 *   -DHDC1080_I2C_HANDLE=hi2c2
 */
#ifndef HDC1080_I2C_HANDLE
#define HDC1080_I2C_HANDLE  hi2c1
#endif

/* I2C 操作超时时间（毫秒） */
#ifndef HDC1080_I2C_TIMEOUT
#define HDC1080_I2C_TIMEOUT  100U
#endif

/**
 * @brief 向器件写入数据（I2C 写事务）
 * @param dev_addr  7-bit 器件地址（HDC1080_I2C_ADDR = 0x40）
 * @param reg_addr  寄存器（指针寄存器）地址
 * @param buf       待写入数据缓冲区；len==0 时可为 NULL
 * @param len       待写入数据长度（字节）
 * @return HDC1080_OK 成功；否则返回错误码
 */
int8_t HDC1080_I2C_Write(uint8_t dev_addr, uint8_t reg_addr,
                         const uint8_t *buf, uint16_t len);

/**
 * @brief 从器件读取数据（I2C 读事务：先写指针，再重启读）
 * @param dev_addr  7-bit 器件地址
 * @param reg_addr  寄存器（指针寄存器）地址
 * @param buf       读取数据缓冲区（不能为 NULL）
 * @param len       读取数据长度（字节）
 * @return HDC1080_OK 成功；否则返回错误码
 */
int8_t HDC1080_I2C_Read(uint8_t dev_addr, uint8_t reg_addr,
                        uint8_t *buf, uint16_t len);

/**
 * @brief 毫秒延时函数
 * @param ms 延时毫秒数
 */
void HDC1080_DelayMs(uint32_t ms);

/*============================================================================
 * 10. 驱动 API 函数声明
 *==========================================================================*/
/**
 * @brief 初始化 HDC1080
 * @note  1) 等待上电就绪（最多 15ms）
 *        2) 读取器件 ID 校验器件在位且 I2C 通信正常
 * @return HDC1080_OK 成功；HDC1080_ERR_I2C / HDC1080_ERR_DEVICE 失败
 */
int8_t HDC1080_Init(void);

/**
 * @brief 软复位（写配置寄存器 RST 位，自清零）
 * @note  复位后器件配置恢复为 POR 默认值，需重新配置分辨率等参数
 * @return HDC1080_OK 成功；否则返回错误码
 */
int8_t HDC1080_SoftReset(void);

/**
 * @brief 读取配置寄存器 (0x02)
 * @param cfg 输出配置值
 * @return HDC1080_OK 成功；HDC1080_ERR_PARAM / HDC1080_ERR_I2C 失败
 */
int8_t HDC1080_ReadConfig(uint16_t *cfg);

/**
 * @brief 写配置寄存器 (0x02)
 * @param cfg 待写入配置值（保留位 bit14 / bit[7:0] 会被强制清零）
 * @return HDC1080_OK 成功；否则返回错误码
 */
int8_t HDC1080_WriteConfig(uint16_t cfg);

/**
 * @brief 设置温湿度测量分辨率（修改配置寄存器 bit10 / bit[9:8]）
 * @param t_res 温度分辨率：HDC1080_RES_14BIT / HDC1080_RES_11BIT（8bit 无效）
 * @param h_res 湿度分辨率：HDC1080_RES_14BIT / 11BIT / 8BIT
 * @return HDC1080_OK 成功；否则返回错误码
 */
int8_t HDC1080_SetResolution(HDC1080_Resolution_t t_res,
                             HDC1080_Resolution_t h_res);

/**
 * @brief 读取器件 ID (0xFF)，正常应返回 0x1050
 * @param dev_id 输出器件 ID
 * @return HDC1080_OK 成功；否则返回错误码
 */
int8_t HDC1080_ReadDeviceID(uint16_t *dev_id);

/**
 * @brief 读取厂商 ID (0xFE)，正常应返回 0x5449
 * @param manuf_id 输出厂商 ID
 * @return HDC1080_OK 成功；否则返回错误码
 */
int8_t HDC1080_ReadManufacturerID(uint16_t *manuf_id);

/**
 * @brief 读取 40-bit 唯一序列号（寄存器 0xFB/0xFC/0xFD）
 * @param serial 输出序列号（低 40 位有效）
 * @return HDC1080_OK 成功；否则返回错误码
 */
int8_t HDC1080_ReadSerialID(uint64_t *serial);

/**
 * @brief 单次测量温湿度（推荐使用）
 * @note  流程：确保 MODE=1 → 指针写 0x00 触发测量 → 按分辨率延时等待
 *        → 一次事务读回温度+湿度 → 换算实际值并填入 data
 * @param data 输出测量结果（含原始值与实际值）
 * @return HDC1080_OK 成功；HDC1080_ERR_PARAM / HDC1080_ERR_I2C 失败
 */
int8_t HDC1080_Measure(HDC1080_data_t *data);

/**
 * @brief 仅触发温湿度测量（写指针 0x00，不等待不读取）
 * @note  供需要自行控制时序的用户使用；触发后需等待转换完成再读取，
 *        转换期间勿向 0x00/0x01 写数据（会中止测量）
 * @return HDC1080_OK 成功；否则返回错误码
 */
int8_t HDC1080_Trigger(void);

/**
 * @brief 读取温湿度（需先触发测量并等待转换完成）
 * @note  前提：已调用 HDC1080_Trigger() 并等待足够转换时间（触发把指针置为 0x00）。
 * @note  ⚠ 实现上【不能】在读之前写指针寄存器：0x00/0x01 是"测量触发"寄存器，
 *        写它们会重新触发测量，本次读随即被 NACK（手册 Figure 13）。
 *        故本函数直接从当前指针发起读事务，共 4 字节：
 *        温度 MSB/LSB + 湿度 MSB/LSB。详见 hdc1080.c 内注释。
 * @param data 输出测量结果（含原始值与实际值）
 * @return HDC1080_OK 成功；HDC1080_ERR_PARAM / HDC1080_ERR_I2C 失败
 */
int8_t HDC1080_ReadTempHum(HDC1080_data_t *data);

/**
 * @brief 仅读取温度原始值（寄存器 0x00，2 字节）
 * @note  需先触发温度测量并等待转换完成（触发写指针 0x00，之后指针即为 0x00）。
 * @param raw 输出温度原始值（14bit 有效）
 * @return HDC1080_OK 成功；否则返回错误码
 */
int8_t HDC1080_ReadTempRaw(uint16_t *raw);

/**
 * @brief 仅读取湿度原始值（寄存器 0x01，2 字节）
 * @warning ⚠ 本函数当前不可用（恒返回 HDC1080_ERR_I2C）：读前写指针 0x01 会重新
 *          触发测量导致读被 NACK；又不能改为直读（触发温度后指针停在 0x00，
 *          直读会返回温度这一静默错误数据）。需先补湿度触发 API 才能正确实现。
 * @param raw 输出湿度原始值（14bit 有效）
 * @return HDC1080_OK 成功；否则返回错误码
 */
int8_t HDC1080_ReadHumRaw(uint16_t *raw);

/**
 * @brief 温度原始值换算实际值：T(°C) = (raw / 2^16) × 165 − 40
 * @param raw     温度原始值
 * @param temp_c  输出温度实际值（°C）
 */
void HDC1080_CalcTemp(uint16_t raw, float *temp_c);

/**
 * @brief 湿度原始值换算实际值：RH(%) = (raw / 2^16) × 100
 * @param raw    湿度原始值
 * @param hum_rh 输出湿度实际值（%RH）
 */
void HDC1080_CalcHum(uint16_t raw, float *hum_rh);

/**
 * @brief CRC-8 计算（TI HDC 系列约定：多项式 0x31，初值 0xFF，MSB 优先）
 * @note  HDC1080 数据手册图 13/14 的标准读帧不含 CRC 字节（与 HDC1000 不同），
 *        本函数供可选的数据完整性校验（如兼容 HDC1000 协议或自定义总线）。
 * @param buf 数据缓冲区
 * @param len 数据长度（字节）
 * @return CRC-8 校验值
 */
uint8_t HDC1080_CRC8(const uint8_t *buf, uint16_t len);

/**
 * @brief CRC-8 校验：比较计算值与给定 CRC 是否一致
 * @param buf 数据缓冲区
 * @param len 数据长度（字节）
 * @param crc 期望的 CRC-8 值
 * @return HDC1080_OK 校验通过；HDC1080_ERR_CRC 校验失败
 */
int8_t HDC1080_CRC_Check(const uint8_t *buf, uint16_t len, uint8_t crc);

#ifdef __cplusplus
}
#endif

#endif /* HDC1080_H */



#endif //INC_2_4G_HDC1080_H
