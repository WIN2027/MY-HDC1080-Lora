//
// Created by WIN on 2026/9/19.
//

#include "HDC1080.h"
/**
 ******************************************************************************
 * @file    HDC1080.c
 * @brief   HDC1080 数字温湿度传感器驱动 —— 源文件
 * @details 基于 TI HDC1080 数据手册 SNAS672 (ZHCSET2A) 实现：
 *          - I2C 8-bit 指针寄存器寻址，所有数据 MSB 优先
 *          - 温湿度顺序采集（配置寄存器 MODE=1，POR 默认即为此模式）
 *          - 单次测量流程：触发（指针写 0x00）→ 延时等待转换 → 一次事务读回
 *          - 原始值换算：T(°C) = (raw/2^16)*165 - 40；RH(%) = (raw/2^16)*100
 *
* @note   默认使用 STM32 HAL 硬件 I2C 阻塞式读写，通过 HDC1080_I2C_HANDLE
 *         宏指定 I2C 句柄（默认 hi2c1），HDC1080_I2C_TIMEOUT 指定超时。
 *         非 HAL 平台可直接修改本文件顶部的 HDC1080_I2C_Write /
 *         HDC1080_I2C_Read / HDC1080_DelayMs 三个函数。
 ******************************************************************************
 */
#include "i2c.h"    /* 提供 I2C 句柄 hi2c1 */
#include "HDC1080.h"

/*============================================================================
 * 内部静态函数声明
 *==========================================================================*/
static int8_t HDC1080_ReadReg16(uint8_t reg, uint16_t *val);   /* 读 16bit 寄存器 */
static int8_t HDC1080_WriteReg16(uint8_t reg, uint16_t val);   /* 写 16bit 寄存器 */

/*============================================================================
 * 硬件 I2C 底层实现（默认 STM32 HAL）
 *==========================================================================*/
int8_t HDC1080_I2C_Write(uint8_t dev_addr, uint8_t reg_addr,
                         const uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st;
    uint16_t addr = (uint16_t)(dev_addr << 1);   /* 转为 8-bit 地址 */

    if (len == 0U) {
        /* 仅写指针字节：START → W地址 → 指针字节 → STOP */
        st = HAL_I2C_Master_Transmit(&HDC1080_I2C_HANDLE, addr,
                                     &reg_addr, 1, HDC1080_I2C_TIMEOUT);
    } else {
        /* 写指针 + 数据：START → W地址 → 指针字节 → 数据 → STOP */
        st = HAL_I2C_Mem_Write(&HDC1080_I2C_HANDLE, addr, reg_addr,
                               I2C_MEMADD_SIZE_8BIT,
                               (uint8_t *)buf, len, HDC1080_I2C_TIMEOUT);
    }

    return (st == HAL_OK) ? HDC1080_OK : HDC1080_ERR_I2C;
}

int8_t HDC1080_I2C_Read(uint8_t dev_addr, uint8_t reg_addr,
                        uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st;
    uint16_t addr = (uint16_t)(dev_addr << 1);

    /* START → W地址 → 指针字节 → RESTART → R地址 → 读 len 字节 → STOP */
    st = HAL_I2C_Mem_Read(&HDC1080_I2C_HANDLE, addr, reg_addr,
                          I2C_MEMADD_SIZE_8BIT, buf, len, HDC1080_I2C_TIMEOUT);

    return (st == HAL_OK) ? HDC1080_OK : HDC1080_ERR_I2C;
}

void HDC1080_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

/*============================================================================
 * 内部静态函数实现
 *==========================================================================*/
/**
 * @brief 读取 16-bit 寄存器（内部函数）
 * @param reg 寄存器地址（指针值）
 * @param val 输出 16bit 值（MSB 优先，先收高字节再收低字节）
 * @return HDC1080_OK 成功；HDC1080_ERR_PARAM / HDC1080_ERR_I2C 失败
 */
static int8_t HDC1080_ReadReg16(uint8_t reg, uint16_t *val)
{
    int8_t  ret;
    uint8_t buf[2];

    if (val == NULL) {
        return HDC1080_ERR_PARAM;
    }

    /* 一次事务：写指针 reg → 重启读 2 字节（MSB 在前） */
    ret = HDC1080_I2C_Read(HDC1080_I2C_ADDR, reg, buf, 2);
    if (ret != HDC1080_OK) {
        return HDC1080_ERR_I2C;
    }

    *val = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    return HDC1080_OK;
}

/**
 * @brief 写 16-bit 寄存器（内部函数）
 * @param reg 寄存器地址（指针值）
 * @param val 待写入 16bit 值（MSB 优先，先发高字节再发低字节）
 * @return HDC1080_OK 成功；HDC1080_ERR_I2C 失败
 */
static int8_t HDC1080_WriteReg16(uint8_t reg, uint16_t val)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(val >> 8);   /* 高字节在前（MSB first） */
    buf[1] = (uint8_t)(val & 0xFF);

    return HDC1080_I2C_Write(HDC1080_I2C_ADDR, reg, buf, 2);
}

/*============================================================================
 * 公开 API 实现
 *==========================================================================*/
/**
 * @brief 初始化：等待上电就绪 + 校验器件 ID
 * @note  时序要求（手册 7.7 节）：VDD 稳定后器件最多需 15ms 才可进行测量，
 *        期间仅能读取序列号寄存器，因此此处先延时 15ms 再通信。
 */
int8_t HDC1080_Init(void)
{
    int8_t  ret;
    uint16_t dev_id = 0U;

    /* 1. 上电等待：确保器件完成 POR 进入可测量状态 */
    HDC1080_DelayMs(HDC1080_POWERUP_TIME_MS);

    /* 2. 读取器件 ID (0xFF)，正常为 0x1050。
     *    校验失败说明 I2C 不通或总线上不是 HDC1080。 */
    ret = HDC1080_ReadDeviceID(&dev_id);
    if (ret != HDC1080_OK) {
        return ret;
    }
    if (dev_id != HDC1080_DEVICE_ID) {
        return HDC1080_ERR_DEVICE;
    }

    return HDC1080_OK;
}

/**
 * @brief 软复位：写配置寄存器 RST 位 (bit15)
 * @note  RST 位写 1 触发复位后自清零；复位后所有寄存器恢复 POR 默认值
 *        （配置寄存器 = 0x1000），需重新配置分辨率等参数。
 *        手册未给出复位完成时间，这里按上电就绪时间 15ms 取安全余量。
 */
int8_t HDC1080_SoftReset(void)
{
    int8_t  ret;
    uint16_t cfg = 0U;

    /* 先读回当前配置，仅置位 RST，避免破坏其他配置位 */
    ret = HDC1080_ReadConfig(&cfg);
    if (ret != HDC1080_OK) {
        return ret;
    }

    cfg |= HDC1080_CFG_RST;
    ret = HDC1080_WriteConfig(cfg);
    if (ret != HDC1080_OK) {
        return ret;
    }

    /* 等待器件复位完成并重新就绪 */
    HDC1080_DelayMs(HDC1080_POWERUP_TIME_MS);
    return HDC1080_OK;
}

/**
 * @brief 读取配置寄存器 (0x02)
 */
int8_t HDC1080_ReadConfig(uint16_t *cfg)
{
    if (cfg == NULL) {
        return HDC1080_ERR_PARAM;
    }
    return HDC1080_ReadReg16(HDC1080_REG_CONFIG, cfg);
}

/**
 * @brief 写配置寄存器 (0x02)
 * @note  保留位 bit14 与 bit[7:0] 手册要求必须写 0，这里强制清零，防止误写。
 */
int8_t HDC1080_WriteConfig(uint16_t cfg)
{
    cfg &= ~HDC1080_CFG_RESERVED;   /* 清除保留位，避免非预期行为 */
    return HDC1080_WriteReg16(HDC1080_REG_CONFIG, cfg);
}

/**
 * @brief 设置温湿度分辨率
 * @note  bit10  温度分辨率：0=14bit / 1=11bit
 *        bit[9:8] 湿度分辨率：00=14bit / 01=11bit / 10=8bit
 */
int8_t HDC1080_SetResolution(HDC1080_Resolution_t t_res,
                             HDC1080_Resolution_t h_res)
{
    int8_t   ret;
    uint16_t cfg = 0U;

    if (t_res != HDC1080_RES_14BIT && t_res != HDC1080_RES_11BIT) {
        return HDC1080_ERR_PARAM;
    }
    if (h_res != HDC1080_RES_14BIT &&
        h_res != HDC1080_RES_11BIT &&
        h_res != HDC1080_RES_8BIT) {
        return HDC1080_ERR_PARAM;
        }

    ret = HDC1080_ReadConfig(&cfg);
    if (ret != HDC1080_OK) {
        return ret;
    }

    /* --- 温度分辨率（仅 14bit / 11bit 有效） --- */
    cfg &= ~HDC1080_CFG_TRES_MASK;
    if (t_res == HDC1080_RES_11BIT) {
        cfg |= HDC1080_CFG_TRES_11BIT;
    }

    /* --- 湿度分辨率（14bit / 11bit / 8bit） --- */
    cfg &= ~HDC1080_CFG_HRES_MASK;
    if (h_res == HDC1080_RES_11BIT) {
        cfg |= HDC1080_CFG_HRES_11BIT;
    } else if (h_res == HDC1080_RES_8BIT) {
        cfg |= HDC1080_CFG_HRES_8BIT;
    }

    return HDC1080_WriteConfig(cfg);
}

/**
 * @brief 读取器件 ID (0xFF)，正常返回 0x1050
 */
int8_t HDC1080_ReadDeviceID(uint16_t *dev_id)
{
    if (dev_id == NULL) {
        return HDC1080_ERR_PARAM;
    }
    return HDC1080_ReadReg16(HDC1080_REG_DEVICE_ID, dev_id);
}

/**
 * @brief 读取厂商 ID (0xFE)，正常返回 0x5449（TI）
 */
int8_t HDC1080_ReadManufacturerID(uint16_t *manuf_id)
{
    if (manuf_id == NULL) {
        return HDC1080_ERR_PARAM;
    }
    return HDC1080_ReadReg16(HDC1080_REG_MANUF_ID, manuf_id);
}

/**
 * @brief 读取 40-bit 唯一序列号
 * @note  寄存器布局：0xFB = SERIAL[40:25]（16bit）
 *                    0xFC = SERIAL[24:9]（16bit）
 *                    0xFD = SERIAL[8:0]（bit[15:7]，9bit）
 */
int8_t HDC1080_ReadSerialID(uint64_t *serial)
{
    int8_t  ret;
    uint16_t regs[3] = {0U, 0U, 0U};

    if (serial == NULL) {
        return HDC1080_ERR_PARAM;
    }

    ret = HDC1080_ReadReg16(HDC1080_REG_SERIAL_ID_1, &regs[0]);
    if (ret != HDC1080_OK) return ret;
    ret = HDC1080_ReadReg16(HDC1080_REG_SERIAL_ID_2, &regs[1]);
    if (ret != HDC1080_OK) return ret;
    ret = HDC1080_ReadReg16(HDC1080_REG_SERIAL_ID_3, &regs[2]);
    if (ret != HDC1080_OK) return ret;

    /* 按位域拼接 40-bit 序列号 */
    *serial = ((uint64_t)regs[0] << 25) |
              ((uint64_t)regs[1] << 9)  |
              ((uint64_t)(regs[2] >> 7));

    return HDC1080_OK;
}

/**
 * @brief 单次测量温湿度（推荐入口）
 * @note  完整时序（手册 8.5.1.3 节）：
 *        1) 确保配置寄存器 MODE=1（温湿度顺序采集，POR 默认即为 1）；
 *        2) 指针写 0x00 触发测量（先温度后湿度）；
 *        3) 等待转换完成：总时间 = 温度转换时间 + 湿度转换时间，
 *           与所选分辨率相关（14bit 约 13ms，11bit 约 8ms）；
 *        4) 从 0x00 起一次事务读 4 字节（温度 MSB/LSB + 湿度 MSB/LSB）；
 *        5) 换算实际值填入 data。
 */
int8_t HDC1080_Measure(HDC1080_data_t *data)
{
    int8_t   ret;
    uint16_t cfg = 0U;
    uint16_t wait_ms = 0U;

    if (data == NULL) {
        return HDC1080_ERR_PARAM;
    }

    /* 1. 读取配置，确认采集模式与分辨率 */
    ret = HDC1080_ReadConfig(&cfg);
    if (ret != HDC1080_OK) {
        return ret;
    }

    /* 2. 若当前为单通道模式（MODE=0），自动切换为温湿度顺序采集（MODE=1） */
    if ((cfg & HDC1080_CFG_MODE) == 0U) {
        cfg |= HDC1080_CFG_MODE;
        ret = HDC1080_WriteConfig(cfg);
        if (ret != HDC1080_OK) {
            return ret;
        }
    }

    /* 3. 触发测量：指针写 0x00（MODE=1 时依次触发温度→湿度转换） */
    ret = HDC1080_Trigger();
    if (ret != HDC1080_OK) {
        return ret;
    }

    /* 4. 按分辨率计算等待时间（转换时间见手册 7.5 节，宏取值已留余量）
     *    总等待 = 温度转换时间 + 湿度转换时间（顺序采集） */
    wait_ms = (cfg & HDC1080_CFG_TRES_MASK) ? HDC1080_CONV_TEMP_11BIT_MS
                                            : HDC1080_CONV_TEMP_14BIT_MS;
    switch (cfg & HDC1080_CFG_HRES_MASK) {
        case HDC1080_CFG_HRES_11BIT:
            wait_ms += HDC1080_CONV_HUM_11BIT_MS;
            break;
        case HDC1080_CFG_HRES_8BIT:
            wait_ms += HDC1080_CONV_HUM_8BIT_MS;
            break;
        default: /* 14bit */
            wait_ms += HDC1080_CONV_HUM_14BIT_MS;
            break;
    }
    HDC1080_DelayMs(wait_ms);

    /* 5. 读取温湿度并换算 */
    return HDC1080_ReadTempHum(data);
}

/**
 * @brief 仅触发测量（指针写 0x00）
 * @note  MODE=1 时写 0x00 依次触发温度与湿度测量；
 *        MODE=0 时写 0x00 仅触发温度测量（湿度需写 0x01）。
 *        触发后必须等待转换完成再读取；转换期间写 0x00/0x01 会中止测量。
 */
int8_t HDC1080_Trigger(void)
{
    /* len == 0 表示只发送"指针字节"（不携带数据），详见 HDC1080_I2C_Write 说明 */
    return HDC1080_I2C_Write(HDC1080_I2C_ADDR, HDC1080_REG_TEMPERATURE, NULL, 0U);
}

/**
 * @brief 读取温湿度（一次读事务取 4 字节）并换算实际值
 * @note  前提：已调用 HDC1080_Trigger() 触发测量并等待足够转换时间
 *        （触发会把指针寄存器置为 0x00）。
 * @note  ⚠ 本函数【不能】使用 HAL_I2C_Mem_Read / HDC1080_I2C_Read：
 *        它们读前必先写一次指针寄存器，而 0x00/0x01 正是"测量触发"寄存器
 *        （手册 8.5.1.3 步骤 2）。重新写 0x00 会再次触发测量，紧随其后的读
 *        因"寄存器内容尚未更新"被从机 NACK（手册 Figure 13），实测表现为
 *        恒返回 HDC1080_ERR_I2C。手册 8.5.1.2 只要求"需要改变指针值时"才写
 *        指针；触发步骤已把指针置为 0x00，故此处直接发起读事务即可。
 * @note  2026-09-19 在 STM32F411 + HDC1080 上以 A/B 受控实验验证（同一轮、
 *        同一触发、同一 14ms 延时、同样 4 字节，唯一差异是"读前是否写指针"）：
 *          A 写指针后读 → NACK；B 不写指针直读 → 成功，读回合理温湿度。
 */
int8_t HDC1080_ReadTempHum(HDC1080_data_t *data)
{
    uint8_t buf[4];

    if (data == NULL) {
        return HDC1080_ERR_PARAM;
    }

    /* 不写指针，直接从当前指针（触发后为 0x00）连续读 4 字节：
     * buf[0]=温度MSB  buf[1]=温度LSB  buf[2]=湿度MSB  buf[3]=湿度LSB
     * （器件指针在 0x00→0x01 之间自动递增，手册图 14）
     * HAL 约定 DevAddress 传 7-bit 地址左移 1 位，R/W 位由 HAL 内部置位。 */
    if (HAL_I2C_Master_Receive(&HDC1080_I2C_HANDLE,
                              (uint16_t)(HDC1080_I2C_ADDR << 1),
                              buf, 4U, HDC1080_I2C_TIMEOUT) != HAL_OK) {
        return HDC1080_ERR_I2C;
    }

    /* 拼接 16bit 原始值（低 2 位为 0，14bit 有效） */
    data->temp_raw = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    data->hum_raw  = ((uint16_t)buf[2] << 8) | (uint16_t)buf[3];

    /* 换算实际值 */
    HDC1080_CalcTemp(data->temp_raw, &data->temp_c);
    HDC1080_CalcHum(data->hum_raw, &data->hum_rh);

    return HDC1080_OK;
}

/**
 * @brief 仅读取温度原始值（寄存器 0x00）
 * @note  适用于 MODE=0 单通道测量；调用前需先触发温度测量并等待转换完成
 *        （触发写指针 0x00，之后指针即为 0x00）。
 * @note  同 HDC1080_ReadTempHum：不写指针、直接读，原因见该函数注释。
 */
int8_t HDC1080_ReadTempRaw(uint16_t *raw)
{
    uint8_t buf[2];

    if (raw == NULL) {
        return HDC1080_ERR_PARAM;
    }

    if (HAL_I2C_Master_Receive(&HDC1080_I2C_HANDLE,
                              (uint16_t)(HDC1080_I2C_ADDR << 1),
                              buf, 2U, HDC1080_I2C_TIMEOUT) != HAL_OK) {
        return HDC1080_ERR_I2C;
    }

    *raw = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    return HDC1080_OK;
}

/**
 * @brief 仅读取湿度原始值（寄存器 0x01）
 * @note  适用于 MODE=0 单通道测量；调用前需自行触发并等待转换完成
 * @warning ⚠ 本函数当前【不可用，恒返回 HDC1080_ERR_I2C】，保留原实现以待后续修正。
 *          原因同上：读前写指针 0x01 会重新触发测量，本次读随即被 NACK。
 *          但它【不能】简单改成"不写指针直读" —— 触发温度测量后指针停在 0x00，
 *          直读会返回【温度】而非湿度，属于静默的错误数据，比报错更危险。
 *          正确实现需要完整的"写指针 0x01（触发湿度测量）→ 等待 → 直读 2 字节"
 *          流程，而当前驱动未暴露湿度触发接口。需要使用单通道湿度测量时请先
 *          补一个湿度触发 API，再按上述流程实现本函数。
 */
int8_t HDC1080_ReadHumRaw(uint16_t *raw)
{
    int8_t  ret;
    uint8_t buf[2];

    if (raw == NULL) {
        return HDC1080_ERR_PARAM;
    }

    ret = HDC1080_I2C_Read(HDC1080_I2C_ADDR, HDC1080_REG_HUMIDITY, buf, 2);
    if (ret != HDC1080_OK) {
        return HDC1080_ERR_I2C;
    }

    *raw = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    return HDC1080_OK;
}

/**
 * @brief 温度原始值 → 实际值
 * @note  换算公式（手册 8.6.1 节）：
 *        T(°C) = (temp_raw / 2^16) × 165 − 40
 *        例：raw = 0x6A66 (27238) → (27238/65536)×165−40 ≈ 28.6°C
 */
void HDC1080_CalcTemp(uint16_t raw, float *temp_c)
{
    if (temp_c != NULL) {
        *temp_c = ((float)raw / 65536.0f) * 165.0f - 40.0f;
    }
}

/**
 * @brief 湿度原始值 → 实际值
 * @note  换算公式（手册 8.6.2 节）：
 *        RH(%) = (hum_raw / 2^16) × 100
 *        例：raw = 0x4B1A (19226) → (19226/65536)×100 ≈ 29.3%RH
 */
void HDC1080_CalcHum(uint16_t raw, float *hum_rh)
{
    if (hum_rh != NULL) {
        *hum_rh = ((float)raw / 65536.0f) * 100.0f;
    }
}

/**
 * @brief CRC-8 计算
 * @note  采用 TI HDC 系列（HDC1000/HDC2010 等）约定：
 *        - 多项式：x^8 + x^5 + x^4 + 1，即 0x31
 *        - 初值 0xFF，MSB 优先逐位处理，无最终异或
 *        注意：HDC1080 数据手册图 13/14 的标准读帧不包含 CRC 字节
 *        （与 HDC1000 不同），因此驱动默认读路径不校验 CRC；
 *        本函数供用户做可选的数据完整性校验。
 */
uint8_t HDC1080_CRC8(const uint8_t *buf, uint16_t len)
{
    uint8_t  crc = 0xFFU;   /* 初值 0xFF */
    uint16_t i;
    uint8_t  bit;

    if (buf == NULL) {
        return crc;
    }

    for (i = 0; i < len; i++) {
        crc ^= buf[i];
        for (bit = 0; bit < 8; bit++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ 0x31U);   /* 多项式 0x31 */
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

/**
 * @brief CRC-8 校验
 * @return HDC1080_OK 校验通过；HDC1080_ERR_CRC 校验失败
 */
int8_t HDC1080_CRC_Check(const uint8_t *buf, uint16_t len, uint8_t crc)
{
    if (HDC1080_CRC8(buf, len) == crc) {
        return HDC1080_OK;
    }
    return HDC1080_ERR_CRC;
}

/* 文件结束 */
