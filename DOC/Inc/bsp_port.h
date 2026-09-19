/**
 ******************************************************************************
 * @file    bsp_port.h
 * @brief   SX1280 / HDC1080 硬件适配层（BSP Port）—— 接口声明
 *
 * @details 本文件声明 sx1280 / hdc1080 两个模块要求的全部平台移植函数
 *          （对应 模块/README.md 第 4 节"移植步骤"中的 5 + 3 个弱函数）。
 *          实现见 bsp_port.c，原型一致性由 bsp_port_check.c 在编译期强制校验。
 *
 * @warning 本头文件刻意 **不包含** sx1280.h / hdc1080.h，实现文件同样不包含。
 *          原因（已用本工程工具链 arm-none-eabi-gcc 13.3.1 实测）：
 *          这两个头文件把移植函数声明为 __attribute__((weak))，而 GCC 会把
 *          **同一编译单元内的定义**也一并标记为 weak 符号（实测 nm 输出为 'W'）。
 *          于是"覆盖用"的实现退化成弱符号，与 sx1280.c / hdc1080.c 底部的
 *          占位弱实现同时存在，链接器按目标文件出现顺序择一 —— 本工程
 *          CMakeLists 中 DOC/Src/sx1280.c 排在 bsp_port.c 之前，极可能链接到
 *          "永远返回失败"的占位实现，表现为 SX1280_Init() 返回 -2。
 *          这里改为重复声明一份不带 weak 的原型，使定义成为强符号（'T'），
 *          链接器必然选用，与链接顺序无关。
 ******************************************************************************
 */
#ifndef BSP_PORT_H
#define BSP_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * 返回值：必须与驱动内部错误码一致（由 bsp_port_check.c 静态断言校验）
 *==========================================================================*/
#define BSP_RET_OK              (0)     /*!< 成功（== SX1280_OK / HDC1080_OK） */
#define BSP_HDC1080_ERR_I2C     (-1)    /*!< == HDC1080_ERR_I2C */
#define BSP_SX1280_ERR_SPI      (-2)    /*!< == SX1280_ERR_SPI  */

/*============================================================================
 * 配置
 *==========================================================================*/
#define BSP_SPI_TIMEOUT_MS      (100U)  /*!< HAL SPI 阻塞超时（ms） */
#define BSP_I2C_TIMEOUT_MS      (100U)  /*!< HAL I2C 阻塞超时（ms） */
#define BSP_SX1280_FRAME_MAX    (260U)  /*!< 驱动单帧上限 259 字节 + 余量 */
#define BSP_NSS_PULSE_US        (2U)    /*!< NSS 低电平最小脉宽（手册 9.3.3 tPulse ≥2µs） */

/*============================================================================
 * BSP 初始化（使能 DWT 周期计数器，供微秒延时使用）
 * @note 必须在任何 SX1280_* 调用前执行一次；可重复调用。
 *==========================================================================*/
void BSP_Port_Init(void);

/*============================================================================
 * I2C 诊断与总线恢复（HDC1080 通信异常时定位用）
 *==========================================================================*/

/**
 * @brief 最近一次 HAL I2C 调用的结果快照
 * @note  字段语义（对照 STM32F4 HAL）：
 *        hal_status : HAL_StatusTypeDef —— 0=HAL_OK 1=HAL_ERROR 2=HAL_BUSY 3=HAL_TIMEOUT
 *        error_code : hi2c.ErrorCode —— bit0=BERR bit1=ARLO **bit2=AF(NACK 无应答)**
 *                     bit3=OVR bit5=TIMEOUT bit6=SIZE
 *        state      : hi2c.State —— 0x20=READY 0x21=BUSY_TX 0x22=BUSY_RX
 *                     0x24=TIMEOUT 0x25=ERROR
 *        "总线被拉死/BUSY 标志卡住"的典型特征：hal_status=2(HAL_BUSY)、
 *        error_code=0、state=0x20(READY)，且总线上完全没有时钟输出。
 */
typedef struct {
    uint8_t hal_status;   /*!< HAL_StatusTypeDef */
    uint8_t error_code;   /*!< hi2c1.ErrorCode 低 8 位 */
    uint8_t state;        /*!< hi2c1.State 低 8 位 */
    uint8_t reserved;
} BSP_I2cDiag_t;

/** @brief 取出最近一次 I2C 调用的诊断快照（diag 不可为 NULL） */
void BSP_I2C_GetDiag(BSP_I2cDiag_t *diag);

/**
 * @brief I2C 总线恢复：把 SCL/SDA 切成 GPIO 手动发 9 个时钟 + 一个 STOP，
 *        然后复位 I2C 外设并恢复 AF 开漏配置
 * @note  用途：
 *        1) 从机异常把 SDA 拉死时，主机此后永远等不到总线空闲
 *           （HAL 等 BUSY 标志 25ms 后直接返回 HAL_BUSY，一个时钟都不发）；
 *        2) 上电时序/热插拔导致外设状态与总线不同步。
 *        返回后 hi2c1 已重新初始化，可直接继续通信；整段耗时 < 1ms。
 * @warning 仅可在总线空闲（无进行中的传输）时调用。
 */
void BSP_I2C_BusRecover(void);

/*============================================================================
 * SX1280 移植接口（覆盖 sx1280.c 底部同名声明的弱函数）
 *==========================================================================*/
int8_t  SX1280_SPI_Transceive(const uint8_t *tx, uint8_t *rx,
                              uint16_t len);
uint8_t SX1280_ReadBusy(void);
void    SX1280_SetResetPin(uint8_t level);
void    SX1280_DelayUs(uint32_t us);
void    SX1280_DelayMs(uint32_t ms);

/*============================================================================
 * HDC1080 移植接口（覆盖 hdc1080.c 底部同名声明的弱函数）
 *==========================================================================*/
int8_t HDC1080_I2C_Write(uint8_t dev_addr, uint8_t reg_addr,
                         const uint8_t *buf, uint16_t len);
int8_t HDC1080_I2C_Read(uint8_t dev_addr, uint8_t reg_addr,
                        uint8_t *buf, uint16_t len);
void   HDC1080_DelayMs(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_PORT_H */
