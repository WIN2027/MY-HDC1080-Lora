/**
 * @file    serial_module.h
 * @brief   通用串口收发外挂模块（DMA + 空闲中断收帧 / CRC8 校验 / 命令分发 / 队列化发送）
 *
 * @note 设计目标
 *      - 与业务完全解耦：不依赖任何具体工程，拷贝 4 个文件即可挂载；
 *      - 支持多串口实例（数量由配置文件决定）；
 *      - RTOS / 裸机双模式（SERIAL_MODULE_USE_OS2 切换）；
 *      - 全静态内存，无 malloc；发送/接收队列可在任意上下文调用（中断安全）；
 *      - 帧协议与 FOC 工程现有上位机协议一致，上位机软件可直接复用。
 *
 * @brief 集成步骤见 README.md
 */
#ifndef SERIAL_MODULE_H
#define SERIAL_MODULE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "serial_module_cfg.h"
#include SERIAL_MODULE_HAL_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

/* ================= 帧协议常量 ================= */
#define SM_SOF                  0x5Au    /* 帧头 */
#define SM_FRAME_OVERHEAD       4u       /* SOF + CMD + LEN + CRC8 */
#define SM_PAYLOAD_MAX          SERIAL_MODULE_PAYLOAD_MAX
/* 模块保留应答码（与 FOC 工程保持一致） */
#define SM_CMD_ACK_OK           0x80u    /* 通用成功应答 */
#define SM_CMD_ERR_CRC          0x81u    /* CRC 校验错误应答 */
#define SM_CMD_ERR_INVALID_CMD  0x82u    /* 无效命令应答 */

/**
 * 命令处理回调（在模块调度上下文中执行：RTOS=模块工作线程，裸机=调用 Poll 的上下文）
 * @param port  端口索引（应答时回传给 SerialModule_Send）
 * @param cmd   命令码
 * @param data  净荷指针（帧内数据，处理期间有效）
 * @param len   净荷长度
 */
typedef void (*SerialModule_Handler_t)(uint8_t port, uint8_t cmd,
                                       const uint8_t *data, uint8_t len);

/** 原始帧旁路钩子：所有校验通过的帧都会回调（在命令分发之前），可为 NULL */
typedef SerialModule_Handler_t SerialModule_FrameCb_t;

/** 端口描述表（由应用层提供，SerialModule_Init 时传入） */
typedef struct {
    UART_HandleTypeDef   *huart;     /**< HAL 串口句柄（必须已由 CubeMX/HAL 初始化） */
    const char           *name;      /**< 端口名（仅调试标识用，可 NULL）            */
    SerialModule_FrameCb_t frame_cb; /**< 原始帧钩子（可 NULL）                      */
} SerialModule_PortCfg_t;

/** 收发统计计数 */
typedef struct {
    uint32_t rx_frames;        /**< 正确接收帧数             */
    uint32_t rx_crc_errors;    /**< CRC 校验失败次数         */
    uint32_t rx_dropped;       /**< 接收队列满丢弃的帧数     */
    uint32_t tx_sent;          /**< 实际发出帧数             */
    uint32_t tx_dropped;       /**< 发送队列满丢弃的帧数     */
    uint32_t tx_errors;        /**< HAL 发送启动失败的次数   */
} SerialModule_Stats_t;

/* ================= API ================= */

/**
 * @brief  初始化模块并启动所有端口的 DMA 接收
 * @param  ports      端口描述表（数组内容会被引用，请保证生命周期覆盖整个运行期）
 * @param  port_count 端口数量（<= SERIAL_MODULE_PORT_MAX）
 * @retval true 成功；false 参数错误或硬件启动失败
 * @note   RTOS 模式下须在 osKernelInitialize() 之后调用（内部会创建调度线程），
 *         可以在 osKernelStart() 之前也可以之后。
 */
bool SerialModule_Init(const SerialModule_PortCfg_t *ports, uint8_t port_count);

/** @brief 注册命令处理函数（重复注册同一命令码会覆盖旧函数） */
bool SerialModule_RegisterCmd(uint8_t cmd, SerialModule_Handler_t handler);

/** @brief 注销命令处理函数 */
void SerialModule_UnregisterCmd(uint8_t cmd);

/**
 * @brief  发送一帧（任意上下文可调用：任务/中断均可）
 * @param  port 端口索引 [0, port_count)
 * @param  cmd  命令码
 * @param  data 净荷（可 NULL，此时 len 必须为 0）；函数内部会拷贝，调用后缓冲可立即释放
 * @param  len  净荷长度（<= SM_PAYLOAD_MAX）
 * @retval true 已入队；false 参数错误或队列满（不会阻塞）
 */
bool SerialModule_Send(uint8_t port, uint8_t cmd,
                       const uint8_t *data, uint8_t len);

/** @brief 获取端口统计信息（快照） */
bool SerialModule_GetStats(uint8_t port, SerialModule_Stats_t *stats);

/**
 * @brief  扫描所有端口：分发已收到的帧、启动待发的帧
 * @note   裸机模式必须在主循环中周期调用（如每 1~10ms）；
 *         RTOS 模式由模块内部线程自动完成，无需调用（但手动调用也无害）。
 */
void SerialModule_Poll(void);

/* ============ 中断胶水接口（从用户 HAL 回调转发进来，每个工程只写一次） ============ */
void SerialModule_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size); /**< HAL_UARTEx_RxEventCallback */
void SerialModule_OnTxCplt(UART_HandleTypeDef *huart);                 /**< HAL_UART_TxCpltCallback    */
void SerialModule_OnError(UART_HandleTypeDef *huart);                  /**< HAL_UART_ErrorCallback     */

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_MODULE_H */
