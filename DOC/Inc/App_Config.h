/**
 * @file    App_Config.h
 * @brief   应用层配置：节点角色、空中帧格式、接收端上报格式
 *
 * 两块完全相同的板子共用这一份代码，靠 NODE_ROLE 宏区分业务：
 *   NODE_ROLE_TX : 采集 HDC1080 → 本地串口遥测(0x18) + 经 SX1280 发送
 *   NODE_ROLE_RX : SX1280 连续接收 → 校验包头 → 串口上报(0x19，含 RSSI/SNR)
 * 两套业务代码都会编译进固件，用 #if 选择运行哪一支。
 *
 * 切换方式（二选一）：
 *   1) 直接改下面 NODE_ROLE 的默认值；
 *   2) 构建时覆盖，例如 -DNODE_ROLE=NODE_ROLE_RX。
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*============================================================================
 * 1. 节点角色
 *==========================================================================*/
#define NODE_ROLE_TX  1
#define NODE_ROLE_RX  2

#ifndef NODE_ROLE
#define NODE_ROLE  NODE_ROLE_TX      /* ← 接收端改成 NODE_ROLE_RX */
#endif

#if (NODE_ROLE != NODE_ROLE_TX) && (NODE_ROLE != NODE_ROLE_RX)
#error "NODE_ROLE 必须是 NODE_ROLE_TX 或 NODE_ROLE_RX"
#endif

/*============================================================================
 * 2. 空中帧格式（发射端 → 接收端），共 9 字节
 *    [0]  0xA5     帧头 0
 *    [1]  0x5A     帧头 1
 *    [2]  status   0 = 测量成功；非 0 = HDC1080_Status_t 返回码
 *    [3]  seqL     序号低字节
 *    [4]  seqH     序号高字节
 *    [5]  tempL    温度 ×100 低字节（int16 小端）
 *    [6]  tempH
 *    [7]  humL     湿度 ×100 低字节（uint16 小端）
 *    [8]  humH
 * 空口完整性由 SX1280 的 LoRa 包 CRC（硬件）保证；此处再放 2 字节包头，
 * 供接收端快速识别/筛帧，也便于以后加节点 ID。
 *==========================================================================*/
#define AIR_HDR0          0xA5u
#define AIR_HDR1          0x5AU
#define AIR_HDR_LEN       2u
#define AIR_TELEMETRY_LEN 7u
#define AIR_FRAME_LEN     (AIR_HDR_LEN + AIR_TELEMETRY_LEN)   /* 9 */

/*============================================================================
 * 3. 接收端串口上报格式（CMD = 0x19），共 11 字节
 *    [0]     status  0 = 收包并校验通过；非 0 = SX1280 错误码/格式校验失败
 *    [1..7]  空中遥测 7 字节原样透传（status+seq+temp+hum）
 *    [8..9]  rssi    dBm（int16 小端，负值）
 *    [10]    snr     dB（int8）
 *==========================================================================*/
#define RX_REPORT_CMD   0x19u
#define RX_REPORT_LEN   11u

/*============================================================================
 * 4. 一次性上报命令码（沿用栈溢出上报 0xE0 的做法）
 *==========================================================================*/
#define SX_INIT_CMD     0xE1u   /* 净荷 1 字节：MYSX1280_Init() 的返回码 */

/*============================================================================
 * 5. SX1280 任务参数
 *==========================================================================*/
#define SX1280_TASK_STACK     1536u   /* 字节。要跑 HAL_SPI_TransmitReceive 等较深调用链 */
#define SX1280_TX_QUEUE_DEPTH 4u      /* 待发空中帧队列深度（1 帧/秒，足够） */
#define SX1280_POLL_MS        100u    /* DIO 漏中断时的兜底扫描周期 */

#endif /* APP_CONFIG_H */
