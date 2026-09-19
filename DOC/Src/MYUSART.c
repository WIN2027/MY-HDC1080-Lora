/**
 * @file    MYUSART.c
 * @brief   通用串口收发外挂模块 —— 实现
 *
 * 运行模型
 * ========
 *  接收链路（每端口独立）：
 *    UART 空闲中断 --DMA--> dma_buf --字节流状态机--> 校验通过 --> 入"待处理帧环"
 *    调度上下文(RTOS线程/裸机Poll) 从环中取出 --> 帧钩子 + 命令分发
 *
 *  发送链路（每端口独立）：
 *    SerialModule_Send() 拷贝入队(任意上下文安全) --> 空闲则立即启动DMA，
 *    忙碌则由 发送完成中断 自动接续下一帧；队列满丢弃并计数。
 *
 * 关键设计点
 * =========
 *  1. 每次(重)启动接收后立即关闭 DMA 半传输(HT)中断：
 *     HT 事件会把缓冲区前半段误当成完整帧（这是实际项目踩过的坑）。
 *  2. 帧解析为增量式状态机：半帧状态跨 DMA 事件保留，
 *     天然支持一帧被拆多次送达、一帧数据中夹带多个帧、垃圾流自动重同步。
 *  3. 所有跨上下文共享的队列操作都在关中断临界区内完成（PRIMASK 级，微秒级窗口），
 *     不依赖 RTOS 也能保证中断安全。
 *  4. 发送使用独立 staging 缓冲：弹出队列与启动 DMA 分离，杜绝环形槽位被生产者复用的竞争。
 *  5. 全静态内存、无阻塞调用、无 HAL_MAX_DELAY。
 */
#include "MYUSART.h"
#include <string.h>

/* ================= 内部常量 ================= */
#define SM_ST_SOF   0u     /* 等待帧头       */
#define SM_ST_CMD   1u     /* 已收 SOF，等CMD*/
#define SM_ST_LEN   2u     /* 已收 CMD，等LEN*/
#define SM_ST_DATA  3u     /* 收净荷中       */
#define SM_ST_CRC   4u     /* 等CRC          */

#define SM_HDR_TMP_SIZE   (2u + SM_PAYLOAD_MAX)               /* CMD+LEN+DATA 暂存 */
#define SM_TX_FRM_SIZE    (SM_FRAME_OVERHEAD + SM_PAYLOAD_MAX)

#if SERIAL_MODULE_USE_OS2
#define SM_BIT_SWEEP      0x00000001u
static void sm_worker(void *argument);   /* 前置声明：Init 中创建线程用 */
#endif

/* ================= 类型定义 ================= */
typedef struct {
    const SerialModule_PortCfg_t *cfg;

    /* ---- 接收 ---- */
    uint8_t  dma_buf[SERIAL_MODULE_RX_DMA_BUF];
    uint8_t  pstate;                                   /* 解析状态机状态        */
    uint8_t  hdr_tmp[SM_HDR_TMP_SIZE];                 /* CMD@0 LEN@1 DATA@2..  */
    uint8_t  hdr_idx;                                  /* 净荷已收字节数        */

    uint8_t  pend_cmd [SERIAL_MODULE_RX_DEPTH];
    uint8_t  pend_len [SERIAL_MODULE_RX_DEPTH];
    uint8_t  pend_data[SERIAL_MODULE_RX_DEPTH][SM_PAYLOAD_MAX];
    volatile uint8_t pend_head;                        /* 队首下标              */
    volatile uint8_t pend_count;                       /* 队列元素个数          */

    /* ---- 发送 ---- */
    uint8_t  tx_q[SERIAL_MODULE_TX_DEPTH][SM_TX_FRM_SIZE];
    uint8_t  tx_len[SERIAL_MODULE_TX_DEPTH];
    uint8_t  tx_head;
    uint8_t  tx_count;
    uint8_t  tx_stage[SM_TX_FRM_SIZE];                 /* DMA 专用暂存          */
    volatile bool tx_busy;                             /* 有传输正在进行        */
    volatile bool kick_pending;                        /* 有帧等待启动发送      */

    SerialModule_Stats_t stats;
} sm_port_t;

typedef struct {
    uint8_t                  cmd;
    bool                     used;
    SerialModule_Handler_t   handler;
} sm_cmd_entry_t;

/* ================= 模块级存储 ================= */
static sm_port_t      m_ports[SERIAL_MODULE_PORT_MAX];
static uint8_t        m_port_cnt = 0;
static bool           m_init     = false;
static sm_cmd_entry_t m_cmds[SERIAL_MODULE_CMD_MAX];

#if SERIAL_MODULE_USE_OS2
static osThreadId_t s_worker = NULL;
static const osThreadAttr_t s_task_attr = {
    .name       = SERIAL_MODULE_TASK_NAME,
    .stack_size = SERIAL_MODULE_TASK_STACK,
    .priority   = SERIAL_MODULE_TASK_PRIO,
};
#endif

/* ================= 底层工具 ================= */

/*! 临界区：保存/恢复 PRIMASK，任务与中断上下文均可用，窗口极短 */
static inline uint32_t sm_crit_enter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}
static inline void sm_crit_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/*! 当前是否处于中断上下文 */
static inline bool sm_in_isr(void)
{
    return (__get_IPSR() != 0U);
}

/*! CRC8：多项式 0x07，初值 0xFF，计算范围 CMD..DATA（兼容 FOC 工程上位机协议） */
static uint8_t sm_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFFu;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8u; b++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

#if SERIAL_MODULE_USE_OS2
/*! 唤醒调度线程（可在中断中调用）；裸机模式为空操作，由 Poll() 兜底 */
static void sm_wake(void)
{
    if (s_worker != NULL) {
        (void)osThreadFlagsSet(s_worker, SM_BIT_SWEEP);
    }
}
#else
#define sm_wake()   ((void)0)
#endif

/*! 根据句柄查找端口索引，找不到返回 0xFF */
static uint8_t sm_find_port(const UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < m_port_cnt; i++) {
        if (m_ports[i].cfg->huart->Instance == huart->Instance) { return i; }
    }
    return 0xFFu;
}

/*! (重)启动某端口 DMA 空闲接收；并关闭 HT 中断 */
static bool sm_restart_rx(sm_port_t *p)
{
    UART_HandleTypeDef *h = p->cfg->huart;
    if (HAL_UARTEx_ReceiveToIdle_DMA(h, p->dma_buf, SERIAL_MODULE_RX_DMA_BUF) != HAL_OK) {
        /* 接收状态机可能卡死：强制中止后重试一次 */
        (void)HAL_UART_AbortReceive(h);
        if (HAL_UARTEx_ReceiveToIdle_DMA(h, p->dma_buf, SERIAL_MODULE_RX_DMA_BUF) != HAL_OK) {
            return false;
        }
    }
    /* 关键步骤：HAL 每次启动都会重新使能 HT 中断，必须再次关闭 */
    __HAL_DMA_DISABLE_IT(h->hdmarx, DMA_IT_HT);
    return true;
}

/* ================= 接收：解析与入队 ================= */

/*! 将一个校验通过的帧推入待处理环（任意上下文安全） */
static void sm_deliver(sm_port_t *p, uint8_t cmd, const uint8_t *data, uint8_t len)
{
    bool ok = false;
    uint32_t pm = sm_crit_enter();
    if (p->pend_count < SERIAL_MODULE_RX_DEPTH) {
        uint8_t slot = (uint8_t)((p->pend_head + p->pend_count) % SERIAL_MODULE_RX_DEPTH);
        p->pend_cmd[slot] = cmd;
        p->pend_len[slot] = len;
        if (len > 0u) { memcpy(p->pend_data[slot], data, len); }
        p->pend_count++;
        ok = true;
    }
    sm_crit_exit(pm);

    if (ok) {
        p->stats.rx_frames++;
        sm_wake();                      /* 尽快通知调度上下文取走处理 */
    } else {
        p->stats.rx_dropped++;          /* 队列满：丢新帧，由统计暴露 */
    }
}

/*! 帧解析状态机：单字节推进（半帧状态保存在端口结构内，可跨事件续传） */
static void sm_step(sm_port_t *p, uint8_t byte)
{
    switch (p->pstate) {

    case SM_ST_SOF:
        if (byte == SM_SOF) { p->pstate = SM_ST_CMD; }
        break;

    case SM_ST_CMD:
        p->hdr_tmp[0] = byte;
        p->pstate = SM_ST_LEN;
        break;

    case SM_ST_LEN:
        if (byte > SM_PAYLOAD_MAX) {
            p->pstate = SM_ST_SOF;      /* 非法长度：回到帧头搜索态自动重同步 */
        } else {
            p->hdr_tmp[1] = byte;
            p->hdr_idx    = 0u;
            p->pstate = (byte == 0u) ? SM_ST_CRC : SM_ST_DATA;
        }
        break;

    case SM_ST_DATA:
        p->hdr_tmp[2u + p->hdr_idx] = byte;
        p->hdr_idx++;
        if (p->hdr_idx >= p->hdr_tmp[1]) { p->pstate = SM_ST_CRC; }
        break;

    case SM_ST_CRC:
    default:
        if (sm_crc8(p->hdr_tmp, (uint8_t)(2u + p->hdr_tmp[1])) == byte) {
            sm_deliver(p, p->hdr_tmp[0], &p->hdr_tmp[2], p->hdr_tmp[1]);
        } else {
            p->stats.rx_crc_errors++;
#if SERIAL_MODULE_AUTO_NACK
            (void)SerialModule_Send(
                (uint8_t)(p - &m_ports[0]), SM_CMD_ERR_CRC, NULL, 0);
#endif
        }
        p->pstate = SM_ST_SOF;          /* 一帧结束（无论成败）都重新找帧头 */
        break;
    }
}

/* ================= 发送 ================= */

/*! 若端口空闲且有排队帧，则弹出一帧到 staging 并启动 DMA（任意上下文安全） */
static void sm_tx_kick(sm_port_t *p)
{
    bool     start  = false;
    uint16_t nbytes = 0u;

    uint32_t pm = sm_crit_enter();
    if (p->kick_pending && !p->tx_busy && (p->tx_count > 0u)) {
        uint8_t slot = p->tx_head;
        nbytes = p->tx_len[slot];
        memcpy(p->tx_stage, p->tx_q[slot], nbytes);   /* 先拷出，防环形槽位被复用 */
        p->tx_head = (uint8_t)((slot + 1u) % SERIAL_MODULE_TX_DEPTH);
        p->tx_count--;
        p->tx_busy = true;
        p->kick_pending = false;
        start = true;
    }
    sm_crit_exit(pm);

    if (start) {
        if (HAL_UART_Transmit_DMA(p->cfg->huart, p->tx_stage, nbytes) != HAL_OK) {
            pm = sm_crit_enter();
            p->tx_busy = false;
            p->stats.tx_errors++;
            sm_crit_exit(pm);
        }
    }
}

/*! 内部入队发送（SerialModule_Send 与自动应答共用） */
static bool sm_send_internal(uint8_t port, sm_port_t *p,
                             uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t frame[SM_TX_FRM_SIZE];
    frame[0] = SM_SOF;
    frame[1] = cmd;
    frame[2] = len;
    if (len > 0u) { memcpy(&frame[3], data, len); }
    frame[3u + len] = sm_crc8(&frame[1], (uint8_t)(2u + len));

    const uint8_t total = (uint8_t)(SM_FRAME_OVERHEAD + len);
    bool need_kick = false;

    uint32_t pm = sm_crit_enter();
    if (p->tx_count >= SERIAL_MODULE_TX_DEPTH) {
        p->stats.tx_dropped++;
        sm_crit_exit(pm);
        return false;                   /* 队列满：不阻塞，直接丢弃 */
    }
    uint8_t slot = (uint8_t)((p->tx_head + p->tx_count) % SERIAL_MODULE_TX_DEPTH);
    memcpy(p->tx_q[slot], frame, total);
    p->tx_len[slot] = total;
    p->tx_count++;
    if (!p->tx_busy && !p->kick_pending) {
        p->kick_pending = true;
        need_kick = true;
    }
    sm_crit_exit(pm);

    if (need_kick) {
        if (sm_in_isr()) {
            sm_wake();                  /* 中断里只做标记，交给调度线程/Poll 启动 */
        } else {
            sm_tx_kick(p);              /* 任务上下文直接启动，最低延迟 */
        }
    }
    return true;
}

/* ================= 调度：分发待处理帧 ================= */

static SerialModule_Handler_t sm_lookup(uint8_t cmd)
{
    for (uint8_t i = 0; i < SERIAL_MODULE_CMD_MAX; i++) {
        if (m_cmds[i].used && (m_cmds[i].cmd == cmd)) { return m_cmds[i].handler; }
    }
    return NULL;
}

/*! 取出所有待处理帧并分发（须在任务上下文调用） */
static void sm_dispatch(sm_port_t *p, uint8_t port_idx)
{
    uint8_t buf[SM_PAYLOAD_MAX];
    uint8_t cmd, len;

    for (;;) {
        uint32_t pm = sm_crit_enter();
        if (p->pend_count == 0u) {
            sm_crit_exit(pm);
            break;
        }
        uint8_t slot = p->pend_head;
        cmd = p->pend_cmd[slot];
        len = p->pend_len[slot];
        if (len > 0u) { memcpy(buf, p->pend_data[slot], len); }  /* 先拷贝再出队 */
        p->pend_head = (uint8_t)((slot + 1u) % SERIAL_MODULE_RX_DEPTH);
        p->pend_count--;
        sm_crit_exit(pm);

        /* 1. 原始帧旁路钩子 */
        if (p->cfg->frame_cb != NULL) {
            p->cfg->frame_cb(port_idx, cmd, buf, len);
        }
        /* 2. 命令分发表 */
        SerialModule_Handler_t fn = sm_lookup(cmd);
        if (fn != NULL) {
            fn(port_idx, cmd, buf, len);
        }
#if SERIAL_MODULE_AUTO_NACK
        else {
            (void)SerialModule_Send(port_idx, SM_CMD_ERR_INVALID_CMD, NULL, 0);
        }
#endif
    }
}

/* ================= 对外 API ================= */

bool SerialModule_Init(const SerialModule_PortCfg_t *ports, uint8_t port_count)
{
    if (m_init)                          { return false; }
    if (ports == NULL)                   { return false; }
    if ((port_count == 0u) ||
        (port_count > SERIAL_MODULE_PORT_MAX)) { return false; }

    memset(m_ports, 0, sizeof(m_ports));
    memset(m_cmds,  0, sizeof(m_cmds));

    m_port_cnt = port_count;
    for (uint8_t i = 0; i < port_count; i++) {
        if (ports[i].huart == NULL) { m_port_cnt = 0; return false; }
        m_ports[i].cfg = &ports[i];
    }

#if SERIAL_MODULE_USE_OS2
    /* 调度线程先建好，避免最早的接收事件丢失唤醒 */
    s_worker = osThreadNew(sm_worker, NULL, &s_task_attr);
    if (s_worker == NULL) { m_port_cnt = 0; return false; }
#endif

    for (uint8_t i = 0; i < port_count; i++) {
        if (!sm_restart_rx(&m_ports[i])) {
            m_port_cnt = 0;
            return false;                /* 串口未初始化或硬件异常 */
        }
    }

    m_init = true;
    return true;
}

bool SerialModule_RegisterCmd(uint8_t cmd, SerialModule_Handler_t handler)
{
    if (!m_init || (handler == NULL)) { return false; }

    uint32_t pm = sm_crit_enter();
    /* 已注册过：覆盖 */
    for (uint8_t i = 0; i < SERIAL_MODULE_CMD_MAX; i++) {
        if (m_cmds[i].used && (m_cmds[i].cmd == cmd)) {
            m_cmds[i].handler = handler;
            sm_crit_exit(pm);
            return true;
        }
    }
    /* 找空位 */
    for (uint8_t i = 0; i < SERIAL_MODULE_CMD_MAX; i++) {
        if (!m_cmds[i].used) {
            m_cmds[i].cmd     = cmd;
            m_cmds[i].handler = handler;
            m_cmds[i].used    = true;
            sm_crit_exit(pm);
            return true;
        }
    }
    sm_crit_exit(pm);
    return false;                        /* 表满 */
}

void SerialModule_UnregisterCmd(uint8_t cmd)
{
    uint32_t pm = sm_crit_enter();
    for (uint8_t i = 0; i < SERIAL_MODULE_CMD_MAX; i++) {
        if (m_cmds[i].used && (m_cmds[i].cmd == cmd)) {
            m_cmds[i].used    = false;
            m_cmds[i].handler = NULL;
            break;
        }
    }
    sm_crit_exit(pm);
}

bool SerialModule_Send(uint8_t port, uint8_t cmd,
                       const uint8_t *data, uint8_t len)
{
    if (!m_init || (port >= m_port_cnt)) { return false; }
    if ((len > 0u) && (data == NULL))    { return false; }
    if (len > SM_PAYLOAD_MAX)            { return false; }
    return sm_send_internal(port, &m_ports[port], cmd, data, len);
}

bool SerialModule_GetStats(uint8_t port, SerialModule_Stats_t *stats)
{
    if (!m_init || (port >= m_port_cnt) || (stats == NULL)) { return false; }
    sm_port_t *p = &m_ports[port];
    uint32_t pm = sm_crit_enter();
    *stats = p->stats;
    sm_crit_exit(pm);
    return true;
}

void SerialModule_Poll(void)
{
    if (!m_init) { return; }
    for (uint8_t i = 0; i < m_port_cnt; i++) {
        sm_dispatch(&m_ports[i], i);   /* 先消化接收 */
        sm_tx_kick(&m_ports[i]);       /* 再补发漏启动的帧 */
    }
}

/* ================= 中断胶水入口 ================= */

void SerialModule_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (!m_init) { return; }
    uint8_t idx = sm_find_port(huart);
    if (idx == 0xFFu) { return; }        /* 非本模块管理的串口 */

    sm_port_t *p = &m_ports[idx];
    if (size > SERIAL_MODULE_RX_DMA_BUF) { size = SERIAL_MODULE_RX_DMA_BUF; }

    for (uint16_t i = 0; i < size; i++) {
        sm_step(p, p->dma_buf[i]);
    }
    (void)sm_restart_rx(p);              /* 无论何种事件，统一重启接收 */
}

void SerialModule_OnTxCplt(UART_HandleTypeDef *huart)
{
    if (!m_init) { return; }
    uint8_t idx = sm_find_port(huart);
    if (idx == 0xFFu) { return; }

    sm_port_t *p = &m_ports[idx];
    p->stats.tx_sent++;

    bool   start_next = false;
    uint8_t nbytes    = 0u;

    uint32_t pm = sm_crit_enter();
    if (p->tx_count > 0u) {
        /* 队列还有帧：保持忙标志并自动接续 */
        uint8_t slot = p->tx_head;
        nbytes = p->tx_len[slot];
        memcpy(p->tx_stage, p->tx_q[slot], nbytes);
        p->tx_head = (uint8_t)((slot + 1u) % SERIAL_MODULE_TX_DEPTH);
        p->tx_count--;
        p->tx_busy = true;
        start_next = true;
    } else {
        p->tx_busy = false;
    }
    sm_crit_exit(pm);

    if (start_next) {
        if (HAL_UART_Transmit_DMA(p->cfg->huart, p->tx_stage, nbytes) != HAL_OK) {
            pm = sm_crit_enter();
            p->tx_busy = false;
            p->stats.tx_errors++;
            sm_crit_exit(pm);
        }
    }
}

void SerialModule_OnError(UART_HandleTypeDef *huart)
{
    if (!m_init) { return; }
    uint8_t idx = sm_find_port(huart);
    if (idx == 0xFFu) { return; }
    /* 溢出/噪声/帧错误等可能让接收停摆：强制重启接收通道 */
    (void)sm_restart_rx(&m_ports[idx]);
}

/* ================= RTOS 调度线程 ================= */

#if SERIAL_MODULE_USE_OS2
static void sm_worker(void *argument)
{
    (void)argument;
    for (;;) {
        /* 事件唤醒或周期兜底超时，都执行一次全量扫描 */
        (void)osThreadFlagsWait(SM_BIT_SWEEP, osFlagsWaitAny,
                                SERIAL_MODULE_POLL_PERIOD_MS);
        SerialModule_Poll();
    }
}
#endif
