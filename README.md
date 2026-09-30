# 2.4G 无线温湿度采集工程（STM32F411 + SX1280 + HDC1080）

两块完全相同的板子共用**同一份代码**，靠 `NODE_ROLE` 宏区分角色：

```
发射端 (NODE_ROLE_TX)                          接收端 (NODE_ROLE_RX)
  HDC1080 采集温湿度                               SX1280 连续接收
        ↓                                              ↓
  组 9 字节空中帧 ──► SX1280 发射 ═══ 2.4GHz LoRa ═══► 校验包头
        ↓                                              ↓
  本地串口输出 0x18 遥测                        组 11 字节帧 → 串口输出 0x19
                                                （含远端温湿度 + RSSI + SNR）
```

---

## 1. 硬件构成

| 部件 | 型号 | 接口 | 说明 |
|---|---|---|---|
| MCU | STM32F411CEU6 (UFQFPN48) | — | HSE 12MHz → SYSCLK 100MHz，APB1 50MHz / APB2 100MHz |
| 无线 | SX1280 | SPI1 | 2.4GHz LoRa，SF7 / BW812.5k / CR4-5 / +6dBm / 前导码 12 |
| 温湿度 | HDC1080 | I2C1 | 7 位地址 `0x40`，14 位分辨率 |
| RTOS | FreeRTOS 10.3.1 + CMSIS-RTOS2 | — | 1000Hz tick，heap_4 |

### 引脚分配

| 功能 | MCU 引脚 | 外设引脚 | 备注 |
|---|---|---|---|
| SX1280 NSS | PA4 | 19 `NSS_CTS` | 软件片选 |
| SX1280 SCK | PA5 | 18 `SCK_RTSN` | SPI Mode0，12.5MHz |
| SX1280 MISO | PA6 | **16 `MISO_TX`** | ⚠ 芯片的输出来自 pin 16 |
| SX1280 MOSI | PA7 | **17 `MOSI_RX`** | ⚠ 芯片的输入在 pin 17 |
| SX1280 BUSY | PB10 | 7 `BUSY` | 输入，无上下拉 |
| SX1280 NRESET | PB12 | 3 `NRESET` | 输出，低有效 |
| SX1280 DIO1/2/3 | PB2 / PB1 / PB0 | 8 / 9 / 10 | EXTI 上升沿 |
| HDC1080 SCL | PB8 | 6 `SCL` | 100kHz，开漏，需上拉 |
| HDC1080 SDA | PB9 | 1 `SDA` | 100kHz，开漏，需上拉 |
| 串口 TX/RX | PB6 / PB7 | — | USART1，**115200 8N1**，DMA |
| 状态指示 | PC13 | — | 1Hz 心跳 |

> ⚠ **SX1280 的 pin 16/17 极易接反**：手册按 UART 功能命名，`MISO_TX`(16) 才是芯片输出、
> `MOSI_RX`(17) 才是芯片输入。接反后表现为 MISO 恒无响应、`SX1280_Init()` 在第一条命令就失败。

> ⚠ **HDC1080 是 PWSON 封装（6 脚，底部无引脚）**，拖焊极易虚焊，导致 I2C 完全失联。

---

## 2. 构建与烧录

```powershell
# 发射端（默认）
cmake -S . -B build/Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
ninja -C build/Debug

# 接收端
cmake -S . -B build/RX -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DNODE_ROLE=NODE_ROLE_RX
ninja -C build/RX

# 烧录
STM32_Programmer_CLI -c port=SWD mode=normal -w build/RX/2.4G.elf -v -rst
```

角色宏定义在 `DOC/Inc/App_Config.h`，也可以在 CMake 里用 `-DNODE_ROLE=` 覆盖。

---

## 3. 串口通信协议 ★

### 3.1 物理层

| 参数 | 值 |
|---|---|
| 波特率 | **115200** |
| 数据位 / 校验 / 停止位 | **8 / None / 1** |
| 流控 | 无 |
| 字节序 | 多字节字段一律**小端（Little-Endian）** |

### 3.2 帧结构

```
┌──────┬──────┬──────┬───────────────┬───────┐
│ 0x5A │ CMD  │ LEN  │ DATA × LEN    │ CRC8  │
└──────┴──────┴──────┴───────────────┴───────┘
  帧头   命令   长度    净荷（LEN 字节）  校验
```

| 字段 | 字节数 | 说明 |
|---|---|---|
| 帧头 | 1 | 固定 `0x5A` |
| CMD | 1 | 命令码，见 3.4 |
| LEN | 1 | DATA 的字节数 |
| DATA | LEN | 净荷，可为 0 |
| CRC8 | 1 | 见 3.3 |

整帧长度 = **4 + LEN**。

### 3.3 CRC8

| 参数 | 值 |
|---|---|
| 多项式 | `0x07` (x⁸+x²+x+1) |
| 初值 | `0xFF` |
| 输入/输出反转 | 否 |
| 结果异或 | `0x00` |
| **校验范围** | **`CMD` + `LEN` + `DATA`**（**不含**帧头 `0x5A`，**不含** CRC 自身） |

参考实现：

```c
/* 覆盖范围：cmd、len、data[]，共 2+len 字节 */
uint8_t crc8(const uint8_t *d, uint16_t n)
{
    uint8_t crc = 0xFFu;
    for (uint16_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (uint8_t b = 0; b < 8u; b++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}
```

```python
def crc8(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc
```

### 3.4 命令码总表

| CMD | 名称 | LEN | 发送方 | 用途 |
|---|---|---|---|---|
| `0x18` | 本地遥测 | 7 | 发射端 | 发射端自己采集的温湿度 |
| `0x19` | 空中接收上报 | 11 | 接收端 | 转发远端温湿度 + 本机接收质量 |
| `0xE0` | 栈溢出告警 | N | 任意 | 净荷 = 溢出任务的 ASCII 名 |
| `0xE1` | SX1280 初始化失败 | 1 | 任意 | 净荷 = `MYSX1280_Init()` 返回码 |

### 3.5 CMD `0x18` —— 发射端本地遥测（LEN = 7）

| 偏移 | 字段 | 类型 | 单位/换算 | 说明 |
|---|---|---|---|---|
| 0 | `status` | int8 | — | **0 = 测量成功**；负值 = HDC1080 错误码（`-1` = I2C 错误） |
| 1–2 | `seq` | uint16 | — | 帧序号，每帧 +1，回绕 |
| 3–4 | `temp` | int16 | **÷100 → ℃** | 有符号，如 `2979` = 29.79 ℃ |
| 5–6 | `hum` | uint16 | **÷100 → %RH** | 如 `3417` = 34.17 %RH |

> `status ≠ 0` 时 `temp`/`hum` 会被清零，请以 `status` 判断数据是否有效。

### 3.6 CMD `0x19` —— 接收端空中上报（LEN = 11）★上位机主要用这个

| 偏移 | 字段 | 类型 | 单位/换算 | 说明 |
|---|---|---|---|---|
| 0 | `status` | int8 | — | **0 = 收包并通过包头校验**；`-8` = 长度错；`-4` = 包头错 |
| 1 | `remote_status` | int8 | — | 转发自发射端遥测的 `status` |
| 2–3 | `remote_seq` | uint16 | — | 发射端的帧序号 |
| 4–5 | `remote_temp` | int16 | **÷100 → ℃** | 发射端测得的温度 |
| 6–7 | `remote_hum` | uint16 | **÷100 → %RH** | 发射端测得的湿度 |
| 8–9 | `rssi` | int16 | **dBm** | 本机接收信号强度，负值 |
| 10 | `snr` | int8 | **dB** | 解调信噪比，通常为正 |

**真实抓包示例（三帧连续）：**

```
5A 19 0B 00 00 BF 01 A3 0B 59 0D F0 FF 0E 15
5A 19 0B 00 00 C0 01 A7 0B 61 0D F0 FF 0E 14
5A 19 0B 00 00 C1 01 A7 0B 6A 0D F0 FF 0E D2
│  │  │  │  │  │  │  │  │  │  │  │  │  │  └ CRC8
│  │  │  │  │  │  │  │  │  │  │  │  │  └── SNR = 0x0E = 14 dB
│  │  │  │  │  │  │  │  │  │  │  │  └───── RSSI 高字节 0xFF
│  │  │  │  │  │  │  │  │  │  │  └──────── RSSI 低字节 0xF0 → 0xFFF0 = -16 dBm
│  │  │  │  │  │  │  │  │  │  └─────────── 湿度低字节 0x59
│  │  │  │  │  │  │  │  │  └────────────── 湿度高字节 0x0D → 0x0D59 = 3417 → 34.17 %
│  │  │  │  │  │  │  │  └───────────────── 温度低字节 0xA3
│  │  │  │  │  │  │  └──────────────────── 温度高字节 0x0B → 0x0BA3 = 2979 → 29.79 ℃
│  │  │  │  │  │  └─────────────────────── 序号高字节 0x01
│  │  │  │  │  └────────────────────────── 序号低字节 0xBF → 0x01BF = 447
│  │  │  │  └───────────────────────────── remote_status = 0（发射端测量成功）
│  │  │  └──────────────────────────────── status = 0（本机收包校验通过）
│  │  └─────────────────────────────────── LEN = 0x0B = 11
│  └────────────────────────────────────── CMD = 0x19
└───────────────────────────────────────── 帧头
```

解析结论：**29.79 ℃ / 34.17 %RH，接收信号 −16 dBm、SNR 14 dB**。

### 3.7 CMD `0xE0` / `0xE1` —— 一次性告警

| CMD | LEN | 净荷 | 含义 |
|---|---|---|---|
| `0xE0` | N | 任务名（ASCII，无结尾 0） | 检测到 FreeRTOS 任务栈溢出 |
| `0xE1` | 1 | int8 返回码 | `MYSX1280_Init()` 失败码：`-2`=SPI `-3`=超时 `-4`=命令 `-5`=状态 |

两者都**只在发生时上报一次**。

### 3.8 Python 上位机解析示例

```python
import serial

def crc8(data: bytes) -> int:
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

def s16(lo: int, hi: int) -> int:
    v = lo | (hi << 8)
    return v - 65536 if v >= 32768 else v

def parse(buf: bytearray):
    """从缓冲区里切出所有完整帧，返回 (帧列表, 剩余缓冲)"""
    frames, i = [], 0
    while i + 4 <= len(buf):
        if buf[i] != 0x5A:
            i += 1
            continue
        cmd, ln = buf[i + 1], buf[i + 2]
        total = 4 + ln
        if i + total > len(buf):
            break                                   # 帧不完整，等更多数据
        payload = bytes(buf[i + 3: i + 3 + ln])
        crc_rx = buf[i + 3 + ln]
        if crc8(bytes([cmd, ln]) + payload) != crc_rx:
            i += 1                                  # CRC 错，跳过帧头继续找
            continue
        frames.append((cmd, payload))
        i += total
    return frames, buf[i:]

ser = serial.Serial("COM4", 115200, timeout=1)
buf = bytearray()
while True:
    buf += ser.read(256)
    frames, buf = parse(buf)
    for cmd, p in frames:
        if cmd == 0x19 and len(p) == 11:
            print(f"温度 {s16(p[4], p[5]) / 100:6.2f} ℃   "
                  f"湿度 {s16(p[6], p[7]) / 100:6.2f} %RH   "
                  f"RSSI {s16(p[8], p[9]):4d} dBm   SNR {p[10] - 256 if p[10] > 127 else p[10]:3d} dB")
        elif cmd == 0x18 and len(p) == 7:
            print(f"[本地] 温度 {s16(p[3], p[4]) / 100:6.2f} ℃   "
                  f"湿度 {s16(p[5], p[6]) / 100:6.2f} %RH")
        elif cmd == 0xE1:
            print(f"[告警] SX1280 初始化失败，返回码 {p[0] - 256 if p[0] > 127 else p[0]}")
        elif cmd == 0xE0:
            print(f"[告警] 任务栈溢出: {p.decode('ascii', 'replace')}")
```

---

## 4. 空中帧格式（发射端 → 接收端）

固定 **9 字节**，空口完整性由 SX1280 的 LoRa 硬件 CRC 保证：

| 偏移 | 字段 | 说明 |
|---|---|---|
| 0 | `0xA5` | 帧头 0 |
| 1 | `0x5A` | 帧头 1 |
| 2 | `status` | 同 `0x18` 净荷的 status |
| 3–4 | `seq` | 序号（小端） |
| 5–6 | `temp` | 温度 ×100（int16 小端） |
| 7–8 | `hum` | 湿度 ×100（uint16 小端） |

接收端校验帧头后，把 `[2..8]` 这 7 字节原样放进 `0x19` 净荷的 `[1..7]`。

---

## 5. 已修复的关键问题（供后续维护参考）

| # | 问题 | 现象 | 处理 |
|---|---|---|---|
| 1 | SX1280 `pin16/17` 原理图接反 | MISO 恒无响应 | 重新打板 |
| 2 | 初始化放在 `osThreadNew` 之后 | FreeRTOS 临界区计数无法归零 → BASEPRI 停在 `0x50` 屏蔽 TIM10 → `HAL_Delay` 死等、**整机启动死机** | `MYSX1280_Init()` 移到 `main.c`、任何任务创建之前 |
| 3 | `SX1280_StatusCheck()` 把合法的命令状态 `0` 判为错误 | 复位后第一条命令必失败 | 放行状态 `0` |
| 4 | 复位后首事务被 SPI/UART 接口判别吞掉（手册 9.7/11.3） | 状态字节读到 `0xFF`/`0x55` | 复位后补发一条 `GET_STATUS(0xC0)` 顶掉它 |
| 5 | SPI 时钟被配成 50MHz（超手册 18MHz 上限 2.8 倍） | 通信异常 | `spi.c` USER CODE 段内覆盖为 `/8` = 12.5MHz |
| 6 | SX1280 虚焊 | 内部稳压仅 0.5V → 52MHz 晶振不起振 → 进不了 TX/RX | 重新焊接 |
| 7 | HDC1080 虚焊（PWSON） | I2C 总线被钳死、地址扫描 0 应答 | 重新焊接 |

---

## 6. 已知问题与后续改进

1. **驱动用"上一条命令"的状态字节判定成败**
   SPI 从机是在事务**开始**时输出状态字节的，所以 `SX1280_StartTransmit()` 等返回的其实是
   前一条命令的结果。当前所有前置命令都成功，因此不影响使用；但严格做法是发完命令后
   再补一次 `GetIrqStatus` 回读确认。

2. **缺少 I2C 总线自恢复**
   若 MCU 在一次传输中途复位，HDC1080 可能钳住 SDA 不放，导致后续所有 I2C 操作失败
   （现象：`SR1` 的 `ARLO` 置位、`SR2.BUSY=1`、SCL/SDA 都被拉低）。
   标准恢复流程（把 SCL 当普通 IO 打 9 个时钟 + 补 STOP + 重新初始化外设）在
   调试期验证有效，建议后续整理成 `HDC1080.c` 的正式函数，在 I2C 失败时自动调用。

3. **接收端不做本地采集**
   `NODE_ROLE_RX` 不使用 HDC1080，本地业务只有 LED 心跳与一次性告警上报。
