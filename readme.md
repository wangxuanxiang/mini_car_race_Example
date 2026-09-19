# 南工绝影 22 届小车赛 —— 示例代码（mini_car_race_Example）

> 本工程为 **南工绝影 22 届小车赛示例代码**，同时也可作为后续赛题的起步模板。
>
> 主要包含：**负压风扇（无刷电调）驱动**、**BMI270 六轴陀螺仪驱动与读取示例**、**光电管阵列（多路复用）读取示例**，
> 以及 **TIM1 双路 PWM 电机控制**、**TIM3 / TIM4 编码器测速**、**USART3 串口调试** 的配置。

- 仓库地址：<https://github.com/wangxuanxiang/mini_car_race_Example.git>
- 主控：**STM32F103C8T6**（LQFP48，64 KB Flash / 20 KB SRAM）
- 工具链：STM32CubeMX 6.15.0 + **MDK-ARM V5.32**（Keil），固件包 STM32Cube FW_F1 V1.8.7
- 工程文件：`micro_smartcar.ioc` / `MDK-ARM/micro_smartcar.uvprojx`

> ⚠️ **本示例不是一辆能跑完整赛道的车**，它是各外设的“最小可用示例集合”：
> 陀螺仪与光电管是**读取示例**，电机 PWM 只给了**固定占空比的初始化值**，
> 控制环（PID、差速、循迹决策）需要你自己在此基础上补全。详见文末「已知限制与待办」。
> 若无更改，本示例程序在通电后会执行：负压风扇初始化并启动，后向两电机输出约27.78%占空比的pwm波，同时读取并发送陀螺仪的数据并输出
---

## 1. 功能模块总览

| 模块 | 外设 | 关键引脚 | 状态 |
|---|---|---|---|
| 负压风扇（无刷电调 ESC） | TIM1 更新事件 + DMA1_CH5 驱动 GPIO | PA11 | ✅ 已实现（软件 ESC 波形） |
| BMI270 六轴陀螺仪/加速度计 | SPI1 + DMA1_CH2/CH3 | PA4(CS) PA5(SCK) PA6(MISO) PA7(MOSI) | ✅ 已实现 |
| 光电管阵列（12 路） | GPIO 4 位地址 + 1 位数据 | PA12(读) PA15/PB3/PB8/PB9(地址) | ✅ 已实现 |
| 电机 PWM | TIM1 CH1 / CH2 | PA8(L_PWM) PA9(R_PWM) | ⚠️ 已配置，无控制逻辑 |
| 编码器测速 | TIM3 / TIM4 编码器模式 | PB4/PB5(右) PB6/PB7(左) | ⚠️ 已配置，无读取逻辑 |
| 串口调试 | USART3 | PB10(TX) PB11(RX) | ✅ printf 输出可用 |
| 周期任务定时器 | TIM2（1 kHz 中断） | — | ⚠️ 已配置，中断回调为空 |

---

## 2. 时钟与系统配置

| 项目 | 值 |
|---|---|
| 时钟源 | 外部晶振 HSE + PLL ×9 |
| SYSCLK / HCLK | **72 MHz** |
| APB1 / APB1 定时器时钟 | 36 MHz / **72 MHz** |
| APB2 / APB2 定时器时钟 | **72 MHz** / **72 MHz** |
| Flash 等待周期 | 2 |
| HAL 时基 | SysTick，优先级 15 |
| NVIC 优先级分组 | `NVIC_PRIORITYGROUP_4`（4 位抢占，0 位子优先级） |

关键点：**APB1 分频为 2，但定时器时钟会被自动倍频回 72 MHz**，所以 TIM2/TIM3/TIM4 的输入时钟都是 72 MHz。

---

## 3. 负压风扇（无刷电调 ESC）

### 3.1 电调控制原理

负压风扇由**航模无刷电调（ESC）**驱动，电调只认标准航模 PWM 信号：

- **频率 50 Hz**（周期 20 ms）
- **脉宽 1000 µs = 最低油门/停车，2000 µs = 最高油门**
- 上电后必须先给一段**解锁（arming）信号**，电调才会响应

### 3.2 为什么用 GPIO + DMA 而不是硬件 PWM

TIM1 的 CH1/CH2 已经被左右电机 PWM 占用，PA11 上**没有可用的定时器通道**（不走重映射就无法让 TIM1_CH4 出现在 PA11），
因此这里采用「**定时器更新事件 + DMA 写 BSRR**」的方式软件合成 50 Hz 波形：

| 参数 | 值 | 说明 |
|---|---|---|
| `TIM1.Prescaler` | 0 | 计数频率 72 MHz |
| `TIM1.Period` | 7199 | 7200 计数 = **10 kHz** 更新事件 |
| `ESC_PULSE_SLOTS` | 200 | 一帧 = 200 个时隙 × 100 µs = **20 ms** |
| `ESC_START_PULSE_SLOTS` | 9 | 9 × 100 µs = **0.9 ms**（解锁脉宽） |
| `ESC_RUN_PULSE_SLOTS` | 20 | 20 × 100 µs = **2.0 ms**（满油门） |
| `ESC_START_PERIODS` | 100 | 100 帧 × 20 ms = **2 s**（解锁时长） |
| DMA | DMA1_Channel5，`DMA_CIRCULAR`，32 位字 | 每次 TIM1 更新事件搬运 1 个字到 `GPIOA->BSRR` |

`esc_dma_table[i]` 的含义：第 i 个时隙里 PA11 该输出什么电平 —— 高电平写 `GPIO_PIN_11`（置位 BSRR 低 16 位），
低电平写 `GPIO_PIN_11 << 16`（置位 BSRR 高 16 位即复位）。

### 3.3 启动流程（`Init_brushless_motor()`）

1. `PA11` 先拉低
2. `ESC_BuildDmaTable(9)` 建表 → 0.9 ms 高电平 / 20 ms 帧
3. 启动 DMA（循环模式）并使能 `TIM_DMA_UPDATE`，DMA 每个更新事件搬 1 个字，200 个字刚好 20 ms 一轮
4. 每帧完成进一次 `ESC_DmaTransferComplete()` 中断，累计到 100 帧（**2 秒**）后：
   - `IF_start_Brushless_motor == 1` → `ESC_SetPulseWidth(20)`，输出 **2.0 ms 满油门**
   - `IF_start_Brushless_motor == 0` → `ESC_SetPulseWidth(10)`，输出 **1.0 ms 停车**
   - 关掉 DMA 传输完成中断，此后波形由 DMA 硬件持续输出，不再占用中断
5. 主函数用 `while (esc_period_count < ESC_START_PERIODS) HAL_Delay(200);` 阻塞等待解锁完成

> **注意**：`ESC_BuildDmaTable` 填的表与 DMA 是**按索引顺序**消费的，
> 而 `ESC_SetPulseWidth` 只改动真正变化的那一段且按索引递增顺序改，
> 因此正在输出的那一帧不会被破坏（源码注释里说明了这一点）。

### 3.4 使用方法

`main.c` 中的宏控制风扇行为：

```c
#define IF_start_Brushless_motor 1   // 1 = 解锁后转到满油门；0 = 解锁后停车
```

- **置 1**：负压启动（2 秒解锁后进入 2.0 ms 满油门）
- **置 0**：负压关闭（2 秒解锁后输出 1.0 ms 停车信号）

> ⚠️ 上电即满油门有安全风险（桨叶/电机突然启动）。调试时建议先置 0，确认接线与方向无误后再改 1。

> ⚠️ 负压风扇震动会导致IMU数据比无负压波动大（大概9-10倍），上负压的组需要做好准备，可能需要自行通过滤波等方法解决IMU零漂等问题

> ⚠️ 若负压风扇启动异常（常为发出滴滴滴的声音并不断起停），则为初始化错误查看是否按照流程初始化，详见[readme.md?plain=1#L77]
---

## 4. BMI270 六轴陀螺仪 / 加速度计

### 4.1 硬件连接

| 信号 | 引脚 | 说明 |
|---|---|---|
| SCK | PA5 | SPI1_SCK |
| MISO | PA6 | SPI1_MISO |
| MOSI | PA7 | SPI1_MOSI |
| CS | **PA4** | 软件片选，普通推挽输出，高速 |
| INT1/INT2 | — | **未使用**（本示例为轮询读取） |

### 4.2 SPI1 配置

| 参数 | 值 |
|---|---|
| 模式 | 主机，全双工，2 线 |
| 数据宽度 | 8 bit |
| CPOL / CPHA | Low / 1Edge（**SPI Mode 0**） |
| NSS | 软件 |
| 位序 | MSB First |
| 分频 | `SPI_BAUDRATEPRESCALER_8` → **9 Mbit/s** |
| DMA | TX = DMA1_Channel3，RX = DMA1_Channel2，均为 `DMA_NORMAL`、字节对齐 |

### 4.3 驱动分层

```
bmi270.c / bmi2.c / bmi2_defs.h     Bosch 官方驱动（不要改）
        ↓
bmi270_port.c / bmi270_port.h       移植层：SPI 读写、CS 控制、微秒延时、传感器配置
        ↓
dodo_BMI270.c / dodo_BMI270.h       用户层：初始化 + 取数 + 物理量换算（作者 pupydodo）
        ↓
main.c                              应用示例
```

**移植层要点**（`bmi270_port.c`）：

- `bmi2_spi_read()`：寄存器地址**或上 `0x80`** 表示读；DMA 收发 `len+1` 字节（首字节为地址），
  跳过第 0 字节后拷贝到 `reg_data`
- `bmi2_spi_write()`：地址**与上 `0x7F`** 表示写；用 `HAL_SPI_TransmitReceive_DMA` / `HAL_SPI_Transmit_DMA`
- 传输完成靠 `HAL_SPI_TxRxCpltCallback` / `HAL_SPI_TxCpltCallback` 置标志位，`while` 空转等待
- **SPI 上电后需要一次 dummy read** 才能进入 SPI 模式（BMI270 数据手册要求），
  代码里先读 `BMI2_CHIP_ID_ADDR (0x00)` 再延时 1 ms，然后才 `bmi270_init()`
- 微秒延时用 **DWT->CYCCNT** 实现（`dwt_init()` 使能 DWT 周期计数器），不占用定时器

**传感器配置**（`bmi270_set_config()`，可按需修改）：

| 传感器 | 参数 | 设置值 |
|---|---|---|
| 加速度计 | ODR | **1600 Hz** |
| | 量程 | **±8 g** |
| | 带宽/滤波 | `BMI2_ACC_NORMAL_AVG4`，`BMI2_PERF_OPT_MODE` |
| 陀螺仪 | ODR | **1600 Hz** |
| | 量程 | **±2000 dps** |
| | 滤波 | `BMI2_GYR_NORMAL_MODE`，`BMI2_PERF_OPT_MODE`，`BMI2_GYR_NOISE_PERF_MODE_MASK` |

### 4.4 使用方法

```c
#include "dodo_BMI270.h"

dodo_BMI270_init();                    // 返回 0 表示成功（内部有 init_flag，重复调用直接返回 0）

while (1) {
    dodo_BMI270_get_data();            // 读一次，更新 6 个全局原始值
    float gyro_x = BMI270_gyro_transition(BMI270_gyro_x);   // 原始值 → °/s
    float accel_x = BMI270_acc_transition(BMI270_accel_x);  // 原始值 → g
}
```

原始值全局变量：`BMI270_gyro_x/y/z`、`BMI270_accel_x/y/z`（`int16_t`）。
换算系数：`BMI270_transition_factor[2] = {4096, 16.4}`，即
**加速度 ÷4096 → g**（对应 ±8 g 量程，4096 LSB/g），**角速度 ÷16.4 → °/s**（对应 ±2000 dps，16.4 LSB/dps）。

> ⚠️ 若修改了量程，**必须同步修改这两个系数**，否则物理值会成比例错掉。


## 5. 光电管阵列（12 路，多路复用）

### 5.1 原理

12 路光电管信号经过一片 **16 选 1 多路复用器** 汇总到**一根** GPIO 上，
用 4 根地址线选择当前读取哪一路（只用其中 12 个通道，编号 **0~11，从左到右**）。

| 信号 | 引脚 | 方向 |
|---|---|---|
| 读数据 `MUX_READ` | **PA12** | 输入 |
| 地址位 0 `MUX_0` | **PA15** | 输出 |
| 地址位 1 `MUX_1` | **PB3** | 输出 |
| 地址位 2 `MUX_2` | **PB8** | 输出 |
| 地址位 3 `MUX_3` | **PB9** | 输出 |

### 5.2 接口

```c
#include "multiplexer.h"

uint16_t mux_value;
MUX_get_value(&mux_value);                       // 一次读完全部 12 路，打包进 1 个 uint16

for (int i = 0; i <= 11; i++) {
    printf("%d,", MUX_GET_CHANNEL(mux_value, i)); // 取出第 i 路的 0/1
}
```

- `MUX_get_value()`：从通道 0 扫到 11，每路设置地址后延时 **1 µs** 等待模拟开关稳定，再读 GPIO
- **位序**：第 i 路存放在 `bit (MULTIPLEXER_CHANNEL_NUM - 1 - i)`，即通道 0 在**最高位**（bit 11），通道 11 在 bit 0
- `MUX_GET_CHANNEL(value, ch)` 宏负责取出对应位，返回 0 或 1
- 通道数由 `multiplexer.h` 的 `MULTIPLEXER_CHANNEL_NUM` 定义（默认 **12**）

> 读取精度依赖那 1 µs 延时，该延时由 DWT 实现（见 `multiplexer_delay_us()`），
> 与 BMI270 移植层共用同一套 DWT 机制。

---

## 6. 电机控制与编码器（TIM）

### 6.1 TIM1 —— 双路 PWM 驱动两个电机

| 参数 | 值 |
|---|---|
| 通道 | **CH1 = PA8（L_PWM，左电机）**、**CH2 = PA9（R_PWM，右电机）** |
| `Prescaler` / `Period` | 0 / 7199 |
| PWM 频率 | 72 MHz ÷ 7200 = **10 kHz** |
| 模式 | `TIM_OCMODE_PWM1`，高电平有效 |
| 初始 CCR | 0，`HAL_TIM_PWM_Start` 之后由主函数写值 |
| 死区 / 刹车 | 未启用（`DeadTime = 0`，`BreakState = TIM_BREAK_DISABLE`） |

启动与调速：

```c
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // PA8 左电机
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   // PA9 右电机

TIM1->CCR1 = 2000;    // 左电机占空比 = 2000 / 7200 ≈ 27.8 %
TIM1->CCR2 = 2000;    // 右电机占空比 = 2000 / 7200 ≈ 27.8 %
```

> **占空比 = CCR / 7200**，CCR 有效范围 0 ~ 7199。
> 示例里写死的 2000 只是“能转起来”的演示值，**不是标定值**。

### 6.2 方向控制引脚

| 信号 | 引脚 | 说明 |
|---|---|---|
| `L_DIR` | **PB15** | 左电机方向，推挽输出 |
| `R_DIR` | **PA10** | 右电机方向，推挽输出 |

方向与 PWM 组合成常见的 **PWM + DIR** 驱动方式（配合 H 桥或电机驱动板）。
当前代码只把两脚初始化为低电平，**没有方向切换逻辑**。

### 6.3 TIM3 / TIM4 —— 正交编码器测速

| 定时器 | 通道 | 引脚 | 对应 |
|---|---|---|---|
| **TIM3** | CH1 / CH2 | **PB4（R_ENCODER_B）/ PB5（R_ENCODER_A）** | 右轮 |
| **TIM4** | CH1 / CH2 | **PB6（L_ENCODER_B）/ PB7（L_ENCODER_A）** | 左轮 |

- 模式：`TIM_ENCODERMODE_TI12`（**双通道正交，4 倍频**）
- `Prescaler = 0`、`Period = 65535` → **16 位计数器**，溢出后回绕
- 输入滤波 `IC1Filter = 0`，不分频，上升沿
- **TIM3 需要部分重映射**：`__HAL_AFIO_REMAP_TIM3_PARTIAL()`（否则 CH1/CH2 落在 PA6/PA7）

读数方式（示例未实现，供你补全）：

```c
int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);   // 右轮计数增量
__HAL_TIM_SET_COUNTER(&htim3, 0);                        // 清零，下一周期再读
```

> ⚠️ `Period` 是 65535，而 `__HAL_TIM_GET_COUNTER` 返回 `uint32_t`，
> **做差值时类型转换要正确**，否则反向转动时正负号会出错。

### 6.4 TIM2 —— 周期任务时基

| 参数 | 值 |
|---|---|
| `Prescaler` / `Period` | 71 / 999 |
| 中断频率 | 72 MHz ÷ 72 ÷ 1000 = **1 kHz（1 ms）** |
| 优先级 | 抢占 1 |
| 用途 | 预留的固定周期控制任务时基 |

**目前 `HAL_TIM_PeriodElapsedCallback()` 没有实现**：TIM2 中断虽然在 `stm32f1xx_hal_msp.c`
里使能了 NVIC，但 `HAL_TIM_IRQHandler(&htim2)` 走到空回调后什么都不做——控制环需要在这里补。

---

## 7. 串口（USART3）

| 参数 | 值 |
|---|---|
| 外设 | **USART3** |
| TX / RX 引脚 | **PB10 / PB11** |
| 波特率 | **115200** |
| 数据位 / 停止位 / 校验 | **8 / 1 / 无** |
| 模式 | 收发（`UART_MODE_TX_RX`） |
| 流控 / 过采样 | 无 / 16 倍 |
| 中断 | `USART3_IRQn` 已使能（`HAL_UART_IRQHandler(&huart3)`） |

**printf 重定向**（`main.c`）：

```c
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
```

因此 `printf()` 会**逐字节阻塞发送**到 USART3。示例输出为 CSV 格式：

```
gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z\r\n
```

> ⚠️ 两个注意点：
> 1. 逐字节 `HAL_UART_Transmit` 在 115200 下每字节约 87 µs，**会明显阻塞主循环**；要提速可改 DMA 发送或用环形缓冲。
> 2. **接收方向（RX）目前没有任何使用示例**——中断服务函数调用了 `HAL_UART_IRQHandler`，
>    但没有启用 `HAL_UART_Receive_IT`，也没有实现 `HAL_UART_RxCpltCallback`。若要上位机调参，需自行补接收解析。

---

## 8. 目录结构

```
mini_car_race_Example/
├── micro_smartcar.ioc              # STM32CubeMX 工程（引脚/外设配置的唯一权威来源）
├── readme.md                       # 本文件
├── Core/
│   ├── Inc/
│   │   ├── main.h                  # 全部 GPIO 引脚宏定义（L_PWM / R_PWM / MUX_x / *_ENCODER_x ...）
│   │   ├── stm32f1xx_hal_conf.h    # HAL 模块裁剪配置
│   │   └── stm32f1xx_it.h
│   ├── Src/
│   │   ├── main.c                  # ★ 应用主体：外设初始化、负压风扇 ESC、陀螺仪示例
│   │   ├── stm32f1xx_hal_msp.c     # ★ 外设底层初始化：GPIO 复用、DMA 通道、NVIC
│   │   ├── stm32f1xx_it.c          # 中断服务函数
│   │   ├── system_stm32f1xx.c      # 系统时钟
│   │   └── syscalls.c / sysmem.c   # newlib 桩函数
│   ├── code/                       # ★ 外设驱动（第三方 + 自研）
│   │   ├── bmi270.c / bmi270.h         # Bosch 官方 BMI270 驱动
│   │   ├── bmi2.c / bmi2.h             # Bosch 官方通用 BMI2 层
│   │   ├── bmi2_defs.h                 # 寄存器/枚举/宏定义
│   │   ├── bmi270_port.c / .h          # 【移植层】SPI 读写、CS、DWT 延时、传感器配置
│   │   ├── dodo_BMI270.c / .h          # 【用户层】init / get_data / 物理量换算宏
│   │   └── multiplexer.c / .h          # 【光电管】多路复用器读取
│   └── Startup/
│       └── startup_stm32f103c8tx.s
├── Drivers/                        # STM32F1xx HAL 库 + CMSIS（勿改）
├── MDK-ARM/                        # Keil 工程、分散加载、调试配置
└── STM32F103C8TX_FLASH.ld          # GCC 链接脚本（供非 Keil 工具链使用）
```

---

## 9. 编译与烧录

### Keil MDK（工程默认）

1. 用 Keil uVision5 打开 `MDK-ARM/micro_smartcar.uvprojx`
2. 确认器件为 `STM32F103C8`，下载器选 **ST-Link / DAP-Link（SWD）**
3. `Build`（F7）→ `Download`（F8）
4. 串口助手连 USART3（**115200 8N1**）查看输出

> `MDK-ARM/清除缓存文件.bat` 可用于清理编译产物。

### 内存占用提醒

- **MDK-Lite 有 32 KB 链接上限**，本工程已接近该上限：
- **MDK-Plus 有 64 KB 链接上限**，本工程未接近上限
若出现内存报错请检查keil是否已破解（详见教程）

### 若用 CubeMX 重新生成代码

- ⚠️ **`Init_brushless_motor()` 及其相关的 ESC 代码块在 `main.c` 中的位置比较特殊**
  （定义在 `USER CODE BEGIN PV` / 首个函数区，`MX_*_Init` 原型之后），
  重新生成前建议先备份 `main.c`，生成后仔细合并

---

## 10. 已知限制与待办（TODO）

按优先级排列，这些是“示例”到“能跑的车”之间要补的东西：

| # | 项目 | 现状 | 待办 |
|---|---|---|---|
| 1 | **控制环** | 主循环只打印陀螺仪数据 | 实现 `HAL_TIM_PeriodElapsedCallback()`，在 TIM2 的 1 kHz 时基里做采样 / PID / 输出 |
| 2 | **编码器测速** | TIM3/TIM4 已初始化，未读取 | 周期读计数器 → 换算线速度 → 速度环 |
| 3 | **电机输出** | `TIM1->CCR1/CCR2` 固定 2000 | 接 PID 输出，加限幅与死区补偿；补 `L_DIR` / `R_DIR` 方向逻辑 |
| 4 | **串口接收** | 只发不收 | 启用 `HAL_UART_Receive_IT` / DMA + 空闲中断，做上位机调参与指令下发 |
| 5 | **printf 阻塞** | 逐字节阻塞发送 | 改 DMA 发送或环形缓冲 |
| 6 | **中断优先级** | `TIM2_IRQn`、`DMA1_Channel5_IRQn` 抢占优先级均为 1 | 明确控制中断时序，避免与 SPI-DMA 等待互相拖慢 |

---

## 11. 参考资料

- BMI270 数据手册与 Bosch Sensortec [BMI270 官方驱动](https://github.com/boschsensortec/BMI270_SensorAPI)（`bmi270.c` / `bmi2.c` 来源）
- `dodo_BMI270.c` 用户层封装作者：[pupydodo](https://github.com/pupydodo)
- STM32F103C8T6 参考手册 / 数据手册（TIM 编码器模式、DMA 请求映射、AFIO 重映射）
- STM32CubeMX / STM32Cube FW_F1 V1.8.7
