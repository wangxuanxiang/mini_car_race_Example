/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 * 本文件内含BMI270陀螺仪的使用示例
 * 本文件内含无刷电桥的启动初始化的使用示例
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "stdlib.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dodo_BMI270.h" //陀螺仪驱动
#include "multiplexer.h" //多路复用器驱动，用于读取光电管读数
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float i_acc;      /* 积分器 */
  float last_err;   /* 上一次误差 */
  float out;        /* 上一次输出 用于调试 */
} PID_t; // 状态 PID

typedef struct {
  const char *name;
  float *p;
  float min, max;
} Param_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 陀螺仪调试打印开关。置 1 会引入约 5.4 kB 的浮点 printf 代码，
   MDK-Lite 的 32 kB 链接上限会因此报 L6047U 而链接失败。
   平时保持 0；要串口看数据时改成 1（并注意镜像大小）。 */
#define GYRO_DEBUG 0
#define IF_start_Brushless_motor 1 //是否启动无刷电机，1为启动，0为不启动

/* 控制周期(ms) */
#define SPD_PERIOD      5U
#define LINE_PERIOD     5U
#define TEL_PERIOD      20U     // 预留：实际帧率由命令 STR 控制的 tel_hz 决定
#define SAFE_PERIOD     10U     // 预留：M8 失效检查周期

/* 编码器标定量(占位待测) */
#define ENC_COUNTS_PER_M_L   1000
#define ENC_COUNTS_PER_M_R   1000
#define ENC_SIGN_L           (+1)
#define ENC_SIGN_R           (-1)     // 待测

/* 电机标定量(占位待测) */
#define MOTOR_ARR            7200
#define MOTOR_DUTY_MAX       6480     // 0.9 × 7200
#define MOTOR_DEAD_L         600      // 待测：起转 PWM 占空比，600 / 7200 = 8.3%
#define MOTOR_DEAD_R         600      // 待测：同上

/* DIR 极性(待测，可能由于镜像安装出现两轮反转) */
#define MOTOR_DIR_INVERT_L   0
#define MOTOR_DIR_INVERT_R   0

/* 协议常量 */
#define LINE_ACTIVE_LEVEL    1        // 传感器极性待测
#define CMD_LINE_MAX         64U
#define TEL_BUF_SIZE         128U

/* 编码器-定时器映射：TIM3=左轮、TIM4=右轮 */
#define ENC_TIM_L            (&htim3)
#define ENC_TIM_R            (&htim4)

/* 三级串级常量 */
#define SPD_WIN              5U       // 速度反馈滑动窗口 = 5 ms，每 1 ms 刷新一次
#define LINE_LOST_DEV        800      // 丢线时强制偏转：800/1000 = 80% 满偏
#define LINE_LOST_MS         300U     // 连续丢线时间过长失效保护
#define GYRO_LPF_ALPHA       0.27f    // 一阶低通滤波 α：调低使波形平滑、调高使响应灵敏
#define YAW_STILL_GZ         1.0f     // Z轴角速度阈值：小于 ±1.0 °/s 认为静止
#define STEER_OUT_LIMIT      600.0f   // 转向环输出限幅：目标角速度 ±600 °/s
#define GYRO_OUT_LIMIT       400.0f   // 角速度环输出限幅：差速 ±400 mm/s
#define SPD_I_LIMIT          2000.0f  // 速度环积分限幅

/* 模式 */
#define MODE_STOP   0U
#define MODE_OPEN   1U
#define MODE_RUN    2U
#define MODE_STRAIGHT  3U
#define MODE_TURN      4U
#define MODE_ROUND     5U

/* 陀螺仪-待测：俯视逆时针转动小车， gz 为负时改成 1 */
#define GYRO_INVERT          0

/* 定距 / 旋转 / 环岛（待赛道实测微调） */
#define DIST_DECEL_MM     200     // 距终点 200 mm 开始按斜率减速
#define DIST_TOL_MM        10     // 到点判定
#define TURN_TOL_D10       15     // 旋转到位判定 1.5°
#define TURN_TIMEOUT_MS   3000U   // 旋转超时：陀螺漂移/打滑/堵转时兜底
#define RING_TIMEOUT_MS   6000U   // 环岛超时：出岛判定失效时兜底
#define RING_SPEED_PCT     60     // 环岛内基础速度 = vset × 60%
#define RING_TURN_DPS     250.0f  // 环岛内固定目标角速度 °/s
#define RING_SWEEP_D10   3300     // 累计航向变化 330° 判定出岛

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_tim1_up;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//负压启动相关宏
//负压启动流程：900us脉宽50Hzpwm信号输出2秒后，改为持续2000us脉宽50Hzpwm信号输出
#define ESC_PULSE_SLOTS       200U       /* 20 ms at the 10 kHz TIM1 update rate */
#define ESC_START_PULSE_SLOTS 9U         /* 0.9 ms */
#define ESC_RUN_PULSE_SLOTS   20U        /* 2.0 ms */
#define ESC_START_PERIODS     100U       /* 100 x 20 ms = 2 s */

static uint32_t esc_dma_table[ESC_PULSE_SLOTS];
static uint32_t esc_high_slots;    /* high time currently stored in the table */
static volatile uint32_t esc_period_count;

/* Fill a whole 20 ms frame.  Only used once, before the DMA is started. */
static void ESC_BuildDmaTable(uint32_t high_slots)
{
  uint32_t i;

  for (i = 0U; i < ESC_PULSE_SLOTS; ++i)
  {
    esc_dma_table[i] = (i < high_slots) ? GPIO_PIN_11 : ((uint32_t)GPIO_PIN_11 << 16U);
  }
  esc_high_slots = high_slots;
}

/* Rewrite only the slots whose level really changes, in increasing index
   order: the circular DMA consumes the table in that same order, and slot 9
   is not transferred before ~0.9 ms after this interrupt, so the frame that
   is already running is never disturbed. */
static void ESC_SetPulseWidth(uint32_t high_slots)
{
  uint32_t first;
  uint32_t last;
  uint32_t i;

  if (high_slots == esc_high_slots)
  {
    return;
  }

  first = (high_slots < esc_high_slots) ? high_slots : esc_high_slots;
  last  = (high_slots > esc_high_slots) ? high_slots : esc_high_slots;

  for (i = first; i < last; ++i)
  {
    esc_dma_table[i] = (i < high_slots) ? GPIO_PIN_11 : ((uint32_t)GPIO_PIN_11 << 16U);
  }
  esc_high_slots = high_slots;
}
static void ESC_fullPulseWidth(void){
  uint32_t i;
  for (i = 0U; i < ESC_PULSE_SLOTS; ++i)
  {
    esc_dma_table[i] = GPIO_PIN_11;
  }
  esc_high_slots = ESC_PULSE_SLOTS;
}
/* One interrupt per 20 ms frame: DMA transfer complete in circular mode.
   The 50 Hz waveform itself is produced by TIM1 + DMA in hardware; this
   handler only counts frames and switches 0.9 ms -> 2.0 ms after 2 s. */
static void ESC_DmaTransferComplete(DMA_HandleTypeDef *hdma)
{
  if (esc_period_count < ESC_START_PERIODS)
  {
    ++esc_period_count;
    if (esc_period_count == ESC_START_PERIODS)
    {
      /* The frame that has just been completed was the last 0.9 ms frame:
         100 x 20 ms = 2 s have elapsed, 2.0 ms is now held forever. */
      if (!IF_start_Brushless_motor)
        ESC_SetPulseWidth(10U);
      else
        ESC_SetPulseWidth(ESC_RUN_PULSE_SLOTS);
      /* Nothing left to time, so stop the 50 Hz interrupt. */
      __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC);
    }
  }
}

/* ---- volatile 块：ISR 中断与主循环通讯 ---- */
volatile uint32_t tick_1ms;
volatile int32_t  v_L, v_R;              /* 轮速 mm/s */
volatile int32_t  pwm_L, pwm_R;          /* 实际 pwm 输出 */
volatile int32_t  v_tgt_L, v_tgt_R;      /* 速度环目标 mm/s */
volatile int32_t  diff_mm_s;             /* 角速度环输出：差速 mm/s */
volatile uint8_t  mode;                  /* MODE_STOP / MODE_OPEN / MODE_RUN */
volatile int32_t  cmd_open_l, cmd_open_r;
volatile int32_t  base_mm_s;             /* 速度环基础速度 */
volatile int32_t  odo_mm;                /* 里程 mm */

/* ---- 参数 其中顺序有内存映射 在 SET 与上位机有对应 ---- */
static float kp_l = 0.30f, kd_l = 0.05f;   /* 转向环 PD */
static float kp_g = 1.50f, kd_g = 0.00f;   /* 角速度环 PD */
static float kp_v = 2.00f, ki_v = 0.02f;   /* 速度环 PI */
static float ff_v = 0.00f;                 /* 前馈-摩擦力待测 */
static float vset = 120.0f;                /* 目标速度 mm/s */
static float kp_y = 0.20f, kd_y = 2.00f;   /* 航向环 PD */
static uint16_t tel_hz = 50U;              /* 遥测帧率 Hz */

/* ---- 仅 ISR 访问 ---- */
static uint16_t enc_last_L, enc_last_R;
static int16_t  enc_win_L[SPD_WIN], enc_win_R[SPD_WIN];
static uint8_t  enc_idx;
static int32_t  enc_sum_L, enc_sum_R;
static int32_t  odo_cnt_L, odo_cnt_R;

/* ---- 仅主循环访问 ---- */
static uint16_t line_mux, line_events;
static int16_t  line_err;
static uint32_t line_lost_since;
static uint8_t  line_lost_active;
static int16_t  tgt_gyro_dps;             /* 转向环输出 */
static float    gz_lpf;                   /* 滤波后的角速度测量值 */
static int16_t  gz_dps;                   /* 取整后用于遥测的测量值 */
static int32_t  yaw_d10;
static float    gz_bias;
static uint8_t  gyro_ready;

/* ---- 模式目标 ---- */
static int32_t  tgt_dist_mm;             /* 定距目标 */
static int32_t  tgt_yaw_d10;             /* 旋转目标（0.1°） */
static int8_t   ring_dir;                /* 环岛方向：+1 右环岛 / -1 左环岛 */
static int32_t  ring_yaw0;               /* 入岛时的航向基准 */
static uint32_t safe_last;
static uint32_t mode_since;              /* 进入当前模式的时刻(ms) */
static uint8_t  mode_prev;               /* 上一拍的 mode，用于检测切换 */

/* ---- PID 实例 ---- */
static PID_t pid_steer, pid_gyro, pid_spd_L, pid_spd_R, pid_yaw;

/* ---- 串口 ---- */
static uint8_t  rx_byte;
static uint8_t  rx_buf[CMD_LINE_MAX];
static volatile uint8_t rx_head, rx_tail;
static char     cmd_line[CMD_LINE_MAX];
static uint8_t  cmd_len;
static char     tel_buf[TEL_BUF_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

static void  PID_Reset(PID_t *p);
static float PID_Update(PID_t *p, float err, float kp, float ki, float kd, float i_limit, float out_limit);

static void  ENC_ReadDelta(int16_t *dl, int16_t *dr);
static void  ODO_Push(int16_t dl, int16_t dr);

static void  MOTOR_SetDuty(int32_t l, int32_t r);
static void  MOTOR_Stop(void);

static void  LINE_Read(void);
static int16_t LINE_Error(uint16_t mux, uint16_t *events);

static void  IMU_Init(void);
static void  IMU_Update(void);

static void  STEER_Update(void);
static void  GYRO_Update(void);
static void  SPD_Update(void);

static void  CTRL_Init(void);
static void  CTRL_Reset(void);

static void  CMD_Reply(const char *s, uint16_t n);
static void  UART_RxStart(void);
static void  CMD_Poll(void);
static void  CMD_Dispatch(char *line);
static void  TEL_Emit(void);

static void  CTRL_ModeStep(void);
static void  SYS_StartSequence(void);
static void  SYS_Failsafe(void);
static uint8_t RUN_KeyPressed(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
float gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z; // 陀螺仪数据

/* ==================== 参数表 ==================== */
static Param_t param_tab[] = {
  { "kp_l", &kp_l,   0.0f,  500.0f },
  { "kd_l", &kd_l, -500.0f, 500.0f },
  { "kp_g", &kp_g,   0.0f,  100.0f },
  { "kd_g", &kd_g,   0.0f,   10.0f },
  { "kp_v", &kp_v,   0.0f,  200.0f },
  { "ki_v", &ki_v,   0.0f,   50.0f },
  { "ff_v", &ff_v,   0.0f,   50.0f },
  { "vset", &vset,   0.0f, 2000.0f },
  { "kp_y", &kp_y,   0.0f,   20.0f },
  { "kd_y", &kd_y, -100.0f, 100.0f },
};
#define PARAM_NUM (sizeof(param_tab) / sizeof(param_tab[0]))

/* ==================== PID ==================== */
static void PID_Reset(PID_t *p)
{
  p->i_acc = 0.0f;
  p->last_err = 0.0f;
  p->out = 0.0f;
}

/* 位置式 PID */
static float PID_Update(PID_t *p, float err, float kp, float ki, float kd, float i_limit, float out_limit)
{
  float out;

  p->i_acc += err;
  if (p->i_acc > i_limit)  { p->i_acc = i_limit; }
  if (p->i_acc < -i_limit) { p->i_acc = -i_limit; }

  out = kp * err + ki * p->i_acc + kd * (err - p->last_err);
  p->last_err = err;

  if (out > out_limit)  { out = out_limit; }
  if (out < -out_limit) { out = -out_limit; }
  p->out = out;
  return out;
}

/* ==================== 编码器-读取并计算里程/方向 ==================== */
static void ENC_ReadDelta(int16_t *dl, int16_t *dr)
{
  uint16_t now_l = (uint16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_L);
  uint16_t now_r = (uint16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_R);

  // 不清零回绕，利用无符号整型的补码机制计算差值，强转为有符号整型后由于最高位为0/1，可以得到正确正/负值；乘以方向系数用于矫正极性；
  *dl = (int16_t)(uint16_t)(now_l - enc_last_L) * ENC_SIGN_L;
  *dr = (int16_t)(uint16_t)(now_r - enc_last_R) * ENC_SIGN_R;

  enc_last_L = now_l;
  enc_last_R = now_r;
}

/* 滑动窗口算法 */
static void ODO_Push(int16_t dl, int16_t dr)
{
  enc_sum_L += (int32_t)dl - (int32_t)enc_win_L[enc_idx];
  enc_sum_R += (int32_t)dr - (int32_t)enc_win_R[enc_idx];
  enc_win_L[enc_idx] = dl;
  enc_win_R[enc_idx] = dr;
  enc_idx = (uint8_t)((enc_idx + 1U) % SPD_WIN);
  // 循环数组实现 O(1) 滑动窗口

  odo_cnt_L += dl;
  odo_cnt_R += dr;
  odo_mm = ((odo_cnt_L + odo_cnt_R) * 500) / (ENC_COUNTS_PER_M_L + ENC_COUNTS_PER_M_R);
  // 左右轮脉冲数取平均计算小车里程
  // *500 实际是 *1000 / 2

  v_L = (enc_sum_L * 1000) / ((int32_t)ENC_COUNTS_PER_M_L * (int32_t)SPD_WIN);
  v_R = (enc_sum_R * 1000) / ((int32_t)ENC_COUNTS_PER_M_R * (int32_t)SPD_WIN);
  // 速度计算
}

/* ==================== 电机 ==================== */
static int32_t MOTOR_Shape(int32_t u, int32_t dead) // 死区补偿与幅值限制
{
  int32_t mag;

  if (u == 0) { return 0; }
  mag = (u > 0) ? u : -u;
  if (mag > MOTOR_DUTY_MAX) { mag = MOTOR_DUTY_MAX; }
  // [0, MAX] 线性映射到 [dead, MAX] 实现死区线性补偿
  mag = dead + (mag * (MOTOR_DUTY_MAX - dead)) / MOTOR_DUTY_MAX;
  return (u > 0) ? mag : -mag;
}

static void MOTOR_SetDuty(int32_t l, int32_t r) // 设置 PWM 占空比
{
  int32_t sl = MOTOR_Shape(l, MOTOR_DEAD_L);
  int32_t sr = MOTOR_Shape(r, MOTOR_DEAD_R);
  uint32_t bl, br;

  bl = (sl >= 0) ? (uint32_t)sl : (uint32_t)(-sl);
  br = (sr >= 0) ? (uint32_t)sr : (uint32_t)(-sr);

  HAL_GPIO_WritePin(L_DIR_GPIO_Port, L_DIR_Pin,
                    ((uint8_t)(sl >= 0) ^ MOTOR_DIR_INVERT_L) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(R_DIR_GPIO_Port, R_DIR_Pin,
                    ((uint8_t)(sr >= 0) ^ MOTOR_DIR_INVERT_R) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  // 方向控制
  // 采用异或，将极性排除，统一将正数视为前进

  TIM1->CCR1 = bl;
  TIM1->CCR2 = br;
  // 写入寄存器

  pwm_L = sl;
  pwm_R = sr;
  // 保存遥测用状态
}

static void MOTOR_Stop(void)
{
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  pwm_L = 0;
  pwm_R = 0;
}

/* ==================== 光电 ==================== */
static void LINE_Read(void)
{
  MUX_get_value(&line_mux);
  line_err = LINE_Error(line_mux, &line_events);

  if (line_events & 0x01)                    /* LOST */
  {
    if (mode == MODE_RUN)
    {
      line_err = (line_err >= 0) ? LINE_LOST_DEV : -LINE_LOST_DEV;
      // 放大偏转
    }
    if (!line_lost_active) // 上升沿触发
    {
       line_lost_active = 1U;
       line_lost_since = tick_1ms; 
    }
  }
  else
  {
    line_lost_active = 0U;
  }
}

// 加权质心算法 - 回正
static int16_t LINE_Error(uint16_t mux, uint16_t *events)
{
  const int32_t span = (int32_t)(MULTIPLEXER_CHANNEL_NUM - 1);
  int32_t sum_w = 0, cnt = 0;
  uint8_t i, b;

  for (i = 0; i < MULTIPLEXER_CHANNEL_NUM; ++i)
  {
    b = (uint8_t)MUX_GET_CHANNEL(mux, i);
    if (!LINE_ACTIVE_LEVEL) { b = (uint8_t)!b; }
    if (b)
    {
      ++cnt;
      sum_w += (span - (int32_t)i * 2);
    }
  }
  // 状态机处理异常事件
  *events = 0;
  if (cnt == 0)                       { *events |= 0x01; return line_err; }  /* LOST  */
  if (cnt == MULTIPLEXER_CHANNEL_NUM) { *events |= 0x02; }                   /* CROSS */
  return (int16_t)((sum_w * 1000) / (cnt * span));
}

/* ==================== 陀螺仪 ==================== */
static void IMU_Init(void)
{
  uint32_t i;
  float sum = 0.0f;

  (void)dodo_BMI270_init();

  for (i = 0; i < 500U; ++i)                 // 1s 内采样求平均值作为零偏
  {
    dodo_BMI270_get_data();
    sum += BMI270_gyro_transition(BMI270_gyro_z);
    HAL_Delay(2);
  }
  // 此过程内不要动车，不要动车！
  gz_bias = sum / 500.0f;
  gz_lpf = 0.0f;
  yaw_d10 = 0;
  gyro_ready = 1U;
  // 恢复状态，记录 flag
}

// IMU 实时姿态解算，在主循环 5 ms 调用一次
static void IMU_Update(void)
{
  static uint32_t last_tick;
  uint32_t now = tick_1ms;
  float gz;

  if (!gyro_ready) { return; }

  dodo_BMI270_get_data();
  gz = BMI270_gyro_transition(BMI270_gyro_z) - gz_bias; // 减去零偏
  #if (GYRO_INVERT)
    gz = -gz;
  #endif
  gz_lpf += GYRO_LPF_ALPHA * (gz - gz_lpf); // 一阶低通滤波
  gz_dps = (int16_t)gz_lpf; // 取整，用于遥测输出

  // 判断是否静止
  // 角速度与线速度趋于 0
  // 风味 abs();
  if ((gz_lpf < YAW_STILL_GZ) && (gz_lpf > -YAW_STILL_GZ) && (v_L < 10) && (v_L > -10))
  {
    gz_bias += 0.0005f * gz;
    // 在线零偏追踪 - 积分修正零偏
  }
  else
  {
    yaw_d10 += (int32_t)(gz_lpf * (float)(now - last_tick) * 0.01f);
    // 航向角积分
    // * 0.01f 实际上是 * 0.001 * 10 (毫秒 -> 秒；角度放大 10 倍)
  }
  last_tick = now;
}

/* ==================== 三级串级 ==================== */
// 转向环 PD：线偏差 -> 目标角速度，运行频率-主循环 200 Hz
static void STEER_Update(void)
{
  tgt_gyro_dps = (int16_t)PID_Update(&pid_steer, (float)line_err,
                                     kp_l, 0.0f, kd_l, 0.0f, STEER_OUT_LIMIT);
}

// 角速度环 PD：目标角速度 -> 差速 mm/s，运行频率-ISR 1 kHz
static void GYRO_Update(void)
{
  float err = (float)tgt_gyro_dps - gz_lpf;
  diff_mm_s = (int32_t)PID_Update(&pid_gyro, err,
                                  kp_g, 0.0f, kd_g, 0.0f, GYRO_OUT_LIMIT);
}

/* 速度环 PI：目标速度 -> PWM 占空比，运行频率-ISR 1 kHz */
static void SPD_Update(void)
{
  float fl, fr;

  v_tgt_L = base_mm_s - diff_mm_s;
  v_tgt_R = base_mm_s + diff_mm_s;

  fl = ff_v * (float)v_tgt_L
     + PID_Update(&pid_spd_L, (float)(v_tgt_L - v_L), kp_v, ki_v, 0.0f, SPD_I_LIMIT, MOTOR_DUTY_MAX);
  fr = ff_v * (float)v_tgt_R
     + PID_Update(&pid_spd_R, (float)(v_tgt_R - v_R), kp_v, ki_v, 0.0f, SPD_I_LIMIT, MOTOR_DUTY_MAX);

  MOTOR_SetDuty((int32_t)fl, (int32_t)fr); // 前馈 + PI 输出
}

/* ==================== 初始化 / 复位 ==================== */
static void CTRL_Reset(void)
{
  PID_Reset(&pid_steer);
  PID_Reset(&pid_gyro);
  PID_Reset(&pid_spd_L);
  PID_Reset(&pid_spd_R);
  PID_Reset(&pid_yaw);

  v_tgt_L = v_tgt_R = 0;
  diff_mm_s = 0;
  base_mm_s = 0;
  cmd_open_l = cmd_open_r = 0;
  odo_cnt_L = odo_cnt_R = 0;
  odo_mm = 0;
  yaw_d10 = 0;
  line_err = 0;
  tgt_gyro_dps = 0;
  mode_prev = 0xFFU;                     // 赋值为棍母，下一拍强制触发 mode 变化计时
  line_lost_active = 0U;
}

static void CTRL_Init(void)
{
  CTRL_Reset();
  enc_idx = 0;
  enc_sum_L = enc_sum_R = 0;
  memset(enc_win_L, 0, sizeof(enc_win_L));
  memset(enc_win_R, 0, sizeof(enc_win_R));

  HAL_TIM_Encoder_Start(ENC_TIM_L, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(ENC_TIM_R, TIM_CHANNEL_ALL);
  enc_last_L = (uint16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_L);
  enc_last_R = (uint16_t)__HAL_TIM_GET_COUNTER(ENC_TIM_R);

  LINE_Read(); // 读取一次光电管状态，避免一开始就触发丢线保护

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) { Error_Handler(); }
}

/* ==================== 串口 ==================== */
static void CMD_Reply(const char *s, uint16_t n) // 回执发送
{
  // 若遥测占用串口，强行掐断
  if (huart3.gState != HAL_UART_STATE_READY) { (void)HAL_UART_AbortTransmit(&huart3); }
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)s, n, 5);
}

static void UART_RxStart(void)
{
  rx_head = rx_tail = 0;
  cmd_len = 0;
  if (HAL_UART_Receive_IT(&huart3, &rx_byte, 1) != HAL_OK) { Error_Handler(); }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) // 中断接收
{
  if (huart->Instance == USART3)
  {

    uint8_t next = (uint8_t)((rx_head + 1U) % CMD_LINE_MAX);
    if (next != rx_tail)
    { 
      rx_buf[rx_head] = rx_byte;
      rx_head = next;
    }// 环形队列缓冲，溢出时丢弃新数据，保持旧数据

    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
  }
}

static uint8_t CMD_Is(const char *line, const char *word) // 字符串匹配
{
  uint8_t n = 0;
  while (word[n]) { if (line[n] != word[n]) { return 0; } ++n; }
  return (line[n] == '\0' || line[n] == ' ') ? 1U : 0U; // 要求命令词后面是空格或结束符
}

static uint8_t CMD_ParseSet(char *args) // 参数解析，解析位置传参与命名传参
{
  char *tok;
  uint8_t i;

  if (strchr(args, '=') != NULL)
  {
    tok = strtok(args, " \t");
    while (tok != NULL)
    {
      char *eq = strchr(tok, '=');
      uint8_t found = 0;
      if (eq == NULL) { return 0U; }
      *eq = '\0';
      for (i = 0; i < PARAM_NUM; ++i)
      {
        if (strcmp(tok, param_tab[i].name) == 0)
        {
          float v = (float)atof(eq + 1);
          if ((v < param_tab[i].min) || (v > param_tab[i].max)) { return 0U; }
          *param_tab[i].p = v;
          found = 1U;
          break;
        }
      }
      if (!found) { return 0U; }
      tok = strtok(NULL, " \t");
    }
    return 1U;
  }

  i = 0;
  tok = strtok(args, " \t");
  while ((tok != NULL) && (i < PARAM_NUM))
  {
    float v = (float)atof(tok);
    if ((v < param_tab[i].min) || (v > param_tab[i].max)) { return 0U; }
    *param_tab[i].p = v;
    ++i;
    tok = strtok(NULL, " \t");
  }
  return (i > 0U) ? 1U : 0U;
}

static void CMD_Poll(void)
{
  char c;

  while (rx_tail != rx_head)
  {
    c = (char)rx_buf[rx_tail];
    rx_tail = (uint8_t)((rx_tail + 1U) % CMD_LINE_MAX);

    if (c == '\r') { continue; } // 丢弃回车符
    if (c == '\n')
    {
      cmd_line[cmd_len] = '\0'; // 补充结束符
      if (cmd_len > 0U) { CMD_Dispatch(cmd_line); } // 提交命令
      cmd_len = 0;
      continue;
    }
    if (cmd_len < (CMD_LINE_MAX - 1U)) { cmd_line[cmd_len++] = c; }
    else { cmd_len = 0; } // 缓冲溢出保护
  }
}

static void CMD_Dispatch(char *line) // 命令分发
{
  if (CMD_Is(line, "STOP"))
  {
    mode = MODE_STOP;
    MOTOR_Stop();
    CTRL_Reset();
    CMD_Reply("OK\r\n", 4);
    return;
  }
  if (CMD_Is(line, "START"))
  {
    SYS_StartSequence();
    CMD_Reply("OK\r\n", 4);
    return;
  }
  if (CMD_Is(line, "RESET"))
  {
    CTRL_Reset();
    CMD_Reply("OK\r\n", 4);
    return;
  }
  if (CMD_Is(line, "SET"))
  {
    if (CMD_ParseSet(line + 3)) { CMD_Reply("OK\r\n", 4); }
    else                        { CMD_Reply("ERR\r\n", 5); }
    return;
  }
  if (CMD_Is(line, "PWM"))
  {
    char *sp = strchr(line + 3, ' ');
    if (sp != NULL)
    {
      cmd_open_l = (int32_t)atoi(line + 3);
      cmd_open_r = (int32_t)atoi(sp + 1);
      mode = MODE_OPEN;
      CMD_Reply("OK\r\n", 4);
    }
    else { CMD_Reply("ERR\r\n", 5); }
    return;
  }
  if (CMD_Is(line, "STR"))
  {
    int hz = atoi(line + 3);
    if ((hz >= 0) && (hz <= 200)) { tel_hz = (uint16_t)hz; CMD_Reply("OK\r\n", 4); }
    else { CMD_Reply("ERR\r\n", 5); }
    return;
  }
  if (CMD_Is(line, "MODE"))
  {
    int m = atoi(line + 4);
    mode = (uint8_t)((m >= 0 && m <= 5) ? m : 0);
    if (mode == MODE_STOP) { MOTOR_Stop(); }
    CMD_Reply("OK\r\n", 4);
    return;
  }
  if (CMD_Is(line, "D"))                           /* 定距目标 mm */
  {
    tgt_dist_mm = atoi(line + 1);
    mode = MODE_STRAIGHT;
    CMD_Reply("OK\r\n", 4);
    return;
  }
  if (CMD_Is(line, "Y"))                           /* 旋转目标 deg (signed) */
  {
    tgt_yaw_d10 = (int32_t)(atof(line + 1) * 10.0f);
    mode = MODE_TURN;
    CMD_Reply("OK\r\n", 4);
    return;
  }
  if (CMD_Is(line, "RING"))                        /* 1 = 右环岛，-1 = 左环岛 */
  {
    ring_dir = (int8_t)((atoi(line + 4) < 0) ? -1 : 1);
    ring_yaw0 = yaw_d10;
    mode = MODE_ROUND;
    CMD_Reply("OK\r\n", 4);
    return;
  }
  CMD_Reply("ERR\r\n", 5);
}

static void TEL_Emit(void)
{
  int n;

  if (huart3.gState != HAL_UART_STATE_READY) { return; }   // 若串口阻塞，则丢弃本次遥测
  n = snprintf(tel_buf, sizeof(tel_buf), "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
               (int)v_tgt_L, (int)v_L, (int)v_tgt_R, (int)v_R,
               (int)pwm_L, (int)pwm_R, (int)line_err,
               (int)tgt_gyro_dps, (int)gz_dps, (int)diff_mm_s,
               (int)odo_mm, (int)yaw_d10, (int)line_mux);
  if (n > 0) { (void)HAL_UART_Transmit_IT(&huart3, (uint8_t *)tel_buf, (uint16_t)n); }
}

/* ==================== TIM2 1 kHz ==================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  int16_t dl, dr;

  if (htim->Instance != TIM2) { return; }

  ++tick_1ms;

  ENC_ReadDelta(&dl, &dr);
  ODO_Push(dl, dr);

  if (mode >= MODE_RUN)
  {
    GYRO_Update();
    SPD_Update();
  }
  else if (mode == MODE_OPEN)
  {
    MOTOR_SetDuty(cmd_open_l, cmd_open_r);
  }
  else
  {
    MOTOR_Stop();
  }
}

/* ==================== 模式机（主循环 200 Hz） ==================== */
static void CTRL_ModeStep(void)
{
  int32_t remain, e;

  switch (mode)
  {
    case MODE_RUN:
      base_mm_s = (int32_t)vset;
      break;

    case MODE_STRAIGHT: // 定距运行
      remain = tgt_dist_mm - odo_mm;
      if (remain <= DIST_TOL_MM) // 斜率减速
      {
        base_mm_s = 0;
        mode = MODE_STOP;
        MOTOR_Stop();
        break;
      }
      base_mm_s = (remain < DIST_DECEL_MM)
                ? (((int32_t)vset * remain) / DIST_DECEL_MM) // 线性插值的斜率减速
                : (int32_t)vset;
      break;

    case MODE_TURN: // 原地旋转
      base_mm_s = 0;
      e = tgt_yaw_d10 - yaw_d10;
      if ((e < TURN_TOL_D10) && (e > -TURN_TOL_D10))
      {
        mode = MODE_STOP;
        MOTOR_Stop();
        break;
      }
      tgt_gyro_dps = (int16_t)PID_Update(&pid_yaw, (float)e,
                                         kp_y, 0.0f, kd_y, 0.0f, STEER_OUT_LIMIT);
      break;

    case MODE_ROUND: // 环岛
      base_mm_s = ((int32_t)vset * RING_SPEED_PCT) / 100;
      tgt_gyro_dps = (int16_t)(RING_TURN_DPS * (float)ring_dir);
      if (((yaw_d10 - ring_yaw0) > RING_SWEEP_D10) || ((ring_yaw0 - yaw_d10) > RING_SWEEP_D10))
      {
        mode = MODE_RUN; // 出岛
      }
      break;

    default:
      base_mm_s = 0;
      break;
  }
}

/* ==================== 发车 / 失效保护 ==================== */
static uint8_t RUN_KeyPressed(void)
{
  static uint32_t last_press;

  if (mode != MODE_STOP) { return 0U; }
  if (HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin) != GPIO_PIN_RESET) { return 0U; }
  HAL_Delay(20); // 消抖
  if (HAL_GPIO_ReadPin(GPIO_RUN_GPIO_Port, GPIO_RUN_Pin) != GPIO_PIN_RESET) { return 0U; }
  if ((tick_1ms - last_press) < 500U) { return 0U; }
  last_press = tick_1ms;
  return 1U;
}

/* 启动流程了 */
static void SYS_StartSequence(void)
{
  CTRL_Reset();
  if (mode == MODE_STOP) { mode = MODE_RUN; } // 防唐
}

static void SYS_Failsafe(void)
{
  // 切换模式时启动计时
  if (mode != mode_prev)
  {
    mode_prev  = mode;
    mode_since = tick_1ms;
  }

  // 旋转超时保护
  if ((mode == MODE_TURN) && ((tick_1ms - mode_since) > TURN_TIMEOUT_MS))
  {
    mode = MODE_STOP;
    MOTOR_Stop();
    return;
  }

  // 环岛超时保护
  if ((mode == MODE_ROUND) && ((tick_1ms - mode_since) > RING_TIMEOUT_MS))
  {
    mode = MODE_STOP;
    MOTOR_Stop();
    return;
  }

  // 丢线保护
  if ((mode == MODE_RUN) && line_lost_active && ((tick_1ms - line_lost_since) > LINE_LOST_MS))
  {
    mode = MODE_STOP;
    MOTOR_Stop();
    return;
  }

  // 定距运行迷失保护
  if ((mode == MODE_STRAIGHT) && (odo_mm > (tgt_dist_mm + 300)))
  {
    mode = MODE_STOP;
    MOTOR_Stop();
  }
}

/* USER CODE END 0 */
void Init_brushless_motor(void)
{
  /* PA11 idles low: the first high level then also lasts exactly 0.9 ms,
     because the DMA writes the first slot 0.1 ms after TIM1 is started. */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  ESC_BuildDmaTable(ESC_START_PULSE_SLOTS);
  esc_period_count = 0U;
  hdma_tim1_up.XferCpltCallback = ESC_DmaTransferComplete;
  if (HAL_DMA_Start_IT(&hdma_tim1_up,(uint32_t)esc_dma_table,(uint32_t)&GPIOA->BSRR,ESC_PULSE_SLOTS) != HAL_OK)
    Error_Handler();
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
  if (IF_start_Brushless_motor)
    while (esc_period_count < ESC_START_PERIODS)
      HAL_Delay(200);//等待无刷电桥负压启动完成
  /* Disabled fan: return immediately; DMA completes the existing
     0.9 ms -> 1.0 ms idle-signal sequence in the background. */
}
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  Init_brushless_motor(); //等待无刷电调负压启动完成
  /* USER CODE BEGIN 2 */

  IMU_Init();         /* 陀螺仪 + 静止标零偏（约 1 s） */
  CTRL_Init();        /* 编码器 + 预读光电 + 启动 TIM2 1 kHz */
  UART_RxStart();     /* USART3 单字节中断接收 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    static uint32_t line_last, tel_last;

    IMU_Update();                                  // 500 Hz 陀螺仪解算

    if ((tick_1ms - line_last) >= LINE_PERIOD)     // 200 Hz 光电读取
    {
      line_last = tick_1ms;
      LINE_Read();
      if (mode == MODE_RUN) { STEER_Update(); }     // 光电转向
      CTRL_ModeStep();                              // 模式机
    }

    CMD_Poll();                                     // 50 Hz 串口命令解析

    if (RUN_KeyPressed()) { SYS_StartSequence(); }  // 按键发车

    if ((tick_1ms - safe_last) >= SAFE_PERIOD)      // 100 Hz 失效检查
    {
      safe_last = tick_1ms;
      SYS_Failsafe();
    }

    if ((tel_hz != 0U) && ((tick_1ms - tel_last) >= (1000U / tel_hz))) // 50 Hz 遥测
    {
      tel_last = tick_1ms;
      TEL_Emit();
    }
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | R_DIR_Pin | GPIO_PIN_11 | MUX_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_DIR_Pin | MUX_1_Pin | MUX_2_Pin | MUX_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_RUN_Pin */
  GPIO_InitStruct.Pin = GPIO_RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_RUN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : L_DIR_Pin */
  GPIO_InitStruct.Pin = L_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_DIR_Pin PA11 */
  GPIO_InitStruct.Pin = R_DIR_Pin | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_READ_Pin */
  GPIO_InitStruct.Pin = MUX_READ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MUX_READ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_0_Pin */
  GPIO_InitStruct.Pin = MUX_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(MUX_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_1_Pin MUX_2_Pin MUX_3_Pin */
  GPIO_InitStruct.Pin = MUX_1_Pin | MUX_2_Pin | MUX_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
