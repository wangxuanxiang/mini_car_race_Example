#ifndef TRACK_LOGIC_H
#define TRACK_LOGIC_H

#include <stdint.h>

/* =====================================================================
 * 只保留环岛的赛道逻辑
 *
 * 设计：
 *   - 不再有直角状态、快上升阈值、88° yaw 闭环；
 *   - 不再有十字状态，四岔口交给 NORMAL 的 PPDD；
 *   - 唯一特征：单侧边缘 3 路 + active 6~9 + 边缘上升距离 >= TL_EDGE_SLOW_CM
 *     连续 4ms -> 环岛；
 *   - 环岛：入环 33°，环内 PPDD×1.0，累计 330° 出环，回正到 entry heading。
 *
 * 90°直角（方块/底部折线）全部由 NORMAL 的 PPDD×1.5 跟线处理。
 * ===================================================================== */

/* ================= 必须标定的两个量 ================= */
/* 编码器：推车100cm，看dbg_odom（可先把本值设为1.0），
   然后 TL_ODOM_COUNTS_PER_CM = dbg_odom / 100.0f */
#define TL_ODOM_COUNTS_PER_CM   376.0f   /* 旧写法 3.76/100 在C里实际等效 /376；推1m显示100，保留这个标定 */

/* 陀螺仪符号：手拿车左转90°，dbg_yaw增大取+1，减小取-1 */
#define TL_GYRO_Z_SIGN          (+1.0f)

/* ================= 环岛特征 ================= */
#define TL_WIDE_MIN             6
#define TL_WIDE_MAX             10

/* 只判“慢上升”。0.1m线宽、直道3~4路时：
 *   90°直角 ≈0cm，45°/60° ≈3~6cm，30°岔口 ≈10cm。
 * 先设7.0，实测后放在45°/60°和30°岔口之间。 */
#define TL_EDGE_SLOW_CM         1.9f
#define TL_FEATURE_CONFIRM_MS   4

/* ================= 环岛 ================= */
#define TL_RA_ENTRY_DEG         33.0f   /* 六边形上顶点支线约30° */
#define TL_RA_ENTRY_KP          0.22f
#define TL_RA_ENTRY_DIFF_MAX    12.0f
#define TL_RA_ENTRY_TIMEOUT_MS  600

#define TL_RA_EXIT_YAW_DEG      330.0f  /* 一圈360°，提前30°回正 */
#define TL_RA_INNER_DETECT_DEG  295.0f  /* 净角下降超过此值，认为进入内环 */
#define TL_RA_INNER_END_DEG      5.0f   /* 净角从最低点回升超过此值，认为内环结束 */
#define TL_RA_EXIT_KP           0.22f
#define TL_RA_EXIT_DIFF_MAX     12.0f
#define TL_RA_EXIT_TIMEOUT_MS   600
#define TL_RA_RUN_TIMEOUT_MS    4600U   /* 环内 5s 未出环：直接回 NORMAL，不打角 */
#define TL_RA_COOLDOWN_MS       6000U   /* 出环后 1s 内禁止再次入环 */

/* ================= 环岛入环方向 =================
 * 1 = 左入环（逆时针跑）
 * 2 = 右入环（顺时针跑）
 * 0 = 按边缘自动（原逻辑）
 *
 * 顺时针/逆时针切换时只改这一个值，重新编译即可。
 * 逆时针：1；顺时针：2。
 */
#define RA_FORCE_DIR            1

typedef enum {
    TS_NORMAL = 0,
    TS_RA_ENTRY,
    TS_RA_RUN,
    TS_RA_EXIT
} track_state_t;

typedef struct {
    float   line_pos;   /* -5.5 ~ +5.5，正=右 */
    uint8_t active;     /* 亮灯总数 */
    uint8_t left3;      /* 0~2 三路之和 */
    uint8_t right3;     /* 9~11 三路之和 */
    float   gyro_z;     /* 原始 deg/s，模块内部减零偏 */
    float   d_cm;       /* 本拍前进距离 cm */
} track_input_t;

typedef struct {
    float   diff;       /* 差速指令 */
    uint8_t override;   /* 1=用diff，0=用PPDD */
    uint8_t state;      /* 当前状态，供调试 */
} track_output_t;

/* 调试变量，Keil Watch 直接看 */
extern volatile uint8_t dbg_state;
extern volatile float   dbg_yaw;
extern volatile float   dbg_edge_rise;
extern volatile float   dbg_ra_turned;
extern volatile float   dbg_odom;
extern volatile uint8_t dbg_ra_dir;

void track_init(void);
void track_calibrate_gyro(void);
void track_update(const track_input_t *in, track_output_t *out);

#endif
