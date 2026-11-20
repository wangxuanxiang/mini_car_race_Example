#include "track_logic.h"
#include "main.h"
#include "dodo_BMI270.h"

volatile uint8_t dbg_state      = 0;
volatile float   dbg_yaw        = 0.0f;
volatile float   dbg_edge_rise  = -1.0f;
volatile float   dbg_ra_turned  = 0.0f;
volatile float   dbg_odom       = 0.0f;
volatile uint8_t dbg_ra_dir     = 0;

/* ---------------- 小工具 ---------------- */
static float tl_abs(float x) { return (x < 0.0f) ? -x : x; }

static float tl_clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float tl_ang_diff(float a, float b)
{
    float d = a - b;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static float tl_yaw_sign(void) { return (TL_GYRO_Z_SIGN >= 0.0f) ? 1.0f : -1.0f; }
static float tl_dir_sign(uint8_t dir) { return (dir == 1U) ? 1.0f : -1.0f; }

/* ---------------- 边缘上升距离 ---------------- */
typedef struct {
    uint8_t cnt;      /* 当前边缘亮灯数 0~3 */
    uint8_t phase;    /* 0=空闲 1=已见第1路 2=已见第3路 */
    float   odom1;    /* 第1路亮时的里程 */
    float   rise;     /* 第1路到第3路的距离 cm，-1=未完成 */
} edge_t;

/* ---------------- 状态 ---------------- */
static track_state_t s_state;
static edge_t s_left, s_right;

static float    s_yaw, s_gyro_bias, s_odom_cm;
static uint32_t s_last_ms;

static uint8_t  s_armed;        /* 允许触发新特征 */
static uint8_t  s_zero_active;  /* 边缘归零计时 */
static uint32_t s_zero_t0;
static uint32_t s_cooldown_until;   /* 出环冷却结束时刻 */

static uint8_t  s_feat_dir;     /* 0=无 1=左 2=右；环岛候选方向 */
static uint32_t s_feat_t0;

static uint8_t  s_ra_dir;
static float    s_ra_entry_heading;
static float    s_ra_net_max;
static float    s_ra_comp;
static uint8_t  s_ra_inner;
static float    s_ra_inner_min;
static float    s_ra_exit_offset;   /* 本次环岛的出环偏移 */
static uint32_t s_ra_start_t0;      /* 入环触发时刻 */
static uint32_t s_ra_t0;

/* ---------------- 初始化 / 陀螺仪零偏 ---------------- */
void track_init(void)
{
    s_state = TS_NORMAL;
    s_left.cnt = 0U;  s_left.phase = 0U;  s_left.odom1 = 0.0f;  s_left.rise = -1.0f;
    s_right = s_left;

    s_yaw = 0.0f;
    s_gyro_bias = 0.0f;
    s_odom_cm = -46.0f;
    s_last_ms = HAL_GetTick();

    s_armed = 1U;
    s_zero_active = 0U;
    s_zero_t0 = 0U;
    s_cooldown_until = 0U;

    s_feat_dir = 0U;
    s_feat_t0 = 0U;

    s_ra_dir = 0U;
    s_ra_entry_heading = 0.0f;
    s_ra_net_max = 0.0f;
    s_ra_comp = 0.0f;
    s_ra_inner = 0U;
    s_ra_inner_min = 0.0f;
    s_ra_exit_offset = TL_RA_EXIT_OFFSET_SMALL_DEG;
    s_ra_start_t0 = 0U;
    s_ra_t0 = 0U;

    dbg_state = 0U;
    dbg_yaw = 0.0f;
    dbg_edge_rise = -1.0f;
    dbg_ra_turned = 0.0f;
    dbg_odom = -46.0f;
    dbg_ra_dir = 0U;
}

void track_calibrate_gyro(void)
{
    float sum = 0.0f;
    uint16_t n = 0U;
    uint32_t t0 = HAL_GetTick();

    while (HAL_GetTick() - t0 < 600U) {
        dodo_BMI270_get_data();
        sum += BMI270_gyro_transition(BMI270_gyro_z);
        n++;
        HAL_Delay(2);
    }

    s_gyro_bias = (n > 0U) ? (sum / (float)n) : 0.0f;
    s_yaw = 0.0f;
    s_last_ms = HAL_GetTick();
}

/* ---------------- 内部更新 ---------------- */
static void tl_yaw_update(float gyro_z, uint32_t now)
{
    float dt = (now - s_last_ms) * 0.001f;
    s_last_ms = now;
    if (dt > 0.05f) dt = 0.05f;   /* 防长时间中断 */
    s_yaw += tl_yaw_sign() * (gyro_z - s_gyro_bias) * dt;
}

static void tl_edge_update(edge_t *e, uint8_t cnt, float odom)
{
    if (cnt == 0U) {
        e->phase = 0U;
        e->rise = -1.0f;
    } else {
        if (e->phase == 0U) {
            e->phase = 1U;
            e->odom1 = odom;
            e->rise = -1.0f;
        }
        if (cnt >= 3U && e->phase == 1U) {
            e->rise = odom - e->odom1;
            e->phase = 2U;
        }
    }
    e->cnt = cnt;
}

static void tl_start_ra(uint8_t dir, uint32_t now)
{
#if (RA_FORCE_DIR == 1) || (RA_FORCE_DIR == 2)
    dir = (uint8_t)RA_FORCE_DIR;   /* 强制方向：1=左(逆时针) 2=右(顺时针) */
#endif
    dbg_ra_dir = dir;
    s_ra_dir = dir;
    s_ra_entry_heading = s_yaw;
    s_ra_net_max = 0.0f;
    s_ra_comp = 0.0f;
    s_ra_inner = 0U;
    s_ra_inner_min = 0.0f;
    s_ra_exit_offset = TL_RA_EXIT_OFFSET_SMALL_DEG;
    s_ra_start_t0 = now;
    dbg_ra_turned = 0.0f;   /* 新环岛从 0 开始打印 */
    s_ra_t0 = now;
    s_state = TS_RA_ENTRY;
    s_armed = 0U;
}

/* ---------------- 主更新 ---------------- */
void track_update(const track_input_t *in, track_output_t *out)
{
    uint32_t now = HAL_GetTick();

    tl_yaw_update(in->gyro_z, now);
    s_odom_cm += in->d_cm;

    dbg_yaw = s_yaw;
    dbg_odom = s_odom_cm;

    tl_edge_update(&s_left,  in->left3,  s_odom_cm);
    tl_edge_update(&s_right, in->right3, s_odom_cm);

    if (in->left3 == 3U)       dbg_edge_rise = s_left.rise;
    else if (in->right3 == 3U) dbg_edge_rise = s_right.rise;
    else                       dbg_edge_rise = -1.0f;

    /* 重新武装：NORMAL 且两侧边缘都归零 20ms 以上 */
    if (s_state == TS_NORMAL && in->left3 == 0U && in->right3 == 0U) {
        if (!s_zero_active) {
            s_zero_active = 1U;
            s_zero_t0 = now;
        } else if ((now - s_zero_t0) >= 20U &&
                   now >= s_cooldown_until) {
            s_armed = 1U;
        }
    } else {
        s_zero_active = 0U;
    }

    out->override = 1U;
    out->diff = 0.0f;

    switch (s_state) {

    case TS_NORMAL: {
        /* 单侧边缘3路全亮，另一侧最多1路（排除四岔口3/3） */
        uint8_t one_sided = 0U;
        if (in->left3 == 3U && in->right3 <= 2U)       one_sided = 1U;
        else if (in->right3 == 3U && in->left3 <= 2U)  one_sided = 2U;

        uint8_t dir = 0U;

        if (s_armed && one_sided != 0U &&
            in->active >= TL_WIDE_MIN && in->active <= TL_WIDE_MAX) {

            edge_t *e = (one_sided == 1U) ? &s_left : &s_right;

            /* 只认“慢上升”：90°直角≈0，45°/60°≈3~6，30°岔口≈10 */
            if (e->phase == 2U && e->rise >= TL_EDGE_SLOW_CM) {
                dir = one_sided;
            }
        }

        if (dir != 0U) {
            if (dir != s_feat_dir) {
                s_feat_dir = dir;
                s_feat_t0   = now;
            } else if ((now - s_feat_t0) >= TL_FEATURE_CONFIRM_MS) {
                tl_start_ra(dir, now);
                s_feat_dir = 0U;
            }
        } else {
            s_feat_dir = 0U;
        }

        if (s_state == TS_NORMAL) {
            out->override = 0U;   /* 用 PPDD * 1.5 */
        }
        break;
    }

    case TS_RA_ENTRY: {
        float target = s_ra_entry_heading +
                       tl_dir_sign(s_ra_dir) * TL_RA_ENTRY_DEG;
        float err = tl_ang_diff(target, s_yaw);

        out->diff = tl_clamp(TL_RA_ENTRY_KP * err,
                             -TL_RA_ENTRY_DIFF_MAX, TL_RA_ENTRY_DIFF_MAX);

        if (tl_abs(err) <= 4.0f ||
            (now - s_ra_t0) > TL_RA_ENTRY_TIMEOUT_MS) {
            s_state = TS_RA_RUN;
            s_ra_t0 = now;
        }
        break;
    }

    case TS_RA_RUN: {
        /* 净转角 + 内环补偿：内环结束再补实际下降量 */
        float net = tl_dir_sign(s_ra_dir) * (s_yaw - s_ra_entry_heading);

        if (!s_ra_inner) {
            if ((s_ra_net_max - net) >= TL_RA_INNER_DETECT_DEG) {
                /* 进入内环：开始记录最低点 */
                s_ra_inner = 1U;
                s_ra_inner_min = net;
                s_ra_exit_offset = TL_RA_EXIT_OFFSET_BIG_DEG;  /* 大环出环偏移 */
            } else if (net > s_ra_net_max) {
                s_ra_net_max = net;
            }
        } else {
            if (net < s_ra_inner_min) {
                s_ra_inner_min = net;
            }
            if (net > (s_ra_inner_min + TL_RA_INNER_END_DEG)) {
                /* 内环结束：补偿实际下降的净角 */
                s_ra_comp += (s_ra_net_max - s_ra_inner_min);
                s_ra_net_max = s_ra_inner_min;
                s_ra_inner = 0U;
            }
        }

        float turned = net + s_ra_comp;
        dbg_ra_turned = turned;

        out->override = 0U;   /* 环内用 PPDD * 1.0，不用1.5 */

        if (turned >= TL_RA_EXIT_YAW_DEG &&
            (now - s_ra_start_t0) >= TL_RA_MIN_RUN_MS) {
            s_state = TS_RA_EXIT;
            s_ra_t0 = now;
        } else if ((now - s_ra_t0) > TL_RA_RUN_TIMEOUT_MS) {
            /* 5s 还没出环：直接回 NORMAL，不进 RA_EXIT，不打角 */
            s_state = TS_NORMAL;
            out->override = 0U;
        }
        break;
    }

    case TS_RA_EXIT: {
        float target = s_ra_entry_heading + s_ra_exit_offset;
        float err = tl_ang_diff(target, s_yaw);
        out->diff = tl_clamp(TL_RA_EXIT_KP * err,
                             -TL_RA_EXIT_DIFF_MAX, TL_RA_EXIT_DIFF_MAX);

        if (tl_abs(err) <= 4.0f ||
            (now - s_ra_t0) > TL_RA_EXIT_TIMEOUT_MS) {
            s_state = TS_NORMAL;
            s_armed = 0U;
            s_cooldown_until = now + TL_RA_COOLDOWN_MS;
        }
        break;
    }

    default:
        s_state = TS_NORMAL;
        out->override = 0U;
        break;
    }

    out->state = (uint8_t)s_state;
    dbg_state = (uint8_t)s_state;
}
