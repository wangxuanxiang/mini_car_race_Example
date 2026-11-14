#include "motor_control.h"
#include "track_logic.h"

//电机端口定义
#define MotorGPIO_Left	L_DIR_GPIO_Port
#define MotorPIN_Left	L_DIR_Pin
#define MotorGPIO_Right	R_DIR_GPIO_Port
#define MotorPIN_Right	R_DIR_Pin

//位置环ppdd参数
#define PPDD_KP1 10.0f
#define PPDD_KP2 1.0f
#define PPDD_KD  0.0f
#define PPDD_GKD 0.0f

//速度环PID参数
#define L_SPEED_KP 200.0f
#define L_SPEED_KI 0.0f
#define L_SPEED_KD 0.0f
#define R_SPEED_KP 200.0f
#define R_SPEED_KI 0.0f
#define R_SPEED_KD 0.0f

//目标速度
#define TARGET_SPEED 25.0f

/* 入弯前降速 + 前馈制动参数 */
#define BRAKE_MS      0U     /* 制动脉冲时间 ms */
#define SLOW_MS       400U    /* 弯中低速窗口 ms */
#define BRAKE_SPEED   2.0f    /* 制动目标速度（>0.5 才能允许反向制动） */
#define SLOW_SPEED    25.0f   /* 弯中目标速度 */
#define TL_NEG_MAX    4.0f   /* 减速转弯时允许的负内轮限幅 */

//速度积分限幅
#define SPEED_I_MAX 2000.0f

//PWM输出限制
#define PWM_MAX 1100
#define PWM_MIN -1100

float target_speed_now=TARGET_SPEED;
float weighted_sum=0.0f;
float last_position_error=0.0f;
uint8_t active_count=0;
uint8_t stop_flag=0;
uint8_t mux_array[12]={0};
uint8_t flag=0;
float debug1=0,debug2=0,debug3=0,debug4=0;

//控制器
PID_Controller steering_ppdd={
    .kp=PPDD_KP1, .kp2=PPDD_KP2, .kd=PPDD_KD, .gkd=PPDD_GKD,
};

PID_Controller left_pid={
	.kp=L_SPEED_KP, .ki=L_SPEED_KI, .kd=L_SPEED_KD, .integral_max=SPEED_I_MAX
};

PID_Controller right_pid={
	.kp=R_SPEED_KP, .ki=R_SPEED_KI, .kd=R_SPEED_KD, .integral_max=SPEED_I_MAX
};

//左右电机
motor_t motor_left;
motor_t motor_right;

//motor初始化函数
void motor_init(void){
    motor_left.speed_pid=left_pid;
    motor_left.motor_GPIO=MotorGPIO_Left;
    motor_left.motor_PIN=MotorPIN_Left;
    motor_left.motor_DIR=GPIO_PIN_SET;
    motor_left.CCR=1;

    motor_right.speed_pid=right_pid;
    motor_right.motor_GPIO=MotorGPIO_Right;
    motor_right.motor_PIN=MotorPIN_Right;
    motor_right.motor_DIR=GPIO_PIN_SET;
    motor_right.CCR=2;
}

void track_debug_update(uint16_t mux_value, float gyro_z,
                        float left_encoder, float right_encoder)
{
    float line_position = calculate_line_position(mux_value);

    track_input_t in;
    in.line_pos = line_position;
    in.active   = active_count;
    in.left3    = (uint8_t)(mux_array[0] + mux_array[1] + mux_array[2]);
    in.right3   = (uint8_t)(mux_array[9] + mux_array[10] + mux_array[11]);
    in.gyro_z   = gyro_z;
    in.d_cm     = 0.5f * (left_encoder + right_encoder) / TL_ODOM_COUNTS_PER_CM;

    track_output_t out;
    track_update(&in, &out);
    (void)out;
}

//设置PWM波
void setPWM(motor_t motor){
    uint16_t pwm_output=my_abs_int(motor.pwm_output);
    if(motor.pwm_output==0||stop_flag){
        if(motor.CCR==1) TIM1->CCR1=pwm_output;
        else if(motor.CCR==2) TIM1->CCR2=pwm_output;
        return;
    }
    if(motor.CCR==1) TIM1->CCR1=pwm_output;
    else if(motor.CCR==2) TIM1->CCR2=pwm_output;
    
    if(motor.pwm_output>0) HAL_GPIO_WritePin(motor.motor_GPIO,motor.motor_PIN,motor.motor_DIR);
    else HAL_GPIO_WritePin(motor.motor_GPIO,motor.motor_PIN,!motor.motor_DIR);
}

//停止PWM
void stopPWM(motor_t motor){
    motor_left.pwm_output=0;
    motor_right.pwm_output=0;
    TIM1->CCR1=0;
    TIM1->CCR2=0;
    stop_flag=1;
}

//计算光电管中心位置偏差
float calculate_line_position(uint16_t mux_value){
    static uint8_t  last_valid_active = 0U;
    static float    last_valid_pos    = 0.0f;

    weighted_sum=0.0f;
    active_count=0;
    for(uint8_t i=0;i<12;i++){
        mux_array[i]=MUX_GET_CHANNEL(mux_value,i);
        if(MUX_GET_CHANNEL(mux_value,i)){
            weighted_sum+=i-5.5f;
            active_count++;
        }
    }
    if(active_count==0){
        /* 丢线：上次 active<=3 且 |偏移|<=0.5 时，固定向右 */
        float p = last_valid_pos;
        if(p < 0.0f) p = -p;

        if(last_valid_active <= 3U && p <= 0.5f){
            return 0.1f;
        }
        return last_position_error;
    }

    last_valid_active = active_count;
    last_valid_pos = weighted_sum/active_count;
    return last_valid_pos;
}

//并联PID控制主函数
void parallel_pid_control(uint16_t mux_value,float gyro_z,float left_encoder,float right_encoder){
    float line_position=calculate_line_position(mux_value);
    last_position_error=line_position;

    //原PPDD转向环，保留
    float base=ppdd_calculate(&steering_ppdd,0.0f,line_position,gyro_z);

    //赛道元素状态机输入
    track_input_t in;
    in.line_pos = line_position;
    in.active   = active_count;
    in.left3    = (uint8_t)(mux_array[0]+mux_array[1]+mux_array[2]);
    in.right3   = (uint8_t)(mux_array[9]+mux_array[10]+mux_array[11]);
    in.gyro_z   = gyro_z;
    in.d_cm     = 0.5f*(left_encoder+right_encoder)/TL_ODOM_COUNTS_PER_CM;

    track_output_t out;
    track_update(&in,&out);

    /* ===== 入弯前降速 + 前馈制动 =====
     * 1) 线变宽(active>=6)且任一侧边缘亮时，认为要入弯；
     *    十字(3/3, active 10~12)和直角(3/2、3/3, active 6~12)都会触发；
     * 2) 第一次触发：先给 BRAKE_SPEED 持续 BRAKE_MS；
     * 3) 之后给 SLOW_SPEED，持续 SLOW_MS；
     * 4) 直道 active 3~4、45°/60° active 4~5 不触发。
     */
    static uint32_t brake_until = 0U;
    static uint32_t slow_until  = 0U;
    uint32_t slow_now_ms = HAL_GetTick();
    uint8_t brake_active = 0U;

    if (out.state == TS_NORMAL && out.override == 0U &&
        in.active >= 6U && (in.left3 >= 1U || in.right3 >= 1U)) {

        if (slow_now_ms >= slow_until) {
            /* 新的一次入弯：只触发一次制动 */
            brake_until = slow_now_ms + BRAKE_MS;
            slow_until  = slow_now_ms + SLOW_MS;
        } else {
            /* 还在同一个弯里：保持低速窗口，不重复制动 */
            slow_until = slow_now_ms + SLOW_MS;
        }
    }

    if (slow_now_ms < brake_until) {
        target_speed_now = BRAKE_SPEED;   /* 前馈制动：最大反拖 */
        brake_active = 1U;
    } else if (slow_now_ms < slow_until) {
        target_speed_now = SLOW_SPEED;    /* 弯中低速 */
    } else {
        target_speed_now = TARGET_SPEED;  /* 直道恢复 */
    }

    /* 指定里程段限速：470~640 时目标速度 22 */
    if (dbg_odom >= 470.0f && dbg_odom <= 640.0f) {
        target_speed_now = 22.0f;
    }

    /* 指定里程段限速：2600~2800、5520~5750 时目标速度 17 */
    if ((dbg_odom >= 2600.0f && dbg_odom <= 2800.0f) ||
        (dbg_odom >= 5520.0f && dbg_odom <= 5750.0f)) {
        target_speed_now = 17.0f;
    }

    //差速选择：
    //  NORMAL / RA_RUN -> PPDD * 1.5
    //  其它状态        -> 状态机给出的yaw闭环差速
    float differential_speed;
    if(out.override){
        differential_speed=out.diff;
    }else{
        differential_speed=base*1.5f;
    }

    /* 前馈制动期间不打角，先直着把速度刹下来 */
    if (brake_active) {
        differential_speed *= 0.5f;
    }

    debug1 = target_speed_now;
    debug2 = differential_speed;

    /* 减速转弯时允许适度负内轮：只在 slow/brake 窗口内 */
    uint8_t allow_neg = (slow_now_ms < slow_until || brake_active) ? 1U : 0U;
    float neg_max = allow_neg ? TL_NEG_MAX : 0.0f;

    float tl = target_speed_now - differential_speed;
    float tr = target_speed_now + differential_speed;
    if (tl < -neg_max) tl = -neg_max;
    if (tr < -neg_max) tr = -neg_max;
    if (!allow_neg) {
        if (tl < 0.0f) tl = 0.0f;
        if (tr < 0.0f) tr = 0.0f;
    }
    motor_left.target_speed = tl;
    motor_right.target_speed = tr;

    debug3 = motor_left.target_speed;
    debug4 = motor_right.target_speed;

    motor_left.current_speed = left_encoder;
    motor_right.current_speed = right_encoder;

    float pwm_l = pid_calculate(&motor_left.speed_pid, motor_left.target_speed, motor_left.current_speed);
    float pwm_r = pid_calculate(&motor_right.speed_pid, motor_right.target_speed, motor_right.current_speed);
    pwm_l = constrain(pwm_l, PWM_MIN, PWM_MAX);
    pwm_r = constrain(pwm_r, PWM_MIN, PWM_MAX);
    if (motor_left.target_speed >= 0.0f && motor_left.target_speed < 0.5f && pwm_l < 0.0f) pwm_l = 0.0f;
    if (motor_right.target_speed >= 0.0f && motor_right.target_speed < 0.5f && pwm_r < 0.0f) pwm_r = 0.0f;
    motor_left.pwm_output  = (int16_t)pwm_l;
    motor_right.pwm_output = (int16_t)pwm_r;
    setPWM(motor_left);
    setPWM(motor_right);
    /* ===== 调试打印：50ms 一行 ===== */
static uint32_t dbg_last = 0U;
uint32_t now_ms = HAL_GetTick();

if ((now_ms - dbg_last) >= 50U) {
    uint16_t bits = 0U;
    uint8_t  i;
    dbg_last = now_ms;

    for (i = 0U; i < 12U; i++) {
        bits = (uint16_t)((bits << 1) | (mux_array[i] ? 1U : 0U));
    }
    printf("%d,%d,%u,%.2f\r\n",
           (int)dbg_odom,
           (int)(dbg_edge_rise * 10.0f),
           (unsigned)out.state,
           dbg_ra_turned);

    // printf("%lu,%u,%u,%d,%d,%d,%d,%d,%d,%d,%03X,%d\r\n",
    //        (unsigned long)now_ms,              /* 1 时间 */
    //        (unsigned)out.state,                /* 2 状态 0~3 */
    //        (unsigned)active_count,             /* 3 亮灯数 */
    //        (int)(dbg_edge_rise * 10.0f),       /* 4 rise 0.1cm */
    //        (int)dbg_yaw,                       /* 5 航向 */
    //        (int)dbg_ra_turned,                 /* 6 环内角度 */
    //        (int)dbg_odom,                      /* 7 里程 */
    //        (int)(differential_speed * 10.0f),  /* 8 差速 ×10 */
    //        (int)left_encoder,                  /* 9 左编码器 */
    //        (int)right_encoder,                 /* 10 右编码器 */
    //        (unsigned)bits,                     /* 11 12路bit图 */
    //        (int)(gyro_z * 10.0f));             /* 12 陀螺仪 ×10 */
}
}
