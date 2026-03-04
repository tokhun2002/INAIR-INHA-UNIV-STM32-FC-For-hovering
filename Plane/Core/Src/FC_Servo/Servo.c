/*
 * Servo.c
 *
 * Created on: Mar 27, 2025
 * Author: leecurrent04
 * Email : leecurrent04@inha.edu
 * Modified by: DongHunLee
 * Modified on: fab 28, 2026 (Added Takeoff Latch / Air-Mode Logic)
 */

/* Includes ------------------------------------------------------------------*/
#include <FC_Servo/Servo_module.h>
#include <math.h>
#include <string.h>
#include "FC_AHRS/AHRS.h"

/* Variables -----------------------------------------------------------------*/
/*
 * 0x[타이머][채널]
 * 타이머 : 1,3-5
 * 채널 : 1-4
 */
const uint8_t SERVO_TIMER_MAP[SERVO_CHANNEL_MAX] = {
        0x42, 0x31, 0x32, 0x44,
        0x51, 0x52, 0x53, 0x54,
        0x33, 0x34, 0x12, 0x14
};

/* -------------------------------------------------------------------------- */
/* Rate control configuration (필요시 여기만 튜닝/이식)                         */
/* -------------------------------------------------------------------------- */

#define TRIM_SAMPLES  1000

/* RC 스틱 -> 목표 각속도(rad/s) */
#define MAX_ROLL_RATE_RAD_S   (4.0f)
#define MAX_PITCH_RATE_RAD_S  (4.0f)
#define MAX_YAW_RATE_RAD_S    (1.0f)   /* yaw는 보통 roll/pitch보다 낮게 */

/* -------------------------------------------------------------------------- */
/* Level(Angle) outer-loop configuration                                      */
/* -------------------------------------------------------------------------- */
#define SERVO_LEVEL_ENABLE        (1)
#define SERVO_ATTITUDE_IN_DEG     (0)
#define DEG2RAD_F                (0.01745329251994329577f)
#define MAX_ANGLE_DEG            (10.0f)
#define MAX_ANGLE_RAD            (MAX_ANGLE_DEG * DEG2RAD_F)
#define ANGLE_KP_RATE_PER_RAD    (4.0f)

#define STICK_DEADBAND_US     (10)     /* 1500±10us는 0으로 */

/* Takeoff latch */
#define THR_LANDED_RESET_US       (1050)
#define THR_TAKEOFF_TRIGGER_US    (1350)

/* 기본 PID 제한 (roll/pitch용) */
#define AXIS_OUT_LIMIT_US     (500.0f)
#define I_LIMIT_US            (400.0f)

/* D-term LPF */
#define D_CUTOFF_HZ           (10.0f)

/* -------------------------------------------------------------------------- */
/*  Yaw 전용 구조: yaw I만 분리 + yaw bias(느린 평균토크 보정)                */
/* -------------------------------------------------------------------------- */
#define YAW_BIAS_ENABLE              (1)

/* yaw 출력/적분 제한: roll/pitch보다 확실히 작게 */
#define YAW_OUT_LIMIT_US             (220.0f)   /* 150~280 추천 */
#define YAW_I_LIMIT_US               (120.0f)   /*  60~160 추천 */

/* yaw bias: "기체가 가만히 있어도 도는" 평균토크를 천천히 상쇄 */
#define YAW_BIAS_KI_US_PER_RAD       (8.0f)     /* 4~12 추천 */
#define YAW_BIAS_LIMIT_US            (180.0f)   /* 120~220 추천 */
#define YAW_BIAS_MIN_THR_US          (1250)
#define YAW_BIAS_MAX_ABS_RATE_RAD_S  (1.5f)

/* saturation 시 감쇠 */
#define YAW_I_DECAY_ON_SAT           (0.98f)
#define YAW_BIAS_DECAY_ON_SAT        (0.99f)

/* 초기 PID 게인 */
#define KP_ROLL_US_PER_RAD_S   (110.0f)
#define KI_ROLL_US_PER_RAD_S   (50.0f)
#define KD_ROLL_US_PER_RAD_S   (0.1f)

#define KP_PITCH_US_PER_RAD_S  (110.0f)
#define KI_PITCH_US_PER_RAD_S  (50.0f)
#define KD_PITCH_US_PER_RAD_S  (0.1f)

#define KP_YAW_US_PER_RAD_S    (60.0f)
#define KI_YAW_US_PER_RAD_S    (10.0f)
#define KD_YAW_US_PER_RAD_S    (0.0f)

/* ms 기반 s_last_ms 대신 us 기반으로 */

typedef struct {
    float kp, ki, kd;

    float i_term;
    float i_limit;

    float prev_meas;
    float d_lpf;
    float d_alpha;

    float out_limit;
} pid_t;

static pid_t pid_roll, pid_pitch, pid_yaw;
static uint8_t s_armed = 0;

/* 비행 상태 래치 변수 (0: 땅, 1: 공중) */
static uint8_t s_is_airborne = 0;

/* trim */
static float s_roll_trim = 0.0f;
static float s_pitch_trim = 0.0f;
static uint8_t s_trim_valid = 0;

/* yaw bias 상태 */
static float s_yaw_bias_us = 0.0f;

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float apply_deadband_us(int16_t x_us, int16_t db_us)
{
    if (x_us > db_us) return (float)(x_us - db_us);
    if (x_us < -db_us) return (float)(x_us + db_us);
    return 0.0f;
}

static float lpf_alpha(float fc_hz, float dt_s)
{
    float a = expf(-2.0f * (float)M_PI * fc_hz * dt_s);
    return clampf(a, 0.0f, 1.0f);
}

static void pid_init(pid_t *pid, float kp, float ki, float kd, float i_limit, float out_limit)
{
    memset(pid, 0, sizeof(*pid));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_limit = i_limit;
    pid->out_limit = out_limit;
    pid->d_alpha = 0.0f;
}

static void pid_reset(pid_t *pid)
{
    pid->i_term = 0.0f;
    pid->prev_meas = 0.0f;
    pid->d_lpf = 0.0f;
}

static float pid_update_rate(pid_t *pid, float sp_rad_s, float meas_rad_s, float dt_s)
{
    float err = sp_rad_s - meas_rad_s;

    float p = pid->kp * err;

    pid->i_term += pid->ki * err * dt_s;
    pid->i_term = clampf(pid->i_term, -pid->i_limit, pid->i_limit);

    float d_meas = (meas_rad_s - pid->prev_meas) / dt_s;
    pid->prev_meas = meas_rad_s;

    float d_raw = -pid->kd * d_meas;
    pid->d_lpf = pid->d_alpha * pid->d_lpf + (1.0f - pid->d_alpha) * d_raw;

    float out = p + pid->i_term + pid->d_lpf;
    out = clampf(out, -pid->out_limit, pid->out_limit);
    return out;
}

static void rate_controller_reset_all(void)
{
    pid_reset(&pid_roll);
    pid_reset(&pid_pitch);
    pid_reset(&pid_yaw);
    s_yaw_bias_us = 0.0f;
}

/* Functions -----------------------------------------------------------------*/
int SERVO_Initialization(void)
{
    LL_TIM_EnableCounter(TIM1);
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_EnableCounter(TIM4);
    LL_TIM_EnableCounter(TIM5);

    if(!(LL_TIM_IsEnabledCounter(TIM1) &&
            LL_TIM_IsEnabledCounter(TIM3) &&
            LL_TIM_IsEnabledCounter(TIM4) &&
            LL_TIM_IsEnabledCounter(TIM5)
            )) return -1;

    /* Rate PID init */
    pid_init(&pid_roll,  KP_ROLL_US_PER_RAD_S,  KI_ROLL_US_PER_RAD_S,  KD_ROLL_US_PER_RAD_S,  I_LIMIT_US,     AXIS_OUT_LIMIT_US);
    pid_init(&pid_pitch, KP_PITCH_US_PER_RAD_S, KI_PITCH_US_PER_RAD_S, KD_PITCH_US_PER_RAD_S, I_LIMIT_US,     AXIS_OUT_LIMIT_US);

    /*  yaw만 i_limit/out_limit 분리 */
    pid_init(&pid_yaw,   KP_YAW_US_PER_RAD_S,   KI_YAW_US_PER_RAD_S,   KD_YAW_US_PER_RAD_S,   YAW_I_LIMIT_US, YAW_OUT_LIMIT_US);

    rate_controller_reset_all();

    s_is_airborne = 0;

    SERVO_doDisarm();
    return 0;
}

void SERVO_doArm(void)
{
    configurePWM(param.servo.RATE);

    for (uint8_t i=0; i<SERVO_CHANNEL_MAX; i++) {
        if(!((param.servo.GPIO_MASK >> i)&0x1)) continue;
        doArm2Channel(i+1, 1);
    }

    s_armed = 1;

    float sum_r = 0.0f, sum_p = 0.0f;
    for (int k=0; k<TRIM_SAMPLES; k++) {
        AHRS_GetData();
        sum_r += msg.attitude.roll;
        sum_p += msg.attitude.pitch;
        LL_mDelay(1);
    }
    s_roll_trim  = sum_r / TRIM_SAMPLES;
    s_pitch_trim = sum_p / TRIM_SAMPLES;

    /* 짐벌 테스트중에는 0 */
    s_trim_valid = 0;

    s_is_airborne = 0;
    rate_controller_reset_all();
}

void SERVO_doDisarm(void)
{
    configurePWM(param.servo.RATE);

    for(uint8_t i=0; i<SERVO_CHANNEL_MAX; i++)
    {
        doArm2Channel(i+1, 0);
    }

    s_armed = 0;
    s_trim_valid = 0;
    s_is_airborne = 0;
    rate_controller_reset_all();
    return;
}

void SERVO_control(void)
{
    if (!msg.timing.imu_new_sample) {
        return;
    }
    calculateServoOutput();
    setPWM();
    return;
}

void SERVO_setFailsafe(void)
{
    setPWM2Channel(1, 1000);
    setPWM2Channel(2, 1500);
    setPWM2Channel(3, 1500);
    setPWM2Channel(4, 1500);
    return;
}

/* Functions -----------------------------------------------------------------*/
/*
 * @brief 특정 채널 출력 모드 설정
 * @parm uint8_t servoCh (in 1-12)
 * @parm uint8_t state
 * 0 : disable
 * 1 : enable
 * @retval 0 : 설정됨
 * @retval -1 : ch 범위 오류
 */
int doArm2Channel(uint8_t servoCh, uint8_t state)
{
    if(servoCh<1 || servoCh>12) return -1;
    const TIM_TypeDef* timerArr[] = {
            0, TIM1, 0, TIM3, TIM4, TIM5
    };

    TIM_TypeDef* timer = (TIM_TypeDef*)timerArr[SERVO_TIMER_MAP[servoCh-1]>>4];

    uint32_t ch;
    switch(SERVO_TIMER_MAP[servoCh-1]&0x0F){
    case 1: ch = LL_TIM_CHANNEL_CH1; break;
    case 2: ch = LL_TIM_CHANNEL_CH2; break;
    case 3: ch = LL_TIM_CHANNEL_CH3; break;
    case 4: ch = LL_TIM_CHANNEL_CH4; break;
    default: ch = 0; break;
    }

    if(timer&&ch){
        if(state == 1) LL_TIM_CC_EnableChannel(timer, ch);
        else if(state == 0) LL_TIM_CC_DisableChannel(timer, ch);
    }

    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_GenerateEvent_UPDATE(TIM3);
    LL_TIM_GenerateEvent_UPDATE(TIM4);
    LL_TIM_GenerateEvent_UPDATE(TIM5);

    return 0;
}

int doArm2Channels(uint8_t *pCh, uint8_t len, uint8_t state)
{
    for(uint8_t i=0; i<len; i++)
    {
        uint8_t ch = pCh[i];
        doArm2Channel(ch, state);
    }
    return 0;
}

/*
 * @brief PWM 주기 설정
 * @detail 50-490Hz까지 변경
 * @parm uint16_t hz (in 50-490)
 * @retval 0 : 설정됨
 * @retval 1 : 주파수 범위 오설정
 */
//volatile uint8_t testhz = 0;
//volatile uint32_t testARR = 0;
uint8_t configurePWM(uint16_t hz)
{
    if(hz>490 || hz<50) return 1;
    //testhz = hz;

    LL_TIM_SetAutoReload(TIM1, 1000000/hz-1);
    LL_TIM_SetAutoReload(TIM3, 1000000/hz-1);
    LL_TIM_SetAutoReload(TIM4, 1000000/hz-1);
    LL_TIM_SetAutoReload(TIM5, 1000000/hz-1);

    LL_TIM_SetPrescaler(TIM1, 168-1);
    LL_TIM_SetPrescaler(TIM3, 84-1);
    LL_TIM_SetPrescaler(TIM4, 84-1);
    LL_TIM_SetPrescaler(TIM5, 84-1);

    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_GenerateEvent_UPDATE(TIM3);
    LL_TIM_GenerateEvent_UPDATE(TIM4);
    LL_TIM_GenerateEvent_UPDATE(TIM5);
//    testARR = TIM3->ARR;
    return 0;
}

/*
 * @brief 출력할 값(CCR) 계산
 * @detail Rate(각속도) PID 제어 기반 쿼드 믹서
 * @parm none
 * @retval none
 */
/* TIM14: PSC=83 (84MHz 기준 1MHz = 1us tick), ARR=65535 free-run */
volatile uint16_t heartservo = 0;
volatile float sp_roll_test = 0.0f;
volatile float sp_pitch_test = 0.0f;

void calculateServoOutput(void)
{
    float dt_s = msg.timing.dt_imu_s;
    if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = 0.001f;

    float alpha = lpf_alpha(D_CUTOFF_HZ, dt_s);
    pid_roll.d_alpha  = alpha;
    pid_pitch.d_alpha = alpha;
    pid_yaw.d_alpha   = alpha;

    uint16_t thr_us, pit_us, rol_us, yaw_us;

    if(param.rc.PROTOCOLS != 0)
    {
        thr_us = Servo_nomalize(msg.RC_channels.value[param.rc.map.THR]);
        pit_us = Servo_nomalize(msg.RC_channels.value[param.rc.map.PIT]);
        rol_us = Servo_nomalize(msg.RC_channels.value[param.rc.map.ROL]);
        yaw_us = Servo_nomalize(msg.RC_channels.value[param.rc.map.YAW]);
    }
    else
    {
        thr_us = 1000;
        pit_us = 1500;
        rol_us = 1500;
        yaw_us = 1500;
    }

    if (!s_armed)
    {
        msg.servo_output_raw.servo_raw[0] = 1000;
        msg.servo_output_raw.servo_raw[1] = 1000;
        msg.servo_output_raw.servo_raw[2] = 1000;
        msg.servo_output_raw.servo_raw[3] = 1000;
        s_is_airborne = 0;
        return;
    }

    /* takeoff latch */
    if (thr_us > THR_TAKEOFF_TRIGGER_US) {
        s_is_airborne = 1;
    }

    if (thr_us < THR_LANDED_RESET_US)
    {
        s_is_airborne = 0;
        rate_controller_reset_all();
    }
    else if (s_is_airborne == 0)
    {
        /* 지상(아직 미이륙): I/bias 금지 */
        pid_roll.i_term  = 0.0f;
        pid_pitch.i_term = 0.0f;
        pid_yaw.i_term   = 0.0f;
        s_yaw_bias_us    = 0.0f;
    }

    float rol_cmd = apply_deadband_us((int16_t)rol_us - 1500, STICK_DEADBAND_US);
    float pit_cmd = apply_deadband_us((int16_t)pit_us - 1500, STICK_DEADBAND_US);
    float yaw_cmd = apply_deadband_us((int16_t)yaw_us - 1500, STICK_DEADBAND_US);

#if SERVO_LEVEL_ENABLE
    #if SERVO_ATTITUDE_IN_DEG
        float roll_meas_rad  = msg.attitude.roll  * DEG2RAD_F;
        float pitch_meas_rad = msg.attitude.pitch * DEG2RAD_F;
    #else
        float roll_meas_rad  = msg.attitude.roll;
        float pitch_meas_rad = msg.attitude.pitch;
        if (s_trim_valid) {
            roll_meas_rad  -= s_roll_trim;
            pitch_meas_rad -= s_pitch_trim;
        }
    #endif

    float sp_roll_angle  = (rol_cmd / 500.0f) * MAX_ANGLE_RAD;
    float sp_pitch_angle = (pit_cmd / 500.0f) * MAX_ANGLE_RAD;

    float sp_roll  = clampf(ANGLE_KP_RATE_PER_RAD * (sp_roll_angle  - roll_meas_rad),
                            -MAX_ROLL_RATE_RAD_S,  MAX_ROLL_RATE_RAD_S);
    float sp_pitch = clampf(ANGLE_KP_RATE_PER_RAD * (sp_pitch_angle - pitch_meas_rad),
                            -MAX_PITCH_RATE_RAD_S, MAX_PITCH_RATE_RAD_S);

    float sp_yaw   = (yaw_cmd / 500.0f) * MAX_YAW_RATE_RAD_S;
#else
    float sp_roll  = (rol_cmd / 500.0f) * MAX_ROLL_RATE_RAD_S;
    float sp_pitch = (pit_cmd / 500.0f) * MAX_PITCH_RATE_RAD_S;
    float sp_yaw   = (yaw_cmd / 500.0f) * MAX_YAW_RATE_RAD_S;
#endif


    float meas_roll  = msg.IMU_filt.gyro_x;
    float meas_pitch = msg.IMU_filt.gyro_y;
    float meas_yaw   = msg.IMU_filt.gyro_z;

    float u_roll  = pid_update_rate(&pid_roll,  sp_roll,  meas_roll,  dt_s);
    float u_pitch = pid_update_rate(&pid_pitch, sp_pitch, meas_pitch, dt_s);

    /* yaw = PID(빠른 잔오차) + bias(느린 평균토크) */
    float u_yaw = pid_update_rate(&pid_yaw, sp_yaw, meas_yaw, dt_s);

#if YAW_BIAS_ENABLE
    uint8_t bias_ok =
        (s_is_airborne == 1) &&
        (thr_us >= YAW_BIAS_MIN_THR_US) &&
        (fabsf(yaw_cmd) < (float)STICK_DEADBAND_US) &&
        (fabsf(meas_yaw) < YAW_BIAS_MAX_ABS_RATE_RAD_S);

    if (bias_ok) {
        /* meas_yaw -> 0으로 보내는 평균 토크를 천천히 학습 */
        s_yaw_bias_us += (-YAW_BIAS_KI_US_PER_RAD) * meas_yaw * dt_s;
        s_yaw_bias_us = clampf(s_yaw_bias_us, -YAW_BIAS_LIMIT_US, YAW_BIAS_LIMIT_US);
    }

    u_yaw += s_yaw_bias_us;
    u_yaw = clampf(u_yaw, -YAW_OUT_LIMIT_US, YAW_OUT_LIMIT_US);
#endif

    float base = 1000.0f + (float)(thr_us - 1000);

    float m0 = base - u_pitch - u_roll + u_yaw;
    float m1 = base + u_pitch + u_roll + u_yaw;
    float m2 = base - u_pitch + u_roll - u_yaw;
    float m3 = base + u_pitch - u_roll - u_yaw;

    /* saturation check (desat 전에 감지) */
    float maxv = m0, minv = m0;
    if (m1 > maxv) maxv = m1;
    if (m1 < minv) minv = m1;
    if (m2 > maxv) maxv = m2;
    if (m2 < minv) minv = m2;
    if (m3 > maxv) maxv = m3;
    if (m3 < minv) minv = m3;

    /* ✅ 싸움 방지: 포화면 yaw I/bias를 조금씩 죽여서 RP 권한을 보호 */
    if ((maxv > 2000.0f) || (minv < 1000.0f)) {
        pid_yaw.i_term *= YAW_I_DECAY_ON_SAT;
#if YAW_BIAS_ENABLE
        s_yaw_bias_us  *= YAW_BIAS_DECAY_ON_SAT;
#endif
    }

    /* Desaturation: 전체 shift로 [1000,2000] 유지 */
    if (maxv > 2000.0f) {
        float shift = maxv - 2000.0f;
        m0 -= shift; m1 -= shift; m2 -= shift; m3 -= shift;
    }
    if (minv < 1000.0f) {
        float shift = 1000.0f - minv;
        m0 += shift; m1 += shift; m2 += shift; m3 += shift;
    }

    msg.servo_output_raw.servo_raw[0] = Servo_nomalize((uint16_t)clampf(m0, 1000.0f, 2000.0f));
    msg.servo_output_raw.servo_raw[1] = Servo_nomalize((uint16_t)clampf(m1, 1000.0f, 2000.0f));
    msg.servo_output_raw.servo_raw[2] = Servo_nomalize((uint16_t)clampf(m2, 1000.0f, 2000.0f));
    msg.servo_output_raw.servo_raw[3] = Servo_nomalize((uint16_t)clampf(m3, 1000.0f, 2000.0f));
}


/*
 * @brief standard PWM 출력
 * @detail
 * @parm none
 * @retval none
 */
void setPWM(void)
{
    setPWM2Channel(1, msg.servo_output_raw.servo_raw[0]);
    setPWM2Channel(2, msg.servo_output_raw.servo_raw[1]);
    setPWM2Channel(3, msg.servo_output_raw.servo_raw[2]);
    setPWM2Channel(4, msg.servo_output_raw.servo_raw[3]);
}

/*
 * @brief 특정 서보 채널에 standard PWM 출력
 * @parm uint8_t ch (in 1-12)
 * @parm uint16_t value (in 1000-2000)
 * @retval 0
 */
int setPWM2Channel(uint8_t ch, uint16_t value)
{
    if(ch<1 || ch>12) return -1;
    if(value<800||value>2000) return -2;

    const TIM_TypeDef* timerArr[] = {
            0, TIM1, 0, TIM3, TIM4, TIM5
    };

    TIM_TypeDef* timer = (TIM_TypeDef*)timerArr[SERVO_TIMER_MAP[ch-1]>>4];

    switch(SERVO_TIMER_MAP[ch-1]&0x0F){
    case 1: timer->CCR1 = value; break;
    case 2: timer->CCR2 = value; break;
    case 3: timer->CCR3 = value; break;
    case 4: timer->CCR4 = value; break;
    }

    return 0;
}

/*
 * @brief 특정 서보 채널 그룹에 standard PWM 출력
 * @parm uint8_t* pCh (each in 1-12)
 * @parm uint8_t len : array size
 * @parm uint16_t value (in 1000-2000)
 * @retval 0
 */
int setPWM2Channels(uint8_t *pCh, uint8_t len, uint16_t value)
{
    for(uint8_t i=0; i<len; i++)
    {
        uint8_t ch = pCh[i];
        setPWM2Channel(ch, value);
    }
    return 0;
}

uint16_t Servo_nomalize(const uint16_t val)
{
    uint16_t temp = val;
    temp = temp>2000?2000:temp;
    temp = temp<1000?1000:temp;

    return temp;
}
