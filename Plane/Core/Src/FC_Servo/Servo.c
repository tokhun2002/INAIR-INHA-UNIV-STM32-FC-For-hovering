/*
 * Servo.c
 *
 * Created on: Mar 27, 2025
 * Author: leecurrent04
 * Email : leecurrent04@inha.edu
 * Modified by: DongHunLee
 * Modified on: Jan 10, 2026 (Added Takeoff Latch / Air-Mode Logic)
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
#define MAX_ROLL_RATE_RAD_S   (5.0f)   //이건 얼만큼 각속도가 튀게 할지 정하는거임. 낮게하면 안튀어서 부드럽긴한데,
#define MAX_PITCH_RATE_RAD_S  (5.0f)	// 이륙할때 충분한힘을 못받을수도있음. 이륙할때는 30도 이상 흔들릴수있음.
#define MAX_YAW_RATE_RAD_S    (2.0f)   /* yaw는 보통 roll/pitch보다 낮게 */

/* -------------------------------------------------------------------------- */
/* Level(Angle) outer-loop configuration                                      */
/* - SERVO_LEVEL_ENABLE=1: 스틱은 각도(roll/pitch), 내부는 rate PID로 구동      */
/* - 스로틀만 올릴 때 '수평 유지'를 원하면 이 모드가 필요함                     */
/* -------------------------------------------------------------------------- */
#define SERVO_LEVEL_ENABLE        (1)
/* msg.attitude.roll/pitch 단위가 deg면 1, rad면 0 */
#define SERVO_ATTITUDE_IN_DEG     (0)
#define DEG2RAD_F                (0.01745329251994329577f)
#define MAX_ANGLE_DEG            (15.0f)
#define MAX_ANGLE_RAD            (MAX_ANGLE_DEG * DEG2RAD_F)
/* 각도 오차(rad)를 목표 각속도(rad/s)로 바꾸는 P 게인 */
#define ANGLE_KP_RATE_PER_RAD    (10.0f) //이건 실험을 여러번해본결과 10이딱좋음.

#define STICK_DEADBAND_US     (10)     /* 1500±10us는 0으로 */

/* [수정됨] 이륙 판정 임계값 */
#define THR_LANDED_RESET_US   (1050)   /* 이 값 밑으로 내려야 착륙으로 간주 */
#define THR_TAKEOFF_TRIGGER_US (1350)  /* 이 값을 넘으면 "비행 중"으로 간주 (1600 이륙 고려하여 약간 낮게 설정) */

/* PID 출력(=모터 믹서에 더해지는 보정량)의 제한(us) */
#define AXIS_OUT_LIMIT_US     (350.0f) /* 기존 코드의 0.6*500=300과 스케일 호환 */
#define I_LIMIT_US            (150.0f)

/* D-term LPF */
#define D_CUTOFF_HZ           (30.0f)

/* 초기 PID 게인(반드시 튜닝 필요: 프레임/프로펠러/필터에 따라 달라짐) */

// D는 절대로 0.2 이상 올리면안됨. 0.1도 충분히 강함.
#define KP_ROLL_US_PER_RAD_S   (30.0f)
#define KI_ROLL_US_PER_RAD_S   (0.0f)
#define KD_ROLL_US_PER_RAD_S   (0.1f)

#define KP_PITCH_US_PER_RAD_S  (30.0f)
#define KI_PITCH_US_PER_RAD_S  (0.0f)
#define KD_PITCH_US_PER_RAD_S  (0.1f)

#define KP_YAW_US_PER_RAD_S    (30.0f)
#define KI_YAW_US_PER_RAD_S    (0.0f)
#define KD_YAW_US_PER_RAD_S    (0.1f)

/* ms 기반 s_last_ms 대신 us 기반으로 */
static uint16_t s_last_cnt_us = 0;


typedef struct {
    float kp, ki, kd;

    float i_term;
    float i_limit;

    float prev_meas;
    float d_lpf;
    float d_alpha;     /* 0~1, 1에 가까울수록 더 강한 smoothing */

    float out_limit;
} pid_t;

static pid_t pid_roll, pid_pitch, pid_yaw;
static uint8_t s_armed = 0;

/* [추가됨] 비행 상태 래치 변수 (0: 땅, 1: 공중) */
static uint8_t s_is_airborne = 0;

/* Level 모드에서 '현재 자세를 수평 기준'으로 잡기 위한 trim */
static float s_roll_trim = 0.0f;
static float s_pitch_trim = 0.0f;
static uint8_t s_trim_valid = 0;
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
    /* alpha = exp(-2*pi*fc*dt) */
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
    /* error = setpoint - measurement */
	// 오차계산
    float err = sp_rad_s - meas_rad_s;

    /* P */
    //오차만큼(비례항)
    float p = pid->kp * err;

    /* I (clamp) */
    //적분항(오차누적)
    pid->i_term += pid->ki * err * dt_s;
    pid->i_term = clampf(pid->i_term, -pid->i_limit, pid->i_limit);

    /* D on measurement: -kd * d(meas)/dt */
    //미분항 진동잡아줌.
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
}

/* Functions -----------------------------------------------------------------*/
/*
 * @brief SERVO 초기화
 * @detail 출력 프로토콜, 주기 변경
 * @parm none
 * @retval none
 */
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
    pid_init(&pid_roll,  KP_ROLL_US_PER_RAD_S,  KI_ROLL_US_PER_RAD_S,  KD_ROLL_US_PER_RAD_S,  I_LIMIT_US, AXIS_OUT_LIMIT_US);
    pid_init(&pid_pitch, KP_PITCH_US_PER_RAD_S, KI_PITCH_US_PER_RAD_S, KD_PITCH_US_PER_RAD_S, I_LIMIT_US, AXIS_OUT_LIMIT_US);
    pid_init(&pid_yaw,   KP_YAW_US_PER_RAD_S,   KI_YAW_US_PER_RAD_S,   KD_YAW_US_PER_RAD_S,   I_LIMIT_US, AXIS_OUT_LIMIT_US);
    rate_controller_reset_all();

    s_last_cnt_us = (uint16_t)LL_TIM_GetCounter(TIM14);
    s_is_airborne = 0; // 초기화 시 땅으로 간주

    SERVO_doDisarm();
    return 0;
}

/*
 * @brief 모든 채널 출력 활성화
 * @parm none
 * @retval none
 */

void SERVO_doArm(void)
{
    configurePWM(param.servo.RATE);

    for (uint8_t i=0; i<SERVO_CHANNEL_MAX; i++) {
        if(!((param.servo.GPIO_MASK >> i)&0x1)) continue;
        doArm2Channel(i+1, 1);
    }

    s_armed = 1;

    // --- trim average ---
    float sum_r = 0.0f, sum_p = 0.0f;
    for (int k=0; k<TRIM_SAMPLES; k++) {
        AHRS_GetData();                 // ★ 여기서 attitude를 갱신시킨다
        sum_r += msg.attitude.roll;
        sum_p += msg.attitude.pitch;
        LL_mDelay(1);                   // 1ms면 1000샘플=1초라 좀 김. 0.2~0.3초로 줄이려면 샘플 수를 줄여.
    }
    s_roll_trim  = sum_r / TRIM_SAMPLES;
    s_pitch_trim = sum_p / TRIM_SAMPLES;
    s_trim_valid = 1;

    s_last_cnt_us = (uint16_t)LL_TIM_GetCounter(TIM14);
    s_is_airborne = 0;
    rate_controller_reset_all();
}

/*
 * @brief 모든 출력 비활성화
 * @detail 출력 프로토콜 따라 수행
 * @parm none
 * @retval none
 */
void SERVO_doDisarm(void)
{
    configurePWM(param.servo.RATE);

    for(uint8_t i=0; i<SERVO_CHANNEL_MAX; i++)
    {
        doArm2Channel(i+1, 0);
    }

    s_armed = 0;
    s_trim_valid = 0;
    s_is_airborne = 0; // Disarm 되면 땅
    rate_controller_reset_all();
    return;
}

/*
 * @brief SERVO 출력
 * @detail 출력 프로토콜 따라 수행
 * @parm none
 * @retval none
 */
void SERVO_control(void)
{
	// 새 IMU 샘플 없으면: 제어/추정 스킵
	if (!msg.timing.imu_new_sample) {
		return;
	}
    calculateServoOutput();
    setPWM();
    return;
}

/*
 * @brief Fail-Safe 동작
 * @retval none
 */
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


void calculateServoOutput(void)
{
	float dt_s = msg.timing.dt_imu_s; // IMU 갱신값 기준 dt
	if (dt_s <= 0.0f || dt_s > 0.05f) dt_s = 0.001f;
    /* D LPF alpha 업데이트 */
    float alpha = lpf_alpha(D_CUTOFF_HZ, dt_s);
    pid_roll.d_alpha  = alpha;
    pid_pitch.d_alpha = alpha;
    pid_yaw.d_alpha   = alpha;

    /* 기본: 활성 채널은 RC 패스스루(기존 동작 유지) */
    for(uint8_t i=0; i<SERVO_CHANNEL_MAX; i++)
    {
        if(!((param.servo.GPIO_MASK >> i)&0x1)){
            continue;
        }
        msg.servo_output_raw.servo_raw[i] = Servo_nomalize(msg.RC_channels.value[i]);
    }

    /* Quad-Copter motor mixer: 채널 1~4만 rate 제어로 덮어씀 */
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
        /* RC가 없으면 안전하게 failsafe 성격 */
        thr_us = 1000;
        pit_us = 1500;
        rol_us = 1500;
        yaw_us = 1500;
    }

    /* Arm 안 됐으면: 모터는 최소로 */
    if (!s_armed)
    {
        msg.servo_output_raw.servo_raw[0] = 1000;
        msg.servo_output_raw.servo_raw[1] = 1000;
        msg.servo_output_raw.servo_raw[2] = 1000;
        msg.servo_output_raw.servo_raw[3] = 1000;
        s_is_airborne = 0; // Arm 안됐으면 땅
        return;
    }

    // -------------------------------------------------------------
    // [중요 수정] 이륙 판정 래치(Takeoff Latch) 및 I항 제어 로직
    // -------------------------------------------------------------

    // 1. 이륙 상태 판정 (스로틀이 1450us를 한 번이라도 넘으면 공중으로 간주)
    if (thr_us > THR_TAKEOFF_TRIGGER_US)
    {
        s_is_airborne = 1;
    }

    // 2. 착륙/초기화 판정 (스로틀을 바닥(1050us)까지 내려야 땅으로 간주)
    if (thr_us < THR_LANDED_RESET_US)
    {
        s_is_airborne = 0;
        rate_controller_reset_all(); // I항, D항 등 모든 PID 상태 초기화
    }
    // 3. 땅에 있지만 스로틀을 올리는 중 (1050 < Throttle < 1450)
    else if (s_is_airborne == 0)
    {
        // 아직 뜨지 않았으므로 I항(적분)이 쌓이면 안 됨 (Wind-up 방지)
        // 하지만 P항(기울기 제어)과 D항(진동 제어)은 작동해서 균형을 잡도록 함
        pid_roll.i_term = 0.0f;
        pid_pitch.i_term = 0.0f;
        pid_yaw.i_term = 0.0f;
    }
    // s_is_airborne == 1 이면? 아무 조치 안 함 -> I항 정상 작동 (이게 핵심)
    // -------------------------------------------------------------


    /* 스틱 입력(1500 기준) */
    float rol_cmd = apply_deadband_us((int16_t)rol_us - 1500, STICK_DEADBAND_US);
    float pit_cmd = apply_deadband_us((int16_t)pit_us - 1500, STICK_DEADBAND_US);
    float yaw_cmd = apply_deadband_us((int16_t)yaw_us - 1500, STICK_DEADBAND_US);

    // 만약에 yaw가 1500이면 무조건 yaw i는 끄는거임. 안전성을 위해서.
    if (fabsf(yaw_cmd) < (float)STICK_DEADBAND_US) {
            pid_yaw.i_term = 0.0f;      // 강하게: 항상 0
            // 더 부드럽게 하고 싶으면 아래처럼 감쇠 방식 사용
            // pid_yaw.i_term *= 0.98f;
        }

#if SERVO_LEVEL_ENABLE //매크로 사용
    #if SERVO_ATTITUDE_IN_DEG //먄약 degree값이 오면 라디안으로 바꿔주는것. 근데 라디안값으로 들어와서 상관 없음.
        float roll_meas_rad  = msg.attitude.roll  * DEG2RAD_F;
        float pitch_meas_rad = msg.attitude.pitch * DEG2RAD_F;
    #else
        //imu에서 온 현재 자세
        float roll_meas_rad  = msg.attitude.roll;
        float pitch_meas_rad = msg.attitude.pitch;

        if (s_trim_valid) {
            roll_meas_rad  -= s_roll_trim;
            pitch_meas_rad -= s_pitch_trim;
        }
    #endif
// 조종기로 들어온 목표자세
    float sp_roll_angle  = (rol_cmd / 500.0f) * MAX_ANGLE_RAD;
    float sp_pitch_angle = (pit_cmd / 500.0f) * MAX_ANGLE_RAD;
//오차 계산
    float sp_roll  = clampf(ANGLE_KP_RATE_PER_RAD * (sp_roll_angle  - roll_meas_rad),
                            -MAX_ROLL_RATE_RAD_S,  MAX_ROLL_RATE_RAD_S);
    float sp_pitch = clampf(ANGLE_KP_RATE_PER_RAD * (sp_pitch_angle - pitch_meas_rad),
                            -MAX_PITCH_RATE_RAD_S, MAX_PITCH_RATE_RAD_S);

    float sp_yaw   = (yaw_cmd / 500.0f) * MAX_YAW_RATE_RAD_S;

#else //제어기 비활성화. 계속해서 수평 유지.
    float sp_roll  = (rol_cmd / 500.0f) * MAX_ROLL_RATE_RAD_S;
    float sp_pitch = (pit_cmd / 500.0f) * MAX_PITCH_RATE_RAD_S;
    float sp_yaw   = (yaw_cmd / 500.0f) * MAX_YAW_RATE_RAD_S;
#endif


// 여기서부터 각속도 pid
    /* 측정 rate */
    float meas_roll  = msg.IMU_filt.gyro_x;  // rad/s (float)
    float meas_pitch = msg.IMU_filt.gyro_y;  // rad/s
    float meas_yaw   = msg.IMU_filt.gyro_z;  // rad/s

    /* Rate PID -> axis correction (us) */
    float u_roll  = pid_update_rate(&pid_roll,  sp_roll,  meas_roll,  dt_s);
    float u_pitch = pid_update_rate(&pid_pitch, sp_pitch, meas_pitch, dt_s);
    float u_yaw   = pid_update_rate(&pid_yaw,   sp_yaw,   meas_yaw,   dt_s);

    /* 베이스 스로틀 */
    float base = 1000.0f + (float)(thr_us - 1000);

    /* 믹서 */
    float m0 = base - u_pitch - u_roll + u_yaw;
    float m1 = base + u_pitch + u_roll + u_yaw;
    float m2 = base - u_pitch + u_roll - u_yaw;
    float m3 = base + u_pitch - u_roll - u_yaw;

    /* Desaturation: 전체 shift로 [1000,2000] 유지 */
    float maxv = m0, minv = m0;
    if (m1 > maxv) maxv = m1;
    if (m1 < minv) minv = m1;
    if (m2 > maxv) maxv = m2;
    if (m2 < minv) minv = m2;
    if (m3 > maxv) maxv = m3;
    if (m3 < minv) minv = m3;

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
    return;
}


/*
 * @brief standard PWM 출력
 * @detail
 * @parm none
 * @retval none
 */
void setPWM(void)
{
    for(uint8_t i=0; i<SERVO_CHANNEL_MAX; i++)
    {
        if(!((param.servo.GPIO_MASK >> i)&0x1)){
            continue;
        }
        setPWM2Channel(i+1, msg.servo_output_raw.servo_raw[i]);
    }
    return;
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
