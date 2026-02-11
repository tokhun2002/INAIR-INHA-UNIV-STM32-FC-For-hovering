/*
 * AHRS.c
 *
 *  Created on: Jul 23, 2025
 *      Author: rlawn, leecurrent04
 *      Email : (rlawn)
 *      		leecurrent04@inha.edu (leecurrent04)
 *
 *  Modified by: DongHunLee
 *  Modified on: Jan 2, 2026
 */



/* Includes ------------------------------------------------------------------*/
#include <FC_AHRS/AHRS_module.h>

/* Variables -----------------------------------------------------------------*/
Kalman1D_t kal_vx, kal_vy;
IMU_Velocity_t kalman_velocity = {0.0f, 0.0f, 0.0f};
IMU_Velocity_t imu_velocity = {0.0f, 0.0f, 0.0f};

extern double gps_speed;
extern double gps_course;

float imu_roll = 0.0f;
float imu_pitch = 0.0f;
float pitch_offset = 0.0f;
float roll_offset = 0.0f;

const float DEG_TO_RAD = 0.0174533f;


/* Functions -----------------------------------------------------------------*/
/*
 * @brief IMU, MAG, Baro Initialization
 * @detail (MAG)LIS2MDF Sensor's default mode is 3-wire SPI.
 * @param
 * 		void
 * @retval 0
 */

static inline float wrap_pi(float a)
{
    while (a >  3.14159265358979f) a -= 2.0f * 3.14159265358979f;
    while (a < -3.14159265358979f) a += 2.0f * 3.14159265358979f;
    return a;
}
// imu용 yaw 추가 지자계 신뢰도 문제로 일단 드리프트 발생해도 imu만 쓸거임.
int AHRS_Initialization(void)
{
    uint8_t err = 0;

    LL_GPIO_SetOutputPin(GYRO1_NSS_GPIO_Port, GYRO1_NSS_Pin);
    LL_GPIO_SetOutputPin(GYRO2_NSS_GPIO_Port, GYRO2_NSS_Pin);
    LL_GPIO_SetOutputPin(MAG_NSS_GPIO_Port, MAG_NSS_Pin);
    LL_GPIO_SetOutputPin(BARO_NSS_GPIO_Port, BARO_NSS_Pin);

    err |= IMU_Initialization()<<0;
    err |= MAG_Initialization()<<2;
    err |= Baro_Initialization()<<4;

    LED_SetYellow(err); // 에러 있으면 노란불

    LPF_init();

    // [추가됨] 센서 초기화 후 자이로 0점 잡기 & Yaw 리셋
    // 드론을 평평한 곳에 두고 전원을 켜야 함
    AHRS_CalibrateOffset();

    return 0;
}

// [필수] 자이로 0점 오프셋 (AHRS_CalibrateOffset 미사용시 0.0f로 두면 됨)
float gyro_off_x = 0.0f;
float gyro_off_y = 0.0f;
float gyro_off_z = 0.0f;

// [필수] 자세 저장용 정적 변수 (이전 값을 기억해야 필터가 됨)
static float ahrs_roll = 0.0f;
static float ahrs_pitch = 0.0f;
static float ahrs_yaw = 0.0f;

volatile float g_vz_mps = 0.0f;     // 수직속도 (m/s)  +면 상승
volatile float g_az_mps2 = 0.0f;    // 선형 수직가속도 (m/s^2) 디버그용


int AHRS_GetData(void)
{

    // 1. 센서 데이터 갱신
    IMU_GetData();
    Baro_GetData();
    // MAG_GetData(); // Yaw 적분만 할거면 당장 필요 없음
    // IMU 샘플 기반 dt 사용 단위: s
    if (!msg.timing.imu_new_sample) {
    		return 0;
	}
    float dt = msg.timing.dt_imu_s;
    //안전장치
    if (dt > 0.002f || dt <= 0.0005f) dt = 0.001f;


    float deadzone = 0.001f;
    float deadzoneyaw = 0.001f;


    // -----------------------------------------------------------
    // [데이터 준비]
    // -----------------------------------------------------------
    SCALED_IMU* imu = &msg.scaled_imu;
    Vector3D acc;
    Vector3D gyro;
    // mg 가 현상태.
    // 가속도 (m/s^2)로 변환
    acc.x = imu->xacc * 0.001f * 9.81f;
    acc.y = imu->yacc * 0.001f * 9.81f;
    acc.z = imu->zacc * 0.001f * 9.81f;

    // 자이로 센서값 → 물리량 변환(m rad/s -> rad/s)
    gyro.x = (imu->xgyro * 0.001f) - gyro_off_x;
    gyro.y = (imu->ygyro * 0.001f) - gyro_off_y;
    gyro.z = (imu->zgyro * 0.001f) - gyro_off_z;

    if (fabs(gyro.z) < deadzoneyaw) gyro.z = 0.0f;
    if (fabs(gyro.x) < deadzone)    gyro.x = 0.0f;
    if (fabs(gyro.y) < deadzone)    gyro.y = 0.0f;

    // LPF 적용
    acc  = LPF_update3D(&lpf_acc, &acc);
    gyro = LPF_update3D(&lpf_gyro, &gyro);

    // 필터링 된 값 저장
    // 다시
    msg.scaled_imu2.xgyro = (int16_t)lrintf(gyro.x * 1000.0f);
    msg.scaled_imu2.ygyro = (int16_t)lrintf(gyro.y * 1000.0f);
    msg.scaled_imu2.zgyro = (int16_t)lrintf(gyro.z * 1000.0f);
    msg.scaled_imu2.xacc  = (int16_t)lrintf(acc.x * (1000.0f / 9.81f));
    msg.scaled_imu2.yacc  = (int16_t)lrintf(acc.y * (1000.0f / 9.81f));
    msg.scaled_imu2.zacc  = (int16_t)lrintf(acc.z * (1000.0f / 9.81f));

    //이건 서보나 다른곳에서 계산용 rad/s
    msg.IMU_filt.gyro_x = gyro.x;
    msg.IMU_filt.gyro_y = gyro.y;
    msg.IMU_filt.gyro_z = gyro.z;
    msg.IMU_filt.acc_x  = acc.x;
    msg.IMU_filt.acc_y  = acc.y;
    msg.IMU_filt.acc_z  = acc.z;

    // -----------------------------------------------------------
    // [1단계] 가속도 기반 각도 계산 (절대 기준)
    // -----------------------------------------------------------
    float acc_roll  = atan2(acc.y, acc.z);
    float acc_pitch = atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z));
    // 참고: Acc Pitch 계산식은 atan2(-x, sqrt(y^2+z^2))가 일반적임.

    // -----------------------------------------------------------
    // [2단계] 상보필터 (Complementary Filter) - 이게 정석입니다
    // 공식: 각도 = 0.98 * (이전각도 + 자이로*dt) + 0.02 * (가속도각도)
    // 자이로의 빠릿함 + 가속도의 안정성 결합
    // -----------------------------------------------------------
    float alpha = 0.995f; // 자이로 비중 (진동 심하면 0.99, 반응 느리면 0.95)

    // Roll 계산
    ahrs_roll = alpha * (ahrs_roll + gyro.x * dt) + (1.0f - alpha) * acc_roll;

    // Pitch 계산
    ahrs_pitch = alpha * (ahrs_pitch + gyro.y * dt) + (1.0f - alpha) * acc_pitch;

    // Yaw 계산 (가속도 보정 불가, 그냥 적분)
    ahrs_yaw += gyro.z * dt;

    // Yaw 범위 제한 (-PI ~ PI) - 선택사항
    if (ahrs_yaw > 3.141592f)  ahrs_yaw -= 6.283185f;
    if (ahrs_yaw < -3.141592f) ahrs_yaw += 6.283185f;

    // -----------------------------------------------------------
    // [결과 저장]
    // -----------------------------------------------------------
    msg.attitude.time_boot_ms = msg.system_time.time_boot_ms;
    msg.attitude.roll  = ahrs_roll;
    msg.attitude.pitch = ahrs_pitch;
    msg.attitude.yaw   = ahrs_yaw;
    msg.attitude.rollspeed = gyro.x;
    msg.attitude.pitchspeed = gyro.y;
    msg.attitude.yawspeed = gyro.z;

    return 0;
}

int AHRS_CalibrateOffset(void)
{
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;
    const int sample_count = 1000; // 1000개 샘플 수집

    // 1. 데이터 수집 (드론이 정지해 있어야 함)
    for(int i = 0; i < sample_count; i++)
    {
        IMU_GetData(); // 최신 데이터 읽기

        // GetData와 동일한 단위(scale)로 누적해야 함 (* 0.001f)
        sum_x += (msg.scaled_imu.xgyro * 0.001f);
        sum_y += (msg.scaled_imu.ygyro * 0.001f);
        sum_z += (msg.scaled_imu.zgyro * 0.001f);

        LL_mDelay(1); // 1ms 대기 (너무 빠른 루프 방지)
    }

    // 2. 평균값 계산 (이게 자이로의 Bias임)
    gyro_off_x = sum_x / (float)sample_count;
    gyro_off_y = sum_y / (float)sample_count;
    gyro_off_z = sum_z / (float)sample_count;

    // 3. [중요] Yaw 각도 리셋
    // 캘리브레이션이 끝나는 순간을 0도로 잡음
    ahrs_yaw = 0.0f;

    // (선택사항) Roll, Pitch는 가속도 기준으로 즉시 초기화해주는 게 좋음
    // 이렇게 하면 시작하자마자 각도가 천천히 따라가는 현상을 막을 수 있음
    SCALED_IMU* imu = &msg.scaled_imu;
    float acc_x = imu->xacc * 0.001f * 9.81f;
    float acc_y = imu->yacc * 0.001f * 9.81f;
    float acc_z = imu->zacc * 0.001f * 9.81f;

    ahrs_roll  = atan2(acc_y, acc_z);
    ahrs_pitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z));

    return 0;
}

volatile float testVz = 0.0f;

void AHRS_computeVelocity(float dt)
{
    // dt 안전장치 (dt는 "초" 단위)
    if (dt <= 0.0f || dt > 0.05f) dt = 0.001f;

    // 1) 필터된 가속도 (m/s^2)
    float ax = msg.IMU_filt.acc_x;
    float ay = msg.IMU_filt.acc_y;
    float az = msg.IMU_filt.acc_z;

    // 2) 자세각 (rad)
    float roll  = msg.attitude.roll;
    float pitch = msg.attitude.pitch;

    const float GRAVITY = 9.81f;

    // 3) 중력의 Body-frame 성분 (Yaw 무시 가정)
    float gx_b = -GRAVITY * sinf(pitch);
    float gy_b =  GRAVITY * sinf(roll) * cosf(pitch);
    float gz_b =  GRAVITY * cosf(roll) * cosf(pitch);

    // 4) 선형가속도
    float ax_lin = ax - gx_b;
    float ay_lin = ay - gy_b;
    float az_lin = az - gz_b;

    // ---- 여기부터 "Vz 안정화" 핵심 ----
    // (A) dt 누적 + 가속도 누적 후 200Hz로만 적분
    static float dt_accum = 0.0f;
    static float ax_accum = 0.0f, ay_accum = 0.0f, az_accum = 0.0f;
    static int   n_accum  = 0;

    dt_accum += dt;
    ax_accum += ax_lin;
    ay_accum += ay_lin;
    az_accum += az_lin;
    n_accum++;

    // 목표 업데이트 주기: 200Hz (5ms)
    const float DT_TARGET = 0.005f;
    if (dt_accum < DT_TARGET) return;

    // 평균 선형가속도
    float invN = 1.0f / (float)n_accum;
    float ax_mean = ax_accum * invN;
    float ay_mean = ay_accum * invN;
    float az_mean = az_accum * invN;

    // 누적 버퍼 리셋
    float dT = dt_accum;
    dt_accum = 0.0f;
    ax_accum = ay_accum = az_accum = 0.0f;
    n_accum = 0;

    // (B) 작은 진동 적분 억제(너무 크면 반응 죽음) - 필요시 튜닝
    const float acc_deadband = 0.05f;   // 0.05~0.20 사이에서 튜닝
    if (fabsf(az_mean) < acc_deadband) az_mean = 0.0f;

    // (C) leak를 dt에 맞게 자동 계산 (루프 주파수 무관)
    // tau_v: 속도가 0으로 돌아가는 시간상수(초). 0.5~2.0초 추천
    const float tau_v = 1.0f;
    float leak = expf(-dT / tau_v);

    imu_velocity.vx = leak * imu_velocity.vx + ax_mean * dT;
    imu_velocity.vy = leak * imu_velocity.vy + ay_mean * dT;
    imu_velocity.vz = leak * imu_velocity.vz + az_mean * dT;

    // 디버그용: km/h (m/s * 3.6)
    testVz = imu_velocity.vz * 3.6f;
}
/*
 * @brief : Hard/Soft-Iron 보정 + 기울기 보상 + yaw 계산
 * @detail : 가속도를 기반으로 roll, pitch 추정
 * @input : 지자계 LPF 보정값, 가속도 기반 roll pitch 계산값
 * @output : 가속도+지자계 기반 yaw 게산값
 */
float AHRS_calculateYAW(Vector3D mag, Euler angle)
{
	// Hard-Iron 보정 상수값
	static const float x_Hmax = 60.0f;
	static const float y_Hmax = 60.0f;
	static const float z_Hmax = 60.0f;
	static const float x_Hmin = -60.0f;
	static const float y_Hmin = -60.0f;
	static const float z_Hmin = -60.0f;

	// Hard-Iron
	static const float x_offset= (x_Hmax + x_Hmin) / (2.0);
	static const float y_offset= (y_Hmax + y_Hmin) / (2.0);
	static const float z_offset= (z_Hmax + z_Hmin) / (2.0);

	// Soft-Iron
	static const float x_scale = ((x_Hmax - x_Hmin)+(y_Hmax - y_Hmin)+(z_Hmax - z_Hmin)) / (3.0*((x_Hmax - x_Hmin)));
	static const float y_scale = ((x_Hmax - x_Hmin)+(y_Hmax - y_Hmin)+(z_Hmax - z_Hmin)) / (3.0*((y_Hmax - y_Hmin)));
	static const float z_scale = ((x_Hmax - x_Hmin)+(y_Hmax - y_Hmin)+(z_Hmax - z_Hmin)) / (3.0*((z_Hmax - z_Hmin)));

	// 보정
	mag.x = (mag.x - x_offset) * (x_scale);
	mag.y = (mag.y - y_offset) * (y_scale);
	mag.z = (mag.z - z_offset) * (z_scale);


	// 기울기 보상
    float Xh = mag.x * cos(angle.pitch) + mag.z * sin(angle.pitch);
    float Yh = mag.x * sin(angle.roll) * sin(angle.pitch) + mag.y * cos(angle.roll) - mag.z * sin(angle.roll) * cos(angle.pitch);


    // yaw값 계산
	float out = atan2(-Yh, Xh);

	// yaw값 출력
	return out;
}



/* Functions (common.h) ------------------------------------------------------*/
// 오일러 각(roll, pitch, yaw)을 쿼터니언(q0_mea, q1_mea, q2_mea, q3_mea)으로 변환
Quaternion AHRS_Euler2Quaternion(const Euler ori)
{
	Quaternion ret;

	Euler a;
	a.roll = ori.roll / 2;
	a.pitch = ori.pitch / 2;
	a.yaw = ori.yaw / 2;

    ret.q0 = cos(a.roll) * cos(a.pitch) * cos(a.yaw) + sin(a.roll) * sin(a.pitch) * sin(a.yaw);
    ret.q1 = sin(a.roll) * cos(a.pitch) * cos(a.yaw) - cos(a.roll) * sin(a.pitch) * sin(a.yaw);
    ret.q2 = cos(a.roll) * sin(a.pitch) * cos(a.yaw) + sin(a.roll) * cos(a.pitch) * sin(a.yaw);
    ret.q3 = cos(a.roll) * cos(a.pitch) * sin(a.yaw) - sin(a.roll) * sin(a.pitch) * cos(a.yaw);

    return ret;
}
// 쿼터니언을 오일러 각도로 변환해서 출력
Euler AHRS_Quaternion2Euler(const Quaternion ori)
{
	Euler ret;
#define POW(x) (x*x)
    ret.roll = RAD2DEG(atan2( 2.0f*(ori.q0 * ori.q1 + ori.q2*ori.q3), 1.0f - 2.0f*(POW(ori.q1) + POW(ori.q2)) ));
    ret.pitch = RAD2DEG(asin( 2.0f*(ori.q0 * ori.q2 - ori.q3*ori.q1) ));
    ret.yaw = RAD2DEG(atan2( 2.0f*(ori.q0*ori.q3 + ori.q1*ori.q2), 1.0f - 2.0f*(POW(ori.q2) + POW(ori.q3)) ));
#undef POW
    return ret;
}

// 쿼터니언 정규화
Quaternion AHRS_NormalizeQuaternion(const Quaternion ori)
{
	Quaternion ret;
#define POW(x) (x*x)
    float norm = sqrtf(POW(ori.q0)+POW(ori.q1)+POW(ori.q2)+POW(ori.q3));
#undef POW
    ret.q0 = ori.q0 / norm;
    ret.q1 = ori.q1 / norm;
    ret.q2 = ori.q2 / norm;
    ret.q3 = ori.q3 / norm;

    return ret;
}
