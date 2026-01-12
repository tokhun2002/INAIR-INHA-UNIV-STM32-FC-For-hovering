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

static float yaw_imu = 0.0f;
static uint8_t yaw_imu_inited = 0;


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

int AHRS_GetData(void)
{
    // 1. 센서 데이터 갱신
    IMU_GetData();
    Baro_GetData();
    // MAG_GetData(); // Yaw 적분만 할거면 당장 필요 없음

    //DT 계산 - TIM14 (us 단위) -> 이건 잘 되던 거니 유지
    static uint16_t prev_cnt = 0;
    static uint8_t first_run = 1;
    float deadzone = 0.0003f;
    float deadzoneyaw = 0.0003f;

    uint16_t now_cnt = (uint16_t)LL_TIM_GetCounter(TIM14);

    if (first_run) {
        prev_cnt = now_cnt;
        first_run = 0;
        return 0;
    }

    uint16_t diff_cnt = (uint16_t)(now_cnt - prev_cnt);
    prev_cnt = now_cnt;
    float dt = (float)diff_cnt * 0.000001f;

    // 안전장치
    if (dt > 0.05f || dt <= 0.0f) dt = 0.001f;

    // -----------------------------------------------------------
    // [데이터 준비]
    // -----------------------------------------------------------
    SCALED_IMU* imu = &msg.scaled_imu;
    Vector3D acc;
    Vector3D gyro;

    // 가속도 (m/s^2)
    acc.x = imu->xacc * 0.001f * 9.81f;
    acc.y = imu->yacc * 0.001f * 9.81f;
    acc.z = imu->zacc * 0.001f * 9.81f;

    // 자이로 센서값 → 물리량 변환(m rad/s -> rad/s)

    gyro.x = (imu->xgyro * 0.001f) - gyro_off_x;
	gyro.y = (imu->ygyro * 0.001f) - gyro_off_y;
	gyro.z = (imu->zgyro * 0.001f) - gyro_off_z;


	if (fabs(gyro.z) < deadzoneyaw) gyro.z = 0.0f;
	if (fabs(gyro.x) < deadzone) gyro.x = 0.0f;
	if (fabs(gyro.y) < deadzone) gyro.y = 0.0f;

    // LPF 적용
    acc = LPF_update3D(&lpf_acc, &acc);
    gyro = LPF_update3D(&lpf_gyro, &gyro);

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
    float alpha = 0.98f; // 자이로 비중 (진동 심하면 0.99, 반응 느리면 0.95)

    // Roll 계산
    ahrs_roll = alpha * (ahrs_roll + gyro.x * dt) + (1.0f - alpha) * acc_roll;

    // Pitch 계산
    ahrs_pitch = alpha * (ahrs_pitch + gyro.y * dt) + (1.0f - alpha) * acc_pitch;

    // Yaw 계산 (가속도 보정 불가, 그냥 적분)
    ahrs_yaw += gyro.z * dt;

    // Yaw 범위 제한 (-PI ~ PI) - 선택사항
    if(ahrs_yaw > 3.141592f) ahrs_yaw -= 6.283185f;
    if(ahrs_yaw < -3.141592f) ahrs_yaw += 6.283185f;


    // -----------------------------------------------------------
    // [결과 저장]
    // -----------------------------------------------------------
    msg.attitude.time_boot_ms = msg.system_time.time_boot_ms;
    msg.attitude.roll  = ahrs_roll;
    msg.attitude.pitch = ahrs_pitch;
    msg.attitude.yaw   = ahrs_yaw;

    // 쿼터니언 값은 굳이 필요 없으면 0이나 변환값 채워넣음 (PID는 오일러 쓰니까 무관)
    // 필요하다면 AHRS_Euler2Quaternion(msg.attitude) 해서 넣으면 됨

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


void AHRS_computeVelocity(float dt)
{
	float ax = (float)msg.scaled_imu.xacc / 1000.0f * 9.81f;
	float ay = (float)msg.scaled_imu.yacc / 1000.0f * 9.81f;
	float az = (float)msg.scaled_imu.zacc / 1000.0f * 9.81f;

	float roll_rad = imu_roll * M_PI / 180.0f;
	float pitch_rad = imu_pitch * M_PI / 180.0f;

	// Body → World 회전행렬
	float R11 = cosf(pitch_rad);
	float R12 = sinf(roll_rad) * sinf(pitch_rad);
	float R13 = cosf(roll_rad) * sinf(pitch_rad);

	float R21 = 0;
	float R22 = cosf(roll_rad);
	float R23 = -sinf(roll_rad);

	float R31 = -sinf(pitch_rad);
	float R32 = sinf(roll_rad) * cosf(pitch_rad);
	float R33 = cosf(roll_rad) * cosf(pitch_rad);

	// 중력 벡터 (World frame)
	float gx_w = 0.0f;
	float gy_w = 0.0f;
	float gz_w = 9.81f;

	// 중력을 Body frame으로 변환: g_b = R^T * g_w
	float gx_b = R11 * gx_w + R21 * gy_w + R31 * gz_w;
	float gy_b = R12 * gx_w + R22 * gy_w + R32 * gz_w;
	float gz_b = R13 * gx_w + R23 * gy_w + R33 * gz_w;

	// 센서의 순수 가속도
	float ax_corrected = ax - gx_b;
	float ay_corrected = ay - gy_b;
	float az_corrected = az - gz_b;

	// 속도 적분
	imu_velocity.vx += ax_corrected * dt;
	imu_velocity.vy += ay_corrected * dt;
	imu_velocity.vz += az_corrected * dt;
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
