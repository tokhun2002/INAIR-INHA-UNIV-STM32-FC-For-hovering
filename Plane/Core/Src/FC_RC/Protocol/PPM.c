/*
 * PPM.c
 *
 *  Created on: Apr 5, 2025
 *      Author: leecurrent04
 *      Email : leecurrent04@inha.edu
 *
 *  Modified by: DongHunLee
 *  Modified on: Jan 2, 2026
 */


/* Includes ------------------------------------------------------------------*/
#include <FC_RC/Protocol/PPM.h>
#include "main.h"
#include "stm32f4xx_ll_tim.h"


/* PPM Decoder State ---------------------------------------------------------*/
/*
 * 설계 의도:
 * - PPM 단독 운용을 전제로, EXTI(엣지) ISR에서 바로 PPM_readData()를 호출하도록 구성
 * - RC_receiveIRQ2 같은 범용 IRQ 디스패처를 거치지 않음
 * - 프레임 경계(>2500us sync gap)에서만 SET_FL_RX()를 발생시켜 완성된 프레임만 메인 루프가 읽도록 함
 *
 * 전제:
 * - TIM14 CNT를 1MHz(=1us tick)로 구동하고, EXTI 엣지에서 CNT를 읽어 dt(us)를 계산
 */

static uint16_t ppm_prev_cnt = 0;
static uint8_t  ppm_cnt = 0;
static uint16_t ppm_work[PPM_MAX_CHANNEL]; // 임시저장소. 여기가 완전히 다 차야지만 buffer로 넘김.


/* Functions 1 ---------------------------------------------------------------*/
/*
 * @brief PPM 입력 설정
 * @detail RC_Initialization()에서 실행됨
 * @retval 0 : 정상
 */
int PPM_init(void)
{
	ppm_prev_cnt = (uint16_t)LL_TIM_GetCounter(TIM14);
	ppm_cnt = 0;

	for (int i = 0; i < PPM_MAX_CHANNEL; i++) {
		ppm_work[i] = 1500;
	}

	// 아직 완성된 프레임이 없으므로 RX flag는 clear 상태로 시작
	CLEAR_FL_RX();

	return 0;
}


/*
 * @brief PPM ISR
 * @detail EXTI 하강엣지마다 호출 (pull-up 신호 기준)
 *         system_time(time_unix_usec) 기반으로 펄스 폭(us)을 계산
 *
 * @parm data : (미사용) 기존 인터페이스 호환 목적
 *
 * @retval 0  : 채널 1개 수신
 * @retval 1  : Sync gap 감지(새 프레임 시작)
 * @retval -1 : 이상 데이터
 * @retval -2 : 버퍼 미설정
 */
volatile uint32_t testdt = 0;
volatile uint8_t testbuffer = 5;
volatile uint32_t testnow_cnt = 0;
int PPM_readData(uint16_t data)
{
	(void)data;

	if(RC_isBufferInit() != 0) {
		// RadioControl.c에서 RC_Buffer를 정적으로 잡아줘야 함
		// testbuffer = RC_isBufferInit();
		return -2;
	}

	uint16_t now_cnt = (uint16_t)LL_TIM_GetCounter(TIM14);
	//testnow_cnt = now_cnt;
	uint16_t dt_cnt  = (uint16_t)(now_cnt - ppm_prev_cnt);
	ppm_prev_cnt = now_cnt;
	uint32_t dt = (uint32_t)dt_cnt;

	// 프레임 동기 구간 (sync gap)
	if(dt > 2500U)
	{
		// 직전 프레임이 완성된 경우에만 커밋 (중간 프레임은 폐기)
		if(ppm_cnt >= PPM_MAX_CHANNEL)
		{
			for (int i = 0; i < PPM_MAX_CHANNEL; i++) {
				((uint16_t*)RC_Buffer)[i] = ppm_work[i];
			}
			SET_FL_RX(); //이 조건이 전부 다 성립해야만 flag 올리는거임.
		}

		ppm_cnt = 0;
		return 1;
	}

	// 채널 폭 범위 체크 (일반적으로 1000~2000us)
	if(dt > 2200U || dt < 800U) {
		return -1;
	}

	testdt = dt;
	// 채널 저장
	if(ppm_cnt < PPM_MAX_CHANNEL) {
		ppm_work[ppm_cnt++] = (uint16_t)dt;

	}

	return 0;
}


/*
 * @brief 조종 데이터 로딩
 * @detail RC_GetData()에서 실행됨
 * @retval 0   : 정상 수신 (완성된 프레임)
 * @retval -1  : 아직 완성된 프레임 없음
 * @retval -2  : 수신 버퍼가 설정되지 않음
 * @retval 0xf2: FailSafe (미사용, 호환용)
 */
volatile int testISFLAG = 0;
volatile uint16_t heartppm = 0;
volatile uint16_t testValueppm = 0;
int PPM_getControlData(void)
{

	if (heartppm > 1000) heartppm = 0;

	testISFLAG = IS_FL_RX;
	if(IS_FL_RX == 0) return -1;
	if(RC_isBufferInit() != 0) return -2;

	// flag clear (이번 프레임은 소비 완료)
	CLEAR_FL_RX();
	heartppm++;
	for(uint8_t i=0; i<PPM_MAX_CHANNEL; i++){
		uint16_t value = ((uint16_t*)RC_Buffer)[i];
		RC_MSG_setChannelValue(value, i);
	}

	RC_MSG_setChannelInfo(PPM_MAX_CHANNEL, 100);

	return 0;
}
