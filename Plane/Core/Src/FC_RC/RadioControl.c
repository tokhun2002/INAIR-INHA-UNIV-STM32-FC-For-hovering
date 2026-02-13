/*
 * RadioControl.c
 * Radio 범용 라이브러리
 *
 * Modified on: Mar 13, 2026
 */

/* Includes ------------------------------------------------------------------*/
#include <FC_RC/RC_module.h>
#include <FC_RC/Protocol/PPM.h>
#include <FC_RC/Protocol/CRSF/CRSF.h> // [추가] CRSF 헤더 파일

/* Variables -----------------------------------------------------------------*/
volatile uint8_t rxFlag = 0;
uint8_t* RC_Buffer = 0; // 공용 포인터

// [핵심] 메모리 정적 할당 (동적 할당 X)
static uint16_t RC_PPM_Buffer[PPM_MAX_BUFFER_SIZE];
static uint8_t  RC_CRSF_Buffer[CRSF_MAX_BUFFER_SIZE]; // CRSF는 uint8_t 사용


/* Functions (driver) --------------------------------------------------------*/

/*
 * @brief RC 초기 설정
 * @detail RC 종류에 따라 메모리 포인터 연결 및 초기화
 * 쓰로틀 체크 및 ESC 캘리브레이션 수행
 */
volatile int testretVal = 0;
volatile int testflag = 0;

int RC_init(void) {
    LL_GPIO_SetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);

    // ---------------------------------------------------------
    // 1. 프로토콜 선택 및 버퍼 연결 (포인터 스위칭)
    // ---------------------------------------------------------
    if (param.rc.PROTOCOLS & (1 << RC_PROTOCOL_CRSF))
    {
        // [CRSF 모드]
        RC_Buffer = (uint8_t*)RC_CRSF_Buffer; // 포인터를 CRSF 버퍼로 연결

        // 버퍼 초기화
        for (int i = 0; i < CRSF_MAX_BUFFER_SIZE; i++) {
            RC_CRSF_Buffer[i] = 0;
        }

        // CRSF 연결 시작
        CRSF_connect();
    }
    else
    {
        // [PPM 모드 (기본값)]
        RC_Buffer = (uint8_t*)RC_PPM_Buffer; // 포인터를 PPM 버퍼로 연결

        // 버퍼 초기화 (PPM 중립값 1500)
        for (int i = 0; i < PPM_MAX_BUFFER_SIZE; i++) {
            RC_PPM_Buffer[i] = 1500;
        }

        // PPM 초기화
        PPM_init();
    }


    // ---------------------------------------------------------
    // 2. 수신 대기 (신호가 들어올 때까지)
    // ---------------------------------------------------------
    uint32_t t0 = msg.system_time.time_boot_ms;
    while (IS_FL_RX == 0) {
        if ((msg.system_time.time_boot_ms - t0) > 100) {
            break; // 100ms 지남
        }

        // CRSF일 경우 데이터를 받아오는 함수를 호출해줘야 할 수도 있음
        if (param.rc.PROTOCOLS & (1 << RC_PROTOCOL_CRSF)) {
             CRSF_getControlData();
        }
    }

    // ---------------------------------------------------------
    // 3. 스로틀 체크 및 ESC 캘리브레이션
    // ---------------------------------------------------------
    uint32_t previous_time = msg.system_time.time_boot_ms;

    while(1){
        // CRSF 사용 시 여기서도 데이터 갱신 필요
        if (param.rc.PROTOCOLS & (1 << RC_PROTOCOL_CRSF)) {
             CRSF_getControlData();
        }

        uint8_t flag_cali = (msg.system_time.time_boot_ms - previous_time > 5000);
        int retVal = RC_checkThrottle(); // 내부에서 RC_GetData 호출됨

        testflag = flag_cali;
        testretVal = retVal;

        if(-2 == retVal) {
             // 연결 끊김 처리 (CRSF 재연결 시도 등 필요 시 추가)
             return retVal;
        }

        // RC 정상 연결 + 스로틀 0% (안전) -> 루프 탈출
        if(0 == retVal) {
            BuzzerDisableThrottleHigh();
            if(RC_enterESCcalibration() == 0) break;
            else return -2;
        }

        /* 스로틀 High 상태 -> 경고음 */
        BuzzerEnableThrottleHigh();

        if(0 == flag_cali) continue;

        /* 5초 지나면 강제 캘리브레이션 진입 */
        BuzzerDisableThrottleHigh();
        if(RC_enterESCcalibration()==0) break;
    }

    LL_GPIO_ResetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);
    return 0;
}


int RC_Initialization(void)
{
	LL_GPIO_SetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);

    // 이 함수는 구버전 호환용으로 남겨둠 (필요 시 위와 같은 로직 적용)
	RC_Buffer = (uint8_t*)RC_PPM_Buffer;
	LL_GPIO_ResetOutputPin(LED_RED_GPIO_Port, LED_RED_Pin);

	return 0;
}


/*
 * @brief 조종 데이터 로딩
 * @detail 프로토콜에 따라 다르게 동작
 */
int RC_GetData(void)
{
    if (param.rc.PROTOCOLS & (1 << RC_PROTOCOL_CRSF))
    {
        int retVal = CRSF_getControlData(); // 헤더에 있는 함수 호출

        if(retVal < 0) return retVal; // 에러 처리

        return 0;
    }

    else
    {
        int retVal = PPM_getControlData();

        if(retVal == -1 || retVal == -2) return retVal;
        if(retVal!=0xf2) fsFlag = 0;

        return 0;
    }
}


/*
 * @brief 쓰로틀 체크
 */
volatile uint16_t complete = 0;
int RC_checkThrottle(void)
{
    for (int i = 0; i < 20; i++)  // 2s 대기
    {
        if (RC_GetData() == 0)    // [중요] 여기서 위에서 만든 RC_GetData 호출
            break;

        HAL_Delay(100);
    }

    uint32_t t0 = msg.system_time.time_boot_ms;
    while (IS_FL_RX == 0) {
        if ((msg.system_time.time_boot_ms - t0) > 100) {
            break;
        }
    }

    if (RC_GetData() != 0)
        return -2;

    complete = 1;

    // 스로틀 Low 체크
    if (msg.RC_channels.value[param.rc.map.THR] > 1050)
        return -1;

    complete = 2;
    return 0;
}


/* Functions -----------------------------------------------------------------*/

/*
 * @brief Buffer가 설정 되었는지 확인
 */
int RC_isBufferInit(void){
	if(RC_Buffer == 0) return -1;
	return 0;
}


/*
 * @brief ESC 캘리브레이션 진입
 */
int RC_enterESCcalibration(void)
{
    uint8_t channels[] = {1,2,3,4};

    configurePWM(50);
    doArm2Channels(channels, sizeof(channels), 1);

    setPWM2Channels(channels, sizeof(channels), 2000);
    HAL_Delay(3000);

    setPWM2Channels(channels, sizeof(channels), 1000);
    HAL_Delay(3000);

    doArm2Channels(channels, sizeof(channels), 0);

    return 0;
}

/*
 * @brief Failsafe 모드로 진입
 */
int RC_setFailsafe(uint16_t protocol)
{
	if((param.rc.OPTIONS>>10)&0x1) {
		return 0;
	}
	fsFlag = 1;
	return 0xf2;
}


/*
 * @brief RC 데이터 송신 (Half-Duplex)
 */
int RC_halfDuplex_Transmit(uint8_t *data, uint8_t len)
{
	if(IS_FL_UART_USING == 1) return -1;

	SET_FL_UART_TX();
	SET_FL_UART_USING();

	for(int i=0; i<len; i++){
		while(!LL_USART_IsActiveFlag_TXE(USART1));
		LL_USART_TransmitData8(USART1, data[i]);
	}

	CLEAR_FL_UART_TX();
	CLEAR_FL_UART_USING();
	return 0;
}


/* Functions 2 ---------------------------------------------------------------*/

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


uint16_t RC_MSG_setChannelInfo(uint8_t chancout, uint8_t rssi)
{
	msg.RC_channels.time_boot_ms = msg.system_time.time_boot_ms;
	msg.RC_channels.chancount = chancout;
	msg.RC_channels.rssi = rssi;
	return 0;
}


volatile uint16_t heartMSG = 0;
uint16_t RC_MSG_setChannelValue(uint16_t value, uint8_t index)
{
	if(!(index>=0 && index<RC_CHANNEL_MAX)) return -1;

	value = RC_applyChannelNormMinMax(value, param.rc.channel[index].MIN, param.rc.channel[index].MAX);
	value = RC_applyChannelReverse(value, (param.rc.reversedMask>>index)&0x01);
	value = RC_applyChannelDeadZone(value, param.rc.channel[index].DZ);

	if (heartMSG > 1000) heartMSG = 0;
	msg.RC_channels.value[index] = value;
	return 0;
}


volatile uint16_t dbg_min, dbg_max, dbg_in, dbg_out;
uint16_t RC_applyChannelNormMinMax(uint16_t value, uint16_t min, uint16_t max)
{
	dbg_in = value; dbg_min = min; dbg_max = max;

	value = value<min?min:value;
	value = value>max?max:value;
	dbg_out = value;
	return value;
}


uint16_t RC_applyChannelReverse(uint16_t value, uint8_t shouldInvert)
{
	if(shouldInvert & 0x1){
		return ((1000+2000) - value);
	}
	return value;
}


uint16_t RC_applyChannelDeadZone(uint16_t value, uint8_t deadZone)
{
	if(value > (1500-deadZone) && value < (1500+deadZone)){
		return 1500;
	}
	return value;
}



/* Functions 3 ---------------------------------------------------------------*/

volatile uint8_t dbg_set_called = 0;

void setFlag(RC_FLAG i)
{
	dbg_set_called = 1;
	rxFlag |= (0x1<<i);
	return;
}

void clearFlag(RC_FLAG i)
{
	rxFlag &= (uint8_t)(~(0x1<<i));
	return;
}


int isFlag(RC_FLAG i)
{
	return (rxFlag>>i)&0x1;
}


/* Compatibility stub */
int RC_receiveIRQ2(const uint16_t data)
{
    (void)data;
    return 0;
}
