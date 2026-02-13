/*
 * FC_RC/Protocol/CRSF/CRSF.c
 * LL Driver Version (Half-Duplex on PA9)
 */

#include "FC_RC/Protocol/CRSF/CRSF.h"
#include "FC_RC/RC_module.h"
#include "main.h"
#include <string.h>

/* --- [변수 선언] --- */
static uint8_t crsf_rx_dma_buffer[CRSF_MAX_BUFFER_SIZE];
static uint16_t crsf_channels[16];

/* --- [내부 함수] --- */
static void CRSF_Parse_Channels(uint8_t *payload) {
    crsf_channels[0]  = ((payload[0]       ) | (payload[1] << 8)) & 0x07FF;
    crsf_channels[1]  = ((payload[1] >> 3  ) | (payload[2] << 5)) & 0x07FF;
    crsf_channels[2]  = ((payload[2] >> 6  ) | (payload[3] << 2) | (payload[4] << 10)) & 0x07FF;
    crsf_channels[3]  = ((payload[4] >> 1  ) | (payload[5] << 7)) & 0x07FF;
    crsf_channels[4]  = ((payload[5] >> 4  ) | (payload[6] << 4)) & 0x07FF;
    crsf_channels[5]  = ((payload[6] >> 7  ) | (payload[7] << 1) | (payload[8] << 9)) & 0x07FF;
    crsf_channels[6]  = ((payload[8] >> 2  ) | (payload[9] << 6)) & 0x07FF;
    crsf_channels[7]  = ((payload[9] >> 5  ) | (payload[10] << 3)) & 0x07FF;
}

static uint16_t map_crsf_to_pwm(uint16_t x) {
    if (x < 172) x = 172;
    if (x > 1811) x = 1811;
    return (uint16_t)((x - 172) * (2000 - 1000) / (1811 - 172) + 1000);
}

/* -------------------------------------------------------------------------- */

/**
  * @brief  CRSF 연결 (Half-Duplex PA9)
  */
int CRSF_connect(void)
{
    // [1] GPIO 설정 (PA9 = USART1_TX/RX Half-Duplex)
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    // PA9 설정 (기존 SRXL2와 동일한 핀 사용)
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_PUSHPULL); // 혹은 OPEN_DRAIN (상황에 따라 다름, 보통 PushPull도 됨)
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_9, LL_GPIO_AF_7); // AF7 = USART1

    // [2] USART1 설정 (반이중 모드 & 420000bps)
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

    LL_USART_Disable(USART1);
    LL_USART_ConfigHalfDuplexMode(USART1); // [핵심] 반이중 모드 설정

    // 보드레이트 420,000 설정
    // PCLK2 주파수(SystemCoreClock/2 or 1)를 가져와서 계산
    // STM32F407 기준 APB2는 보통 84MHz
    LL_USART_SetBaudRate(USART1, SystemCoreClock/2, LL_USART_OVERSAMPLING_16, 420000);

    LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
    LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);
    LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);

    LL_USART_Enable(USART1);

    // [3] DMA 설정 (USART1_RX는 DMA2 Stream2 Channel4)
    // 반이중 모드여도 수신 데이터는 RX 레지스터로 들어오므로 DMA 설정은 RX와 동일합니다.
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    LL_DMA_DisableStream(DMA2, LL_DMA_STREAM_2);

    LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);

    LL_DMA_SetPeriphAddress(DMA2, LL_DMA_STREAM_2, LL_USART_DMA_GetRegAddr(USART1));
    LL_DMA_SetMemoryAddress(DMA2, LL_DMA_STREAM_2, (uint32_t)crsf_rx_dma_buffer);
    LL_DMA_SetDataLength(DMA2, LL_DMA_STREAM_2, CRSF_MAX_BUFFER_SIZE);

    // [4] DMA 및 RX 인터럽트 활성화
    LL_DMA_EnableStream(DMA2, LL_DMA_STREAM_2);
    LL_USART_EnableDMAReq_RX(USART1);

    return 0;
}

/**
  * @brief  데이터 파싱
  */
int CRSF_getControlData(void)
{
    const uint8_t CRSF_SYNC_BYTE = 0xC8;
    const uint8_t CRSF_RC_TYPE   = 0x16;

    for (int i = 0; i < CRSF_MAX_BUFFER_SIZE - 26; i++)
    {
        if (crsf_rx_dma_buffer[i] == CRSF_SYNC_BYTE)
        {
            if (crsf_rx_dma_buffer[i+2] == CRSF_RC_TYPE)
            {
                CRSF_Parse_Channels(&crsf_rx_dma_buffer[i+3]);

                // 채널 매핑
                msg.RC_channels.value[param.rc.map.ROL] = map_crsf_to_pwm(crsf_channels[0]);
                msg.RC_channels.value[param.rc.map.PIT] = map_crsf_to_pwm(crsf_channels[1]);
                msg.RC_channels.value[param.rc.map.THR] = map_crsf_to_pwm(crsf_channels[2]);
                msg.RC_channels.value[param.rc.map.YAW] = map_crsf_to_pwm(crsf_channels[3]);

                // AUX 채널 (스위치)
                msg.RC_channels.value[4] = map_crsf_to_pwm(crsf_channels[4]);
                msg.RC_channels.value[5] = map_crsf_to_pwm(crsf_channels[5]);
                msg.RC_channels.value[6] = map_crsf_to_pwm(crsf_channels[6]);
                msg.RC_channels.value[7] = map_crsf_to_pwm(crsf_channels[7]);

                return 0;
            }
        }
    }
    return -1;
}

uint8_t CRSF_getRssi(void) { return 0; }
int CRSF_readByteIRQ2(const uint8_t data) { (void)data; return 0; }
