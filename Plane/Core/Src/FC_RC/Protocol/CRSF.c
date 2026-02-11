/*
 * FC_RC/Protocol/CRSF/CRSF.c
 *
 *  Created on: Nov 12, 2025
 *      Author: tokhu
 */

/* Includes ------------------------------------------------------------------*/
#include <FC_RC/Protocol/CRSF/CRSF.h>
#include <FC_RC/Protocol/CRSF/CRSF_type.h>
#include <FC_RC/Protocol/CRSF/CRSF_module.h>

/* Variables -----------------------------------------------------------------*/
static uint8_t s_rssi = 0;  /* 프레임 기반 추정 */

/* ===== 내부 유틸 ============================================= */

/* CRC-8(D5), init 0x00 */
static uint8_t CRSF_calcCRC8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0x00;
    for (uint32_t i = 0; i < len; i++) {
        uint8_t in = data[i];
        for (int b = 0; b < 8; b++) {
            uint8_t mix = (crc ^ in) & 0x80;
            crc <<= 1;
            if (mix) crc ^= 0xD5u;
            in <<= 1;
        }
    }
    return crc;
}

/* 16채널 × 11bit 언팩 → raw[0..2047] */
static void CRSF_unpackChannels11bit(const uint8_t *payload22, uint16_t *raw16)
{
    uint32_t bitpos = 0;
    for (int ch = 0; ch < (int)CRSF_RC_CHANS; ch++) {
        const uint32_t byte_ix = bitpos >> 3;
        const uint32_t bit_off = bitpos & 7;
        uint32_t v =  (uint32_t)payload22[byte_ix]
                    | ((uint32_t)payload22[byte_ix + 1] << 8)
                    | ((uint32_t)payload22[byte_ix + 2] << 16);
        v >>= bit_off;
        raw16[ch] = (uint16_t)(v & 0x7FFu);
        bitpos += 11;
    }
}

/* raw(172..1811) → 1000..2000us */
static inline uint16_t CRSF_rawToUs(uint16_t raw)
{
    const float in_min = 172.f, in_max = 1811.f;
    float usf = 1000.f + ((float)raw - in_min) * 1000.f / (in_max - in_min);
    if (usf < 1000.f) usf = 1000.f;
    if (usf > 2000.f) usf = 2000.f;
    return (uint16_t)(usf + 0.5f);
}

/* ===== 공개 함수(헤더 4개 그대로) ========================================= */

int CRSF_connect(void)
{
    s_rssi = 0;
    LL_USART_Disable(USART1);  // 일시 비활성화
        LL_USART_SetBaudRate(USART1,
                             SystemCoreClock,
                             LL_USART_OVERSAMPLING_16,
                             420000);  // ELRS 기본 속도 420 000 bps
        LL_USART_Enable(USART1);   // 다시 활성화
        /* ======================================== */


    return 0;
}

int CRSF_getControlData(void)
{
    if (IS_FL_RX == 0) return -1;
    CLEAR_FL_RX();

    /* RC_Buffer: [0]=addr, [1]=len(type..payload..crc), [2]=type, [3..]=payload, [end]=crc */
    const uint8_t addr = RC_Buffer[0];
    const uint8_t len  = RC_Buffer[1];

    if (addr != CRSF_ADDR_RX) return -2;
    if (len < 3 || len > (CRSF_MAX_BUFFER_SIZE - 2)) return -2;

    /* CRC 검증: 대상=type..payload(len-1), 마지막=crc */
    {
        const uint8_t crc_calc = CRSF_calcCRC8(&RC_Buffer[2], (uint32_t)(len - 1U));
        const uint8_t crc_rx   = RC_Buffer[1 + 1 + len - 1]; /* = RC_Buffer[2 + len - 1] */
        if (crc_calc != crc_rx) return -2;
    }

    /* RC 프레임만 처리 */
    if (RC_Buffer[2] != CRSF_FRAME_TYPE_RC) return -2;

    /* 채널 언팩 → us 매핑 → 메시지 반영 */
    {
        const uint8_t *payload = &RC_Buffer[3]; /* 22바이트 */
        uint16_t raw[CRSF_RC_CHANS];
        CRSF_unpackChannels11bit(payload, raw);

        for (int i = 0; i < (int)CRSF_RC_CHANS; i++) {
            uint16_t us = CRSF_rawToUs(raw[i]);
            RC_MSG_setChannelValue(us, (uint8_t)i);
        }
    }

    s_rssi = 254U; /* 프레임 수신 기반 추정값 */
    RC_MSG_setChannelInfo((uint8_t)CRSF_RC_CHANS, s_rssi);

    return 0;
}

uint8_t CRSF_getRssi(void)
{
    return s_rssi;
}

int CRSF_readByteIRQ2(const uint8_t data)
{
    static uint8_t cnt    = 0;
    static uint8_t maxLen = 0; /* 총 길이 = 2 + len */

    if (RC_isBufferInit() != 0) return -2;
    if (cnt >= CRSF_MAX_BUFFER_SIZE) { cnt = 0; return -2; }

    switch (cnt)
    {
    case 0: /* address */
        if (data == CRSF_ADDR_RX) {
            RC_Buffer[cnt] = data;
            cnt++;
        } else {
            cnt = 0;
            return -1;
        }
        break;

    case 1: /* len (type..payload..crc 바이트 수) */
        if (data == 0 || data > (CRSF_MAX_BUFFER_SIZE - 2)) {
            cnt = 0;
            return -1;
        } else {
            RC_Buffer[cnt] = data;
            cnt++;
            maxLen = (uint8_t)(2U + data); /* addr + len + len */
        }
        break;

    default:
        RC_Buffer[cnt] = data;
        if (cnt == (uint8_t)(maxLen - 1U)) {
            cnt = 0;
            return 0;      /* 프레임 끝 → 상위( RadioControl.c )가 SET_FL_RX() */
        } else {
            cnt++;
        }
        break;
    }

    return 1;
}
