/*
 * FC_RC/Protocol/CRSF/CRSF_type.h
 *
 *  Created on: Nov 12, 2025
 *      Author: tokhu
 */

#ifndef INC_FC_RC_PROTOCOL_CRSF_CRSF_TYPE_H_
#define INC_FC_RC_PROTOCOL_CRSF_CRSF_TYPE_H_

#include "main.h"

/* CRSF 상수 */
#define CRSF_ADDR_RX            (0xC8U)   /* Receiver → FC */
#define CRSF_FRAME_TYPE_RC      (0x16U)   /* RC 채널 프레임 */
#define CRSF_RC_CHANS           (16U)     /* 16채널 */
#define CRSF_RC_PAYLOAD_LEN     (22U)     /* 16ch * 11bit = 22바이트 */
#define CRSF_MAX_FRAME_LEN      (64U)     /* 최대 프레임 길이 */

#endif /* INC_FC_RC_PROTOCOL_CRSF_CRSF_TYPE_H_ */
