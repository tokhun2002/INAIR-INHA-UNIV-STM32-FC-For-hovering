/*
 * FC_RC/Protocol/CRSF/CRSF.h
 *
 *  Created on: Nov 12, 2025
 *      Author: tokhu
 */

#ifndef INC_FC_RC_PROTOCOL_CRSF_CRSF_H_
#define INC_FC_RC_PROTOCOL_CRSF_CRSF_H_

#include "main.h"

/* Macro ---------------------------------------------------------------------*/
#define CRSF_MAX_BUFFER_SIZE    (64)

/* Functions -----------------------------------------------------------------*/
int     CRSF_connect(void);
int     CRSF_getControlData(void);
uint8_t CRSF_getRssi(void);
int     CRSF_readByteIRQ2(const uint8_t data);

#endif /* INC_FC_RC_PROTOCOL_CRSF_CRSF_H_ */
