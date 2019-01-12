#ifndef _MAVLINKLITE_H_
#define _MAVLINKLITE_H_

#include "stdint.h"

#define MAVLINK_HEADER		0xfe
#define MAVLINK_PAYLOAD_LEN_MAX		32

typedef enum {
	searchHeader,
	check
}decodeStatus_e;
	

typedef struct{
	decodeStatus_e decodeStatus;
	
	uint16_t (*mavLinkLiteCheckFunc)(uint8_t *buf, uint16_t len);
	int (*mavLinkLiteSendFunc)(uint8_t *buf, uint16_t len);
	uint8_t decodeFrame[MAVLINK_PAYLOAD_LEN_MAX+4];
	uint16_t decodeFrameCharCnt;
}mavLinkLiteHandle_t;



#endif

