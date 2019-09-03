/*
 * mavLinkLite.h
 *
 *  Created on: 2019-1-14
 *      Author: zdh
 */

#ifndef MAVLINKLITE_H_
#define MAVLINKLITE_H_

#include "stdint.h"

#define MAVLINK_HEADER							0xfe
#define MAVLINK_PAYLOAD_LEN_MAX			64
#define MAVLINK_CYCLE_BUFFER_LEN		128
#define MAVLINK_PROTOCOL_VERSION		0
#define MAVLINK_CHECK_BYTE_LEN			2
#define MAVLINKLITE_DEBUG						0

typedef struct{
	uint8_t frameHeader;
	uint8_t protocolVersion;
	uint16_t payloadLength;
}mavLinkLiteHeader_t;

typedef struct{
	uint16_t decodeFrameParseCharIndex;
	uint16_t decodeFrameWriteInCharIndex;
	uint8_t decodeFrame[MAVLINK_CYCLE_BUFFER_LEN];

	uint16_t (*mavLinkLiteCheckFunc)(uint8_t *buf, uint16_t len);
	int (*mavLinkLiteSendFunc)(uint8_t *buf, uint16_t len);
	void (*mavLinkLiteCommandIdParse)(uint8_t *buf, int len);
}mavLinkLiteHandle_t;

//the following functions have no measure about reentry, make sure there is no context during the function is execute
void mavLinkLiteInit(mavLinkLiteHandle_t *mavLinkLiteHandle);
int mavLinkLitePayloadSend(mavLinkLiteHandle_t *mavLinkLiteHandle, uint8_t *buf, uint16_t len);
int mavLinkLiteDecodeStreamWriteIn(mavLinkLiteHandle_t *mavLinkLiteHandle, uint8_t *buf, uint16_t len);
//make sure only one service function, this function do not have any measure about reentry 
int mavLinkLiteDecode(mavLinkLiteHandle_t *mavLinkLiteHandle);

#endif /* MAVLINKLITE_H_ */
