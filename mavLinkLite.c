/*
 * mavLinkLite.c
 *
 *  Created on: 2019-1-14
 *      Author: zdh
 */

#include "mavLinkLite.h"
#include "string.h"
#include "stdio.h"

void mavLinkLiteInit(mavLinkLiteHandle_t *mavLinkLiteHandle){
	mavLinkLiteHandle->decodeFrameParseCharIndex = 0;
	mavLinkLiteHandle->decodeFrameWriteInCharIndex = 0;
}

int mavLinkLitePayloadSend(mavLinkLiteHandle_t *mavLinkLiteHandle, uint8_t *buf, uint16_t len){
	uint8_t frame[MAVLINK_PAYLOAD_LEN_MAX + sizeof(mavLinkLiteHeader_t) + MAVLINK_CHECK_BYTE_LEN]={0};
	uint16_t checkValue;
	mavLinkLiteHeader_t header;

	if(len > MAVLINK_PAYLOAD_LEN_MAX){
		return -1;
	}

	header.frameHeader = MAVLINK_HEADER;
	header.protocolVersion = MAVLINK_PROTOCOL_VERSION;
	header.payloadLength = len;
	memcpy(frame, &header, sizeof(mavLinkLiteHeader_t));

	memcpy(frame+sizeof(mavLinkLiteHeader_t), buf , len);

	checkValue = mavLinkLiteHandle->mavLinkLiteCheckFunc(frame, len+sizeof(mavLinkLiteHeader_t));
	memcpy(frame+sizeof(mavLinkLiteHeader_t)+len, (uint8_t *)&checkValue, sizeof(checkValue));

	return mavLinkLiteHandle->mavLinkLiteSendFunc(frame, len+sizeof(mavLinkLiteHeader_t)+sizeof(checkValue));
}

int mavLinkLiteDecodeStreamWriteIn(mavLinkLiteHandle_t *mavLinkLiteHandle, uint8_t *buf, uint16_t len){
	uint16_t i=0;

	for(i=0; i<len; i++){
		mavLinkLiteHandle->decodeFrame[mavLinkLiteHandle->decodeFrameWriteInCharIndex++] = buf[i];
		if(mavLinkLiteHandle->decodeFrameWriteInCharIndex == MAVLINK_CYCLE_BUFFER_LEN){
			mavLinkLiteHandle->decodeFrameWriteInCharIndex = 0;
		}
	}

	return len;
}

int mavLinkLiteDecode(mavLinkLiteHandle_t *mavLinkLiteHandle){
	uint16_t i=0;
	uint16_t unparsedCharCnt=0;

	//if cycle buffer write in index not equal to parsed index
	if(mavLinkLiteHandle->decodeFrameWriteInCharIndex != mavLinkLiteHandle->decodeFrameParseCharIndex){
		//calculate how many bytes need to parse
		if(mavLinkLiteHandle->decodeFrameWriteInCharIndex > mavLinkLiteHandle->decodeFrameParseCharIndex){
			unparsedCharCnt = mavLinkLiteHandle->decodeFrameWriteInCharIndex - mavLinkLiteHandle->decodeFrameParseCharIndex;
		}else{
			unparsedCharCnt = MAVLINK_CYCLE_BUFFER_LEN - mavLinkLiteHandle->decodeFrameParseCharIndex + mavLinkLiteHandle->decodeFrameWriteInCharIndex;
		}
		//unpared bytes at least have header and check bytes
		if(unparsedCharCnt > sizeof(mavLinkLiteHeader_t) + MAVLINK_CHECK_BYTE_LEN){
			//get frame header if not pass first byte 
			if(mavLinkLiteHandle->decodeFrame[mavLinkLiteHandle->decodeFrameParseCharIndex] == MAVLINK_HEADER){
				mavLinkLiteHeader_t *header;
				uint8_t unparsedFrame[MAVLINK_PAYLOAD_LEN_MAX + sizeof(mavLinkLiteHeader_t) + MAVLINK_CHECK_BYTE_LEN]={0};

				//copy header to buffer
				for(i=0; i<sizeof(mavLinkLiteHeader_t); i++){
					if(mavLinkLiteHandle->decodeFrameParseCharIndex + i >= MAVLINK_CYCLE_BUFFER_LEN){
						unparsedFrame[i] = mavLinkLiteHandle->decodeFrame[mavLinkLiteHandle->decodeFrameParseCharIndex + i - MAVLINK_CYCLE_BUFFER_LEN];
					}else{
						unparsedFrame[i] = mavLinkLiteHandle->decodeFrame[mavLinkLiteHandle->decodeFrameParseCharIndex + i];
					}
				}
				
				header = (mavLinkLiteHeader_t *)unparsedFrame;
				//check if length is legal, else, bad frame pass first byte
				if(header->payloadLength < MAVLINK_PAYLOAD_LEN_MAX){
					//check length is enough to parse 
					if(unparsedCharCnt >= header->payloadLength + sizeof(mavLinkLiteHeader_t) + MAVLINK_CHECK_BYTE_LEN){
						//copy whole frame to buffer
						for(; i<header->payloadLength + sizeof(mavLinkLiteHeader_t) + MAVLINK_CHECK_BYTE_LEN; i++){
							if(mavLinkLiteHandle->decodeFrameParseCharIndex + i >= MAVLINK_CYCLE_BUFFER_LEN){
								unparsedFrame[i] = mavLinkLiteHandle->decodeFrame[mavLinkLiteHandle->decodeFrameParseCharIndex + i - MAVLINK_CYCLE_BUFFER_LEN];
							}else{
								unparsedFrame[i] = mavLinkLiteHandle->decodeFrame[mavLinkLiteHandle->decodeFrameParseCharIndex + i];
							}
						}
						
						//caculate check value
						uint16_t calculateValue = mavLinkLiteHandle->mavLinkLiteCheckFunc(unparsedFrame, header->payloadLength + sizeof(mavLinkLiteHeader_t));
						//get check value in frame 
						uint16_t checkValue = unparsedFrame[header->payloadLength+sizeof(mavLinkLiteHeader_t)] | (unparsedFrame[header->payloadLength+sizeof(mavLinkLiteHeader_t)+1]<<8);

						if(checkValue == calculateValue){
							mavLinkLiteHandle->mavLinkLiteCommandIdParse(unparsedFrame, unparsedCharCnt);//command id parse
							mavLinkLiteHandle->decodeFrameParseCharIndex += i;					//write in index pass whole frame
							if(mavLinkLiteHandle->decodeFrameParseCharIndex >= MAVLINK_CYCLE_BUFFER_LEN){
								mavLinkLiteHandle->decodeFrameParseCharIndex -= MAVLINK_CYCLE_BUFFER_LEN;
							}
						}else{//bad check value, pass first byte
							mavLinkLiteHandle->decodeFrameParseCharIndex++;
							if(mavLinkLiteHandle->decodeFrameParseCharIndex == MAVLINK_CYCLE_BUFFER_LEN){
								mavLinkLiteHandle->decodeFrameParseCharIndex = 0;
							}
						}
					}
				}else{
					mavLinkLiteHandle->decodeFrameParseCharIndex++;
					if(mavLinkLiteHandle->decodeFrameParseCharIndex == MAVLINK_CYCLE_BUFFER_LEN){
						mavLinkLiteHandle->decodeFrameParseCharIndex = 0;
					}
				}
			}else{
				mavLinkLiteHandle->decodeFrameParseCharIndex++;
				if(mavLinkLiteHandle->decodeFrameParseCharIndex == MAVLINK_CYCLE_BUFFER_LEN){
					mavLinkLiteHandle->decodeFrameParseCharIndex = 0;
				}
			}
		}
	}

	return -1;
}



