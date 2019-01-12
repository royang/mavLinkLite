#include "mavLinkLite.h"
#include "string.h"

void mavLinkLiteInit(mavLinkLiteHandle_t *mavLinkLiteHandle){
	mavLinkLiteHandle->decodeStatus = searchHeader;
	mavLinkLiteHandle->decodeFrameCharCnt = 0;
}

int mavLinkLitePayloadSend(mavLinkLiteHandle_t *mavLinkLiteHandle, uint8_t *buf, uint16_t len){
	uint8_t frame[MAVLINK_PAYLOAD_LEN_MAX+4]={0};
	uint16_t checkValue;
	
	if(len > MAVLINK_PAYLOAD_LEN_MAX){
		return -1;
	}
	
	frame[0] = MAVLINK_HEADER;
	frame[1] = len;
	memcpy(frame+2, buf , len);
	checkValue = mavLinkLiteHandle->mavLinkLiteCheckFunc(frame, len+2);
	frame[len+2] = (checkValue&0xff00) >> 8;
	frame[len+3] = checkValue&0x00ff;
	
	mavLinkLiteHandle->mavLinkLiteSendFunc(frame, len+4);
	
	return 0;
}


int  mavLinkLiteDecode(mavLinkLiteHandle_t *mavLinkLiteHandle, uint8_t *buf, uint16_t len){
	uint16_t i=0;
	
	for(i=0; i<len; i++){
		switch(mavLinkLiteHandle->decodeStatus){
			case searchHeader:
				if(buf[i] == MAVLINK_HEADER){
					mavLinkLiteHandle->decodeFrameCharCnt = 0;
					mavLinkLiteHandle->decodeFrame[mavLinkLiteHandle->decodeFrameCharCnt] = buf[i];
				}
				break;
			case check:
				
				break;
			default:
				break;
		}
	}
	
	return -1;
}



