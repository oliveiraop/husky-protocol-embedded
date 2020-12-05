#include "Message.h"
#include <stdint.h>


Message setMessage(uint8_t *payload, uint8_t length, uint16_t messageType, uint32_t timeStamp, uint8_t flags) {
	Message nova;
	nova.length = length;
	nova.lengthCompliment = 0xFF - length;
	nova.timeStamp = timeStamp;
	nova.flags = flags;
	nova.messageType = messageType;
	nova.(*payload) = *payload;
	nova.checksum = getChecksum(nova);
}

uint16_t getChecksum(Message nova) {
	uint8_t data[nova.length + 12];
	data[0] = nova.soh;
	data[1] = nova.length;
	data[2] = nova.lenghtCompliment;
	data[3] = nova.version;
	data[4] = (uint8_t)nova.timeStamp;
	data[5] = (uint8_t)(nova.timeStamp >> 8);
	data[6] = (uint8_t)(nova.timeStamp >> 16);
	data[7] = (uint8_t)(nova.timeStamp >> 24);
	data[8] = nova.flags;
	data[9] = (uint8_t)nova.messageType;
	data[10] = (uint8_t)(nova.messageType >> 8);
	data[11] =  nova.stx;

	for (int i = 12; i < 12 + nova.length ; i++) {
		data[i] = nova.payload[i-12];
	}

	return crc16(16, 0xFFFF, data);

}
