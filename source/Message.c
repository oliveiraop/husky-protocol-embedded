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
	nova.checksum = crc16(nova);
}


