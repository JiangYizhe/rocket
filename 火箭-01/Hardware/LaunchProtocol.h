#ifndef __LAUNCH_PROTOCOL_H
#define __LAUNCH_PROTOCOL_H

#include <stdint.h>

#define LAUNCH_PROTOCOL_SOF_1                 0xA5U
#define LAUNCH_PROTOCOL_SOF_2                 0x5AU
#define LAUNCH_PROTOCOL_VERSION               0x01U
#define LAUNCH_PROTOCOL_FRAME_SIZE            15U

#define LAUNCH_MESSAGE_PING                   0x10U
#define LAUNCH_MESSAGE_LAUNCH                 0x20U
#define LAUNCH_MESSAGE_PONG                   0x90U
#define LAUNCH_MESSAGE_LAUNCH_ACK             0xA0U

#define LAUNCH_STATUS_LINK_QUALIFIED          0x0001U
#define LAUNCH_STATUS_FLIGHT_READY            0x0002U

#define LAUNCH_ACK_ACCEPTED                   0x0001U
#define LAUNCH_ACK_REJECTED                   0x0000U

typedef struct
{
	uint8_t type;
	uint8_t sequence;
	uint32_t token;
	uint16_t value;
	uint16_t auxiliary;
} LaunchProtocol_Frame;

typedef struct
{
	uint8_t data[LAUNCH_PROTOCOL_FRAME_SIZE];
	uint8_t index;
} LaunchProtocol_Parser;

void LaunchProtocol_ParserInit(LaunchProtocol_Parser *parser);
uint8_t LaunchProtocol_ParserFeed(LaunchProtocol_Parser *parser,
								  uint8_t byte,
								  LaunchProtocol_Frame *frame);
void LaunchProtocol_Build(uint8_t output[LAUNCH_PROTOCOL_FRAME_SIZE],
						  uint8_t type,
						  uint8_t sequence,
						  uint32_t token,
						  uint16_t value,
						  uint16_t auxiliary);

#endif
