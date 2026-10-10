#include "LaunchProtocol.h"

static uint16_t LaunchProtocol_Crc16(const uint8_t *data, uint8_t length)
{
	uint16_t crc = 0xFFFFU;
	uint8_t index;
	uint8_t bit;

	for(index = 0U; index < length; index++)
	{
		crc ^= (uint16_t)data[index] << 8;
		for(bit = 0U; bit < 8U; bit++)
		{
			if((crc & 0x8000U) != 0U)
			{
				crc = (uint16_t)((crc << 1) ^ 0x1021U);
			}
			else
			{
				crc <<= 1;
			}
		}
	}
	return crc;
}

void LaunchProtocol_ParserInit(LaunchProtocol_Parser *parser)
{
	parser->index = 0U;
}

uint8_t LaunchProtocol_ParserFeed(LaunchProtocol_Parser *parser,
								  uint8_t byte,
								  LaunchProtocol_Frame *frame)
{
	uint16_t received_crc;
	uint16_t calculated_crc;

	if(parser->index == 0U)
	{
		if(byte == LAUNCH_PROTOCOL_SOF_1)
		{
			parser->data[0] = byte;
			parser->index = 1U;
		}
		return 0U;
	}

	if(parser->index == 1U)
	{
		if(byte == LAUNCH_PROTOCOL_SOF_2)
		{
			parser->data[1] = byte;
			parser->index = 2U;
		}
		else if(byte != LAUNCH_PROTOCOL_SOF_1)
		{
			parser->index = 0U;
		}
		return 0U;
	}

	parser->data[parser->index] = byte;
	parser->index++;
	if(parser->index < LAUNCH_PROTOCOL_FRAME_SIZE)
	{
		return 0U;
	}
	parser->index = 0U;

	if(parser->data[2] != LAUNCH_PROTOCOL_VERSION)
	{
		return 0U;
	}
	received_crc = (uint16_t)parser->data[13] |
		((uint16_t)parser->data[14] << 8);
	calculated_crc = LaunchProtocol_Crc16(parser->data, 13U);
	if(received_crc != calculated_crc)
	{
		return 0U;
	}

	frame->type = parser->data[3];
	frame->sequence = parser->data[4];
	frame->token =
		(uint32_t)parser->data[5] |
		((uint32_t)parser->data[6] << 8) |
		((uint32_t)parser->data[7] << 16) |
		((uint32_t)parser->data[8] << 24);
	frame->value = (uint16_t)parser->data[9] |
		((uint16_t)parser->data[10] << 8);
	frame->auxiliary = (uint16_t)parser->data[11] |
		((uint16_t)parser->data[12] << 8);
	return 1U;
}

void LaunchProtocol_Build(uint8_t output[LAUNCH_PROTOCOL_FRAME_SIZE],
						  uint8_t type,
						  uint8_t sequence,
						  uint32_t token,
						  uint16_t value,
						  uint16_t auxiliary)
{
	uint16_t crc;

	output[0] = LAUNCH_PROTOCOL_SOF_1;
	output[1] = LAUNCH_PROTOCOL_SOF_2;
	output[2] = LAUNCH_PROTOCOL_VERSION;
	output[3] = type;
	output[4] = sequence;
	output[5] = (uint8_t)token;
	output[6] = (uint8_t)(token >> 8);
	output[7] = (uint8_t)(token >> 16);
	output[8] = (uint8_t)(token >> 24);
	output[9] = (uint8_t)value;
	output[10] = (uint8_t)(value >> 8);
	output[11] = (uint8_t)auxiliary;
	output[12] = (uint8_t)(auxiliary >> 8);
	crc = LaunchProtocol_Crc16(output, 13U);
	output[13] = (uint8_t)crc;
	output[14] = (uint8_t)(crc >> 8);
}
