#include "stm32f10x.h"
#include "RocketConfig.h"
#include "LaunchProtocol.h"
#include "RemoteLaunch.h"

#define REMOTE_LAUNCH_UID_BASE              0x1FFFF7E8UL
#define REMOTE_LAUNCH_RX_BUFFER_SIZE        64U

static LaunchProtocol_Parser s_parser;
static volatile uint8_t s_rx_buffer[REMOTE_LAUNCH_RX_BUFFER_SIZE];
static volatile uint8_t s_rx_head = 0U;
static volatile uint8_t s_rx_tail = 0U;
static volatile uint8_t s_rx_resync_requested = 0U;
static volatile uint16_t s_rx_overflow_count = 0U;
static volatile uint16_t s_rx_error_count = 0U;
static uint32_t s_session_token = 0U;
static uint32_t s_last_ping_ms = 0U;
static uint8_t s_last_ping_sequence = 0U;
static uint8_t s_have_ping = 0U;
static uint8_t s_good_ping_count = 0U;
static uint8_t s_sequence_gap_count = 0U;
static uint8_t s_link_qualified = 0U;
static uint8_t s_flight_ready = 0U;
static uint8_t s_launch_requested = 0U;
static uint8_t s_launch_started = 0U;
static uint8_t s_launch_sequence = 0U;
static uint16_t s_launch_age_ms = 0U;

static uint8_t RemoteLaunch_ReadByte(uint8_t *byte)
{
	uint8_t tail;

	if(byte == 0)
	{
		return 0U;
	}
	tail = s_rx_tail;
	if(tail == s_rx_head)
	{
		return 0U;
	}
	*byte = s_rx_buffer[tail];
	tail++;
	if(tail >= REMOTE_LAUNCH_RX_BUFFER_SIZE)
	{
		tail = 0U;
	}
	s_rx_tail = tail;
	return 1U;
}

static void RemoteLaunch_RecoverRx(void)
{
	__disable_irq();
	s_rx_tail = s_rx_head;
	s_rx_resync_requested = 0U;
	__enable_irq();
	LaunchProtocol_ParserInit(&s_parser);
}

static uint32_t RemoteLaunch_CreateToken(uint32_t now_ms)
{
	volatile const uint32_t *uid =
		(volatile const uint32_t *)REMOTE_LAUNCH_UID_BASE;
	uint32_t token;

	token = uid[0] ^ uid[1] ^ uid[2] ^ now_ms ^ 0x524F434BU;
	if(token == 0U)
	{
		token = 0x4C41554EU;
	}
	return token;
}

static uint8_t RemoteLaunch_SendByte(uint8_t byte)
{
	uint32_t timeout = ROCKET_REMOTE_TX_TIMEOUT_LOOPS;

	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
	{
		if(timeout == 0U)
		{
			return 0U;
		}
		timeout--;
	}
	USART_SendData(USART2, byte);
	return 1U;
}

static void RemoteLaunch_Send(uint8_t type,
							  uint8_t sequence,
							  uint16_t value)
{
	uint8_t frame[LAUNCH_PROTOCOL_FRAME_SIZE];
	uint8_t index;

	LaunchProtocol_Build(frame, type, sequence, s_session_token, value, 0U);
	for(index = 0U; index < LAUNCH_PROTOCOL_FRAME_SIZE; index++)
	{
		if(RemoteLaunch_SendByte(frame[index]) == 0U)
		{
			break;
		}
	}
}

static void RemoteLaunch_ResetQualification(void)
{
	s_have_ping = 0U;
	s_good_ping_count = 0U;
	s_sequence_gap_count = 0U;
	s_link_qualified = 0U;
	s_flight_ready = 0U;
}

static void RemoteLaunch_HandlePing(const LaunchProtocol_Frame *frame,
									uint32_t now_ms)
{
	uint16_t status = 0U;
	uint8_t expected_sequence;

	if(frame->token == 0U)
	{
		s_session_token = RemoteLaunch_CreateToken(now_ms);
		RemoteLaunch_ResetQualification();
	}
	else if(frame->token != s_session_token)
	{
		return;
	}

	if(s_have_ping == 0U)
	{
		s_good_ping_count = 1U;
		s_sequence_gap_count = 0U;
	}
	else
	{
		expected_sequence = (uint8_t)(s_last_ping_sequence + 1U);
		if((frame->sequence == expected_sequence) &&
		   ((uint32_t)(now_ms - s_last_ping_ms) <=
			ROCKET_REMOTE_PING_MAX_GAP_MS))
		{
			s_sequence_gap_count = 0U;
			if(s_good_ping_count < ROCKET_REMOTE_QUALIFY_COUNT)
			{
				s_good_ping_count++;
			}
		}
		else
		{
			if(s_link_qualified == 0U)
			{
				s_good_ping_count = 1U;
				s_sequence_gap_count = 0U;
			}
			else
			{
				if(s_sequence_gap_count <
				   ROCKET_REMOTE_MAX_SEQUENCE_GAPS)
				{
					s_sequence_gap_count++;
				}
				if(s_sequence_gap_count >=
				   ROCKET_REMOTE_MAX_SEQUENCE_GAPS)
				{
					s_good_ping_count = 1U;
					s_sequence_gap_count = 0U;
					s_link_qualified = 0U;
				}
			}
		}
	}
	s_have_ping = 1U;
	s_last_ping_sequence = frame->sequence;
	s_last_ping_ms = now_ms;
	if(s_good_ping_count >= ROCKET_REMOTE_QUALIFY_COUNT)
	{
		s_link_qualified = 1U;
		status |= LAUNCH_STATUS_LINK_QUALIFIED;
	}
	if((s_link_qualified != 0U) && (s_flight_ready != 0U) &&
	   (s_launch_started == 0U))
	{
		status |= LAUNCH_STATUS_FLIGHT_READY;
	}
	RemoteLaunch_Send(LAUNCH_MESSAGE_PONG, frame->sequence, status);
}

static void RemoteLaunch_HandleLaunch(const LaunchProtocol_Frame *frame,
									  uint32_t now_ms)
{
	uint8_t accepted = 0U;

	if((s_launch_started != 0U) &&
	   (frame->token == s_session_token) &&
	   (frame->sequence == s_launch_sequence))
	{
		accepted = 1U;
	}
	/* The flight controller owns ignition timing; frame->value is informational. */
	else if((s_launch_started == 0U) &&
			(frame->token == s_session_token) &&
			(frame->auxiliary <= ROCKET_REMOTE_MAX_COMMAND_AGE_MS) &&
			(s_link_qualified != 0U) &&
			(s_flight_ready != 0U) &&
			((uint32_t)(now_ms - s_last_ping_ms) <=
			 ROCKET_REMOTE_LINK_TIMEOUT_MS))
	{
		s_launch_started = 1U;
		s_launch_sequence = frame->sequence;
		s_launch_age_ms = frame->auxiliary;
		s_launch_requested = 1U;
		accepted = 1U;
	}

	RemoteLaunch_Send(LAUNCH_MESSAGE_LAUNCH_ACK,
						frame->sequence,
						(accepted != 0U) ?
						LAUNCH_ACK_ACCEPTED : LAUNCH_ACK_REJECTED);
}

void RemoteLaunch_Init(void)
{
	GPIO_InitTypeDef gpio_init;
	USART_InitTypeDef usart_init;
	NVIC_InitTypeDef nvic_init;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	gpio_init.GPIO_Pin = GPIO_Pin_2;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);
	gpio_init.GPIO_Pin = GPIO_Pin_3;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio_init);

	USART_StructInit(&usart_init);
	usart_init.USART_BaudRate = ROCKET_REMOTE_BAUD_RATE;
	usart_init.USART_WordLength = USART_WordLength_8b;
	usart_init.USART_StopBits = USART_StopBits_1;
	usart_init.USART_Parity = USART_Parity_No;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &usart_init);
	USART_Cmd(USART2, ENABLE);

	LaunchProtocol_ParserInit(&s_parser);
	s_rx_head = 0U;
	s_rx_tail = 0U;
	s_rx_resync_requested = 0U;
	s_rx_overflow_count = 0U;
	s_rx_error_count = 0U;
	(void)USART2->SR;
	(void)USART2->DR;
	NVIC_ClearPendingIRQ(USART2_IRQn);
	nvic_init.NVIC_IRQChannel = USART2_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 1U;
	nvic_init.NVIC_IRQChannelSubPriority = 0U;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	s_session_token = 0U;
	s_last_ping_ms = 0U;
	s_last_ping_sequence = 0U;
	s_have_ping = 0U;
	s_good_ping_count = 0U;
	s_sequence_gap_count = 0U;
	s_link_qualified = 0U;
	s_flight_ready = 0U;
	s_launch_requested = 0U;
	s_launch_started = 0U;
	s_launch_sequence = 0U;
	s_launch_age_ms = 0U;
}

void RemoteLaunch_Task(uint32_t now_ms, uint8_t flight_ready)
{
	LaunchProtocol_Frame frame;
	uint8_t byte;

	s_flight_ready = flight_ready;
	if(s_rx_resync_requested != 0U)
	{
		RemoteLaunch_RecoverRx();
	}
	while(RemoteLaunch_ReadByte(&byte) != 0U)
	{
		if(LaunchProtocol_ParserFeed(&s_parser, byte, &frame) == 0U)
		{
			continue;
		}
		if(frame.type == LAUNCH_MESSAGE_PING)
		{
			RemoteLaunch_HandlePing(&frame, now_ms);
		}
		else if(frame.type == LAUNCH_MESSAGE_LAUNCH)
		{
			RemoteLaunch_HandleLaunch(&frame, now_ms);
		}
	}

	if((s_launch_started == 0U) && (s_have_ping != 0U) &&
	   ((uint32_t)(now_ms - s_last_ping_ms) >
		ROCKET_REMOTE_LINK_TIMEOUT_MS))
	{
		RemoteLaunch_ResetQualification();
	}
}

uint8_t RemoteLaunch_ConsumeRequest(void)
{
	uint8_t requested = s_launch_requested;

	s_launch_requested = 0U;
	return requested;
}

uint16_t RemoteLaunch_GetLaunchAgeMs(void)
{
	return s_launch_age_ms;
}

uint8_t RemoteLaunch_IsLinkQualified(void)
{
	return s_link_qualified;
}

uint8_t RemoteLaunch_IsReady(void)
{
	return (uint8_t)((s_link_qualified != 0U) &&
					 (s_flight_ready != 0U) &&
					 (s_launch_started == 0U));
}

uint8_t RemoteLaunch_HasStarted(void)
{
	return s_launch_started;
}

void USART2_IRQHandler(void)
{
	uint32_t status;
	uint8_t byte;
	uint8_t next_head;

	status = USART2->SR;
	if((status & (USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_NE |
				 USART_FLAG_FE | USART_FLAG_PE)) == 0U)
	{
		return;
	}
	byte = (uint8_t)USART2->DR;
	if((status & (USART_FLAG_ORE | USART_FLAG_NE |
				 USART_FLAG_FE | USART_FLAG_PE)) != 0U)
	{
		s_rx_error_count++;
		s_rx_resync_requested = 1U;
		return;
	}
	if((status & USART_FLAG_RXNE) == 0U)
	{
		return;
	}

	next_head = (uint8_t)(s_rx_head + 1U);
	if(next_head >= REMOTE_LAUNCH_RX_BUFFER_SIZE)
	{
		next_head = 0U;
	}
	if(next_head == s_rx_tail)
	{
		s_rx_overflow_count++;
		s_rx_resync_requested = 1U;
		return;
	}
	s_rx_buffer[s_rx_head] = byte;
	s_rx_head = next_head;
}
