#include "stm32f10x.h"
#include "BaseConfig.h"
#include "LaunchProtocol.h"
#include "GroundLink.h"

#define GROUND_LINK_RX_BUFFER_SIZE          64U

static LaunchProtocol_Parser s_parser;
static volatile uint8_t s_rx_buffer[GROUND_LINK_RX_BUFFER_SIZE];
static volatile uint8_t s_rx_head = 0U;
static volatile uint8_t s_rx_tail = 0U;
static volatile uint8_t s_rx_resync_requested = 0U;
static volatile uint16_t s_rx_overflow_count = 0U;
static volatile uint16_t s_rx_error_count = 0U;
static uint32_t s_session_token = 0U;
static uint32_t s_last_ping_tx_ms = 0U;
static uint32_t s_last_valid_rx_ms = 0U;
static uint32_t s_launch_last_tx_ms = 0U;
static uint32_t s_launch_start_ms = 0U;
static uint8_t s_next_sequence = 0U;
static uint8_t s_pending_ping_sequence = 0U;
static uint8_t s_ping_pending = 0U;
static uint8_t s_good_response_count = 0U;
static uint8_t s_consecutive_response_misses = 0U;
static uint8_t s_local_qualified = 0U;
static uint8_t s_remote_qualified = 0U;
static uint8_t s_flight_ready = 0U;
static uint8_t s_launch_started = 0U;
static uint8_t s_launch_sequence = 0U;
static uint8_t s_launch_retry_count = 0U;
static uint8_t s_launch_acknowledged = 0U;
static uint8_t s_launch_failed = 0U;

static uint8_t GroundLink_ReadByte(uint8_t *byte)
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
	if(tail >= GROUND_LINK_RX_BUFFER_SIZE)
	{
		tail = 0U;
	}
	s_rx_tail = tail;
	return 1U;
}

static void GroundLink_RecoverRx(void)
{
	__disable_irq();
	s_rx_tail = s_rx_head;
	s_rx_resync_requested = 0U;
	__enable_irq();
	LaunchProtocol_ParserInit(&s_parser);
}

static uint8_t GroundLink_SendByte(uint8_t byte)
{
	uint32_t timeout = BASE_LINK_TX_TIMEOUT_LOOPS;

	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
	{
		if(timeout == 0U)
		{
			return 0U;
		}
		timeout--;
	}
	USART_SendData(USART1, byte);
	return 1U;
}

static uint8_t GroundLink_Send(uint8_t type,
							 uint8_t sequence,
							 uint32_t token,
							 uint16_t value,
							 uint16_t auxiliary)
{
	uint8_t frame[LAUNCH_PROTOCOL_FRAME_SIZE];
	uint8_t index;

	LaunchProtocol_Build(frame, type, sequence, token, value, auxiliary);
	for(index = 0U; index < LAUNCH_PROTOCOL_FRAME_SIZE; index++)
	{
		if(GroundLink_SendByte(frame[index]) == 0U)
		{
			return 0U;
		}
	}
	return 1U;
}

static void GroundLink_ResetQualification(void)
{
	s_ping_pending = 0U;
	s_good_response_count = 0U;
	s_consecutive_response_misses = 0U;
	s_local_qualified = 0U;
	s_remote_qualified = 0U;
	s_flight_ready = 0U;
}

static void GroundLink_HandlePong(const LaunchProtocol_Frame *frame,
								  uint32_t now_ms)
{
	if((s_launch_started != 0U) ||
	   (s_ping_pending == 0U) ||
	   (frame->sequence != s_pending_ping_sequence) ||
	   (frame->token == 0U) ||
	   ((s_session_token != 0U) &&
		(frame->token != s_session_token)))
	{
		return;
	}

	s_session_token = frame->token;
	s_ping_pending = 0U;
	s_last_valid_rx_ms = now_ms;
	s_consecutive_response_misses = 0U;
	if(s_good_response_count < BASE_LINK_QUALIFY_COUNT)
	{
		s_good_response_count++;
	}
	if(s_good_response_count >= BASE_LINK_QUALIFY_COUNT)
	{
		s_local_qualified = 1U;
	}
	s_remote_qualified =
		(uint8_t)((frame->value & LAUNCH_STATUS_LINK_QUALIFIED) != 0U);
	s_flight_ready =
		(uint8_t)((frame->value & LAUNCH_STATUS_FLIGHT_READY) != 0U);
}

static void GroundLink_HandleLaunchAck(const LaunchProtocol_Frame *frame,
									   uint32_t now_ms)
{
	if((s_launch_started == 0U) ||
	   (frame->sequence != s_launch_sequence) ||
	   (frame->token != s_session_token))
	{
		return;
	}

	s_last_valid_rx_ms = now_ms;
	if(frame->value == LAUNCH_ACK_ACCEPTED)
	{
		s_launch_acknowledged = 1U;
		s_launch_failed = 0U;
	}
	else
	{
		s_launch_failed = 1U;
	}
}

static void GroundLink_Poll(uint32_t now_ms)
{
	LaunchProtocol_Frame frame;
	uint8_t byte;

	if(s_rx_resync_requested != 0U)
	{
		GroundLink_RecoverRx();
	}
	while(GroundLink_ReadByte(&byte) != 0U)
	{
		if(LaunchProtocol_ParserFeed(&s_parser, byte, &frame) == 0U)
		{
			continue;
		}
		if(frame.type == LAUNCH_MESSAGE_PONG)
		{
			GroundLink_HandlePong(&frame, now_ms);
		}
		else if(frame.type == LAUNCH_MESSAGE_LAUNCH_ACK)
		{
			GroundLink_HandleLaunchAck(&frame, now_ms);
		}
	}
}

void GroundLink_Init(void)
{
	GPIO_InitTypeDef gpio_init;
	USART_InitTypeDef usart_init;
	NVIC_InitTypeDef nvic_init;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
						   RCC_APB2Periph_USART1,
						   ENABLE);
	/* U4: connector RX is PA10 and connector TX is PA9. */
	gpio_init.GPIO_Pin = GPIO_Pin_9;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init);
	gpio_init.GPIO_Pin = GPIO_Pin_10;
	gpio_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio_init);

	USART_StructInit(&usart_init);
	usart_init.USART_BaudRate = BASE_BLUETOOTH_BAUD_RATE;
	usart_init.USART_WordLength = USART_WordLength_8b;
	usart_init.USART_StopBits = USART_StopBits_1;
	usart_init.USART_Parity = USART_Parity_No;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &usart_init);
	USART_Cmd(USART1, ENABLE);

	LaunchProtocol_ParserInit(&s_parser);
	s_rx_head = 0U;
	s_rx_tail = 0U;
	s_rx_resync_requested = 0U;
	s_rx_overflow_count = 0U;
	s_rx_error_count = 0U;
	(void)USART1->SR;
	(void)USART1->DR;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_ClearPendingIRQ(USART1_IRQn);
	nvic_init.NVIC_IRQChannel = USART1_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 1U;
	nvic_init.NVIC_IRQChannelSubPriority = 0U;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	s_session_token = 0U;
	s_last_ping_tx_ms = 0U;
	s_last_valid_rx_ms = 0U;
	s_launch_last_tx_ms = 0U;
	s_launch_start_ms = 0U;
	s_next_sequence = 0U;
	s_pending_ping_sequence = 0U;
	s_launch_started = 0U;
	s_launch_sequence = 0U;
	s_launch_retry_count = 0U;
	s_launch_acknowledged = 0U;
	s_launch_failed = 0U;
	GroundLink_ResetQualification();
}

void GroundLink_Task(uint32_t now_ms)
{
	uint32_t launch_age_ms;

	GroundLink_Poll(now_ms);

	if(s_launch_started != 0U)
	{
		if((s_launch_acknowledged == 0U) &&
		   (s_launch_failed == 0U) &&
		   ((uint32_t)(now_ms - s_launch_last_tx_ms) >=
			BASE_LAUNCH_RETRY_PERIOD_MS))
		{
			if(s_launch_retry_count >= BASE_LAUNCH_RETRY_COUNT)
			{
				s_launch_failed = 1U;
			}
			else
			{
				launch_age_ms = (uint32_t)(now_ms - s_launch_start_ms);
				if(launch_age_ms > BASE_LAUNCH_MAX_COMMAND_AGE_MS)
				{
					launch_age_ms = BASE_LAUNCH_MAX_COMMAND_AGE_MS;
				}
				(void)GroundLink_Send(LAUNCH_MESSAGE_LAUNCH,
								  s_launch_sequence,
								  s_session_token,
								  (uint16_t)BASE_STAGE2_DELAY_MS,
								  (uint16_t)launch_age_ms);
				s_launch_retry_count++;
				s_launch_last_tx_ms = now_ms;
			}
		}
		return;
	}

	if((s_ping_pending != 0U) &&
	   ((uint32_t)(now_ms - s_last_ping_tx_ms) >=
		BASE_LINK_RESPONSE_TIMEOUT_MS))
	{
		s_ping_pending = 0U;
		if(s_local_qualified == 0U)
		{
			GroundLink_ResetQualification();
		}
		else
		{
			if(s_consecutive_response_misses <
			   BASE_LINK_MAX_CONSECUTIVE_MISSES)
			{
				s_consecutive_response_misses++;
			}
			if(s_consecutive_response_misses >=
			   BASE_LINK_MAX_CONSECUTIVE_MISSES)
			{
				GroundLink_ResetQualification();
			}
		}
	}

	if((s_local_qualified != 0U) &&
	   ((uint32_t)(now_ms - s_last_valid_rx_ms) > BASE_LINK_TIMEOUT_MS))
	{
		GroundLink_ResetQualification();
	}

	if((s_ping_pending == 0U) &&
	   ((uint32_t)(now_ms - s_last_ping_tx_ms) >=
		BASE_LINK_PING_PERIOD_MS))
	{
		s_pending_ping_sequence = s_next_sequence;
		s_next_sequence++;
		if(GroundLink_Send(LAUNCH_MESSAGE_PING,
						   s_pending_ping_sequence,
						   s_session_token,
						   0U,
						   0U) != 0U)
		{
			s_ping_pending = 1U;
			s_last_ping_tx_ms = now_ms;
		}
		else
		{
			GroundLink_ResetQualification();
		}
	}
}

uint8_t GroundLink_IsReady(void)
{
	return (uint8_t)((s_local_qualified != 0U) &&
					 (s_remote_qualified != 0U) &&
					 (s_flight_ready != 0U) &&
					 (s_launch_started == 0U));
}

uint8_t GroundLink_StartLaunch(uint32_t now_ms)
{
	/* Keep the ready indication tolerant, but never launch on stale status. */
	if((GroundLink_IsReady() == 0U) ||
	   ((uint32_t)(now_ms - s_last_valid_rx_ms) >
		BASE_LAUNCH_READY_FRESH_MS))
	{
		return 0U;
	}

	s_launch_started = 1U;
	s_launch_sequence = s_next_sequence;
	s_next_sequence++;
	s_launch_retry_count = 1U;
	s_launch_acknowledged = 0U;
	s_launch_failed = 0U;
	s_launch_last_tx_ms = now_ms;
	s_launch_start_ms = now_ms;
	if(GroundLink_Send(LAUNCH_MESSAGE_LAUNCH,
						 s_launch_sequence,
						 s_session_token,
						 (uint16_t)BASE_STAGE2_DELAY_MS,
						 0U) == 0U)
	{
		s_launch_retry_count = 0U;
	}
	return 1U;
}

uint8_t GroundLink_HasStarted(void)
{
	return s_launch_started;
}

uint8_t GroundLink_IsLaunchAcknowledged(void)
{
	return s_launch_acknowledged;
}

uint8_t GroundLink_HasLaunchFailed(void)
{
	return s_launch_failed;
}

void USART1_IRQHandler(void)
{
	uint32_t status;
	uint8_t byte;
	uint8_t next_head;

	status = USART1->SR;
	if((status & (USART_FLAG_RXNE | USART_FLAG_ORE | USART_FLAG_NE |
				 USART_FLAG_FE | USART_FLAG_PE)) == 0U)
	{
		return;
	}
	byte = (uint8_t)USART1->DR;
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
	if(next_head >= GROUND_LINK_RX_BUFFER_SIZE)
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
