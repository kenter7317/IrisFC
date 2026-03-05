/*
 * neom8n.c
 *
 *  Created on: 2025. 12. 3.
 *      Author: coder0908
 */

#include <math.h>
#include <stdio.h>
#include "platform/platform.h"
#include "platform/usart.h"
#include "platform/i2c.h"
#include "lib/ubx/utils/ubx_queue.h"
#include "lib/crsf/crsf.h"
#include "middleware/morb/morb.h"
#include "neom8n.h"

#define NEOM8N_LEN_RXBUF 150

static UART_HandleTypeDef * const s_neom8n_huart = &huart6;
static uint8_t s_uart_rx_buf[NEOM8N_LEN_RXBUF];

static bool s_is_uart_rx_buf_full = false;

bool neom8n_init()
{


	if (HAL_UART_Receive_DMA(s_neom8n_huart, s_uart_rx_buf, NEOM8N_LEN_RXBUF) != HAL_OK) {
		return false;
	}

	return true;
}

void neom8n_loop()
{
	if (!s_is_uart_rx_buf_full) {
		return;
	}
	s_is_uart_rx_buf_full = false;

	struct topic_neom8n topic_neom8n = {0,};

	uint16_t read_len = 0;
	ubx_q_init(&(topic_neom8n.ubx_q));
	ubx_q_parse_frames(&(topic_neom8n.ubx_q), s_uart_rx_buf, NEOM8N_LEN_RXBUF, &read_len);

	morb_publish(TOPIC_NUM_NEOM8N, (void*)&topic_neom8n);


	HAL_UART_Receive_DMA(s_neom8n_huart, s_uart_rx_buf, NEOM8N_LEN_RXBUF);

}




void neom8n_uart_rx_cplt_callback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == s_neom8n_huart->Instance) {
		s_is_uart_rx_buf_full = true;
	}

}
