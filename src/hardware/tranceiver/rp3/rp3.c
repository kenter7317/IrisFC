/*
 * rp3.c
 *
 *  Created on: 2025. 12. 3.
 *      Author: coder0908
 */

#include "platform/usart.h"
#include "lib/crsf/utils/crsf_queue.h"
#include "middleware/morb/morb.h"
#include "rp3.h"

#define UART_LEN_RXBUF ((CRSF_PLDLEN_RC_CHANNELS + CRSF_LEN_SYNC + CRSF_LEN_LEN + CRSF_LEN_CRC) * 3)

static UART_HandleTypeDef *s_huart = &huart2;
static uint8_t s_uart_rx_buf[UART_LEN_RXBUF];
static bool s_is_uart_rx_buf_full = false;






bool rp3_init()
{
	HAL_StatusTypeDef status = HAL_ERROR;




	status = HAL_UART_Receive_DMA(s_huart, s_uart_rx_buf, UART_LEN_RXBUF);
	if (status != HAL_OK) {
		return false;
	}

	return true;
}



void rp3_loop()
{
	if (!s_is_uart_rx_buf_full) {
		return;
	}

	s_is_uart_rx_buf_full = false;

	uint64_t read_len = 0;
	struct topic_rp3 topic_rp3 = {0,};
	if (!crsf_q_init(&(topic_rp3.crsf_q))) {
		return;
	}

	crsf_q_parse_frames(&(topic_rp3.crsf_q), s_uart_rx_buf, UART_LEN_RXBUF, &read_len);


	morb_publish(TOPIC_NUM_RP3, (void*)&topic_rp3);


	if (HAL_UART_Receive_DMA(s_huart, s_uart_rx_buf, UART_LEN_RXBUF) != HAL_OK) {
		assert(false);
		return;
	}

}

void rp3_uart_rx_cplt_callback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != s_huart->Instance) {
		return;
	}
	s_is_uart_rx_buf_full = true;
}


