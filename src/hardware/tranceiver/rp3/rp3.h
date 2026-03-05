/*
 * rp3.h
 *
 *  Created on: 2025. 12. 3.
 *      Author: coder0908
 */

#pragma once

#include <stdbool.h>
#include "platform/platform.h"


/*
 * elrs
 * 	100hz full or 333hz full
 * 	8channel
 *
 * */

bool rp3_init();
void rp3_loop();
void rp3_uart_rx_cplt_callback(UART_HandleTypeDef *huart);

