/*
 * main.c
 *
 *  Created on: 2026. 1. 20.
 *      Author: coder0908
 */

#include <stdio.h>
#include "platform/platform.h"
#include "platform/usart.h"
#include "hardware/battery/battery.h"
#include "hardware/actuator/actuator.h"
#include "hardware/gnss/neom8n/neom8n.h"
#include "hardware/imu/icm20602/icm20602.h"
#include "hardware/tranceiver/rtt24/rtt24.h"
#include "hardware/tranceiver/rp3/rp3.h"
#include "hardware/tranceiver/rtt24/rtt24.h"

#include "modules/attitude_controller/attitude_controller.h"
#include "modules/attitude_estimator/attitude_estimator.h"


int _write(int file, char* p, int len){
	HAL_UART_Transmit(&huart1, (uint8_t*)p, len, 10);
	return len;
}

int main()
{
	platform_init();

	if (!battery_init()) {
		return -1;
	}

	if (!actuator_init()) {
		return -1;
	}

	if (!neom8n_init()) {
		return -1;
	}

	if (!icm20602_init()) {
		return -1;
	}

	if (!rp3_init()) {
		return -1;
	}

	if (!attitude_controller_init()) {
		return -1;
	}

	if (!attitude_estimator_init()) {
		return -1;
	}

	if (!rtt24_init()) {
		return -1;
	}

	int i=0;

	for (;;) {

		battery_loop();
		actuator_loop();
		neom8n_loop();
		icm20602_loop();
		rp3_loop();
		attitude_controller_loop();
		attitude_estimator_loop();
		rtt24_loop();

		i++;
	}

	return 0;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	neom8n_uart_rx_cplt_callback(huart);
	rp3_uart_rx_cplt_callback(huart);
}

