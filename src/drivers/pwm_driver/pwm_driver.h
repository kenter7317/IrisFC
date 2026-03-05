/*
 * pwm_driver.h
 *
 *  Created on: Jun 28, 2025
 *      Author: coder0908
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "platform/platform.h"

struct drv_pwm {
	TIM_HandleTypeDef *htim;
	uint32_t channel;

	uint32_t min_input;
	uint32_t max_input;

	uint32_t min_ccr;
	uint32_t max_ccr;
};

bool drv_pwm_init(struct drv_pwm *drv_pwm, TIM_HandleTypeDef *htim, uint32_t channel, uint32_t min_input, uint32_t max_input, uint32_t min_ccr, uint32_t max_ccr);
bool drv_pwm_write(struct drv_pwm *drv_pwm, uint32_t input);

