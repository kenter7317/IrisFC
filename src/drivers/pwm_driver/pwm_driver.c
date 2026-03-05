/*
 * pwm_driver.c
 *
 *  Created on: Jun 28, 2025
 *      Author: coder0908
 */

#include <assert.h>
#include "pwm_driver.h"

static uint32_t map(uint32_t x, uint32_t min_in, uint32_t max_in, uint32_t min_out, uint32_t max_out)
{
  return (x - min_in) * (max_out - min_out) / (max_in - min_in) + min_out;
}

bool drv_pwm_init(struct drv_pwm *drv_pwm, TIM_HandleTypeDef *htim, uint32_t channel, uint32_t min_input, uint32_t max_input, uint32_t min_ccr, uint32_t max_ccr)
{
	assert(drv_pwm);
	assert(htim);

	drv_pwm->htim = htim;
	drv_pwm->channel = channel;

	drv_pwm->min_input = min_input;
	drv_pwm->max_input = max_input;

	drv_pwm->min_ccr = min_ccr;
	drv_pwm->max_ccr = max_ccr;

	HAL_StatusTypeDef status = HAL_TIM_PWM_Start(drv_pwm->htim, drv_pwm->channel);
	if (status != HAL_OK) {
		return false;
	}

	return true;
}

bool drv_pwm_write(struct drv_pwm *drv_pwm, uint32_t input)
{
	assert(drv_pwm);

	if (input < drv_pwm->min_input) {
		input = drv_pwm->min_input;
	} else if  (input > drv_pwm->max_input) {
		input = drv_pwm->max_input;
	}

	uint32_t ccr = map(input, drv_pwm->min_input, drv_pwm->max_input, drv_pwm->min_ccr, drv_pwm->max_ccr);

	__HAL_TIM_SET_COMPARE(drv_pwm->htim, drv_pwm->channel, ccr);

	return true;
}
