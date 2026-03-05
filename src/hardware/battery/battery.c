/*
 * battery.c
 *
 *  Created on: Nov 19, 2025
 *      Author: coder0908
 */


#include "platform/platform.h"
#include "middleware/morb/morb.h"
#include "lib/crsf/utils/crsf_queue.h"
#include "platform/adc.h"
#include "battery.h"

#define ADC1_FULLSCALE				(4095.0f)
#define ADC1_VOLTAGESCALE_CH4		((47.0f+10.0f)/10.0f)
#define ADC1_VOLTAGESCALE_CH7		((47.0f+10.0f)/10.0f)
#define ADC1_CNT_CHANNEL			2

static uint32_t s_adc1_values_lsb[ADC1_CNT_CHANNEL + 1];	// last one's for vrefint
static struct topic_battery s_topic_battery;

bool battery_init()
{


	if (HAL_ADC_Start_DMA(&hadc1, s_adc1_values_lsb, ADC1_CNT_CHANNEL+1) != HAL_OK) {
		return false;
	}

	return true;
}




void battery_loop()
{
	float vrefplus_mv = (float)VREFINT_CAL_VREF * (float)(*VREFINT_CAL_ADDR) / (float)s_adc1_values_lsb[ADC1_CNT_CHANNEL];	// battery_adc1_values[ADC1_CNT_CHANNEL] = vrefint data

	s_topic_battery.batt_0 = ((float)s_adc1_values_lsb[0]/ADC1_FULLSCALE) * (vrefplus_mv/1000.0f) * ADC1_VOLTAGESCALE_CH4;
	s_topic_battery.batt_1 = ((float)s_adc1_values_lsb[1]/ADC1_FULLSCALE) * (vrefplus_mv/1000.0f) * ADC1_VOLTAGESCALE_CH7;

	morb_publish(TOPIC_NUM_BATTERY, (void*)&s_topic_battery);



}


