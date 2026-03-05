/*
 * morb.c
 *
 *  Created on: 2026. 1. 27.
 *      Author: coder0908
 */

#include <assert.h>
#include "platform/platform.h"
#include "morb.h"


#define MORB_MAX_CALLBACKS_LEN 10


struct morb {
	morb_callback_fp callbacks[MORB_MAX_CALLBACKS_LEN];
	uint32_t callbacks_len;
};

static struct morb morb_topics[TOPIC_CNT];

bool morb_subscribe(enum topic_number topic_num, morb_callback_fp callback)
{
	assert(callback);

	if (topic_num<0 || topic_num>=TOPIC_CNT) {
		return false;
	}

	struct morb *morb = &morb_topics[topic_num];

	if (morb->callbacks_len == MORB_MAX_CALLBACKS_LEN) {
		return false;
	}

	morb->callbacks[morb->callbacks_len] = callback;
	morb->callbacks_len += 1;

	return true;
}

bool morb_unsubscribe(enum topic_number topic_num, morb_callback_fp callback)
{
	assert(callback);

	struct morb *morb = &morb_topics[topic_num];

	for (uint32_t i=0; i<morb->callbacks_len; i++) {
		if (morb->callbacks[i] == callback) {
			morb->callbacks[i] = morb->callbacks[morb->callbacks_len];
			morb->callbacks_len -= 1;

			return true;
		}
	}

	return false;
}

bool morb_publish(enum topic_number topic_num, void *topic)
{
	assert(topic);

	struct topic_frame frame = {0,};
	frame.tick = HAL_GetTick();
	frame.topic_num = topic_num;

	switch (topic_num) {
	case TOPIC_NUM_ICM20602:
		frame.icm20602 = *((struct topic_icm20602*)topic);
		break;
	case TOPIC_NUM_NEOM8N:
		frame.neom8n = *((struct topic_neom8n*)topic);
		break;
	case TOPIC_NUM_RP3:
			frame.rp3 = *((struct topic_rp3*)topic);
			break;
	case TOPIC_NUM_BATTERY:
			frame.battery= *((struct topic_battery*)topic);
			break;
	case TOPIC_NUM_ACTUATOR:
			frame.actuator= *((struct topic_actuator*)topic);
			break;
	case TOPIC_NUM_HMC5883L:
			frame.hmc5883l = *((struct topic_hmc5883l*)topic);
			break;
	case TOPIC_NUM_ATTITUDE_ESTIMATOR:
			frame.attitude_estimator= *((struct topic_attitude_estimator*)topic);
			break;
	case TOPIC_NUM_ATTITUDE_CONTROLLER:
			frame.attitude_controller= *((struct topic_attitude_controller*)topic);
			break;
	default:
		return false;
	}

	struct morb *morb = &morb_topics[topic_num];

	for (uint32_t i=0; i<morb->callbacks_len; i++) {
		morb->callbacks[i](&frame);
	}

	return true;
}

