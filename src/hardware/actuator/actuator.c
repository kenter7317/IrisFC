/*
 * actuator.c
 *
 *  Created on: May 3, 2025
 *      Author: coder0908
 */

#include <stdio.h>
#include "platform/platform.h"
#include "platform/tim.h"
#include "drivers/pwm_driver/pwm_driver.h"
#include "lib/crsf/utils/crsf_queue.h"
#include "middleware/morb/morb.h"
#include "hardware/tranceiver/rp3/rp3_utils.h"
#include "actuator.h"



#define ELE_OFFSET (200)
#define LEFT_AIL_OFFSET (50)
#define RIGHT_AIL_OFFSET (130)

static struct drv_pwm  s_bldc_l_mot, s_bldc_r_mot, s_servo_l_ail, s_servo_r_ail, s_servo_ele, s_servo_rud, s_servo_lan, s_servo_l_flap, s_servo_r_flap;
static struct crsf_queue s_rp3_crsf_q;
static bool s_is_rp3_updated;

static float s_pitch_cmd;
static float s_roll_cmd;
static float s_yaw_cmd;
static bool s_is_attitude_controller_updated;


static void rp3_morb_callback(const struct topic_frame *topic_frame);
static void attitude_controller_morb_callback(const struct topic_frame *topic_frame);

//static uint16_t map(uint16_t x, uint16_t min_in, uint16_t max_in, uint16_t min_out, uint16_t max_out)
//{
//	return (x - min_in) * (max_out - min_out) / (max_in - min_in) + min_out;
//}

bool actuator_init()
{
	bool ret = false;

	drv_pwm_init(&s_bldc_l_mot, &htim1, TIM_CHANNEL_1, 172, 1810, 500, 1000);
	drv_pwm_init(&s_bldc_r_mot, &htim1, TIM_CHANNEL_2, 172, 1810, 500, 1000);
	drv_pwm_init(&s_servo_l_ail, &htim3, TIM_CHANNEL_4, 172, 1810, 500, 1000);
	drv_pwm_init(&s_servo_r_ail, &htim4, TIM_CHANNEL_2, 172, 1810, 500, 1000);
	drv_pwm_init(&s_servo_ele, &htim3, TIM_CHANNEL_2, 172, 1810, 350, 1150);
	drv_pwm_init(&s_servo_rud, &htim4, TIM_CHANNEL_1, 172, 1810, 250, 1250);
	drv_pwm_init(&s_servo_lan, &htim3, TIM_CHANNEL_1, 172, 1810, 500, 1000);
	drv_pwm_init(&s_servo_l_flap, &htim2, TIM_CHANNEL_2, 172, 1810, 250, 1250);
	drv_pwm_init(&s_servo_r_flap, &htim3, TIM_CHANNEL_3, 172, 1810, 250, 1250);

	ret = drv_pwm_write(&s_servo_l_ail, RP3_CH_VALUE_MID + LEFT_AIL_OFFSET);
	if (!ret) {
		return false;
	}
	ret = drv_pwm_write(&s_servo_r_ail, RP3_CH_VALUE_MID + RIGHT_AIL_OFFSET);
	if (!ret) {
		return false;
	}
	ret = drv_pwm_write(&s_servo_ele, RP3_CH_VALUE_MID + ELE_OFFSET);
	if (!ret) {
		return false;
	}
	ret = drv_pwm_write(&s_servo_rud, 992);
	if (!ret) {
		return false;
	}
	ret = drv_pwm_write(&s_bldc_l_mot, 172);
	if (!ret) {
		return false;
	}
	ret = drv_pwm_write(&s_bldc_r_mot, 172);
	if (!ret) {
		return false;
	}
	ret = drv_pwm_write(&s_servo_lan, RP3_CH_VALUE_MID);
	if (!ret) {
		return false;
	}
	ret = drv_pwm_write(&s_servo_l_flap, RP3_CH_VALUE_MID);
	if (!ret) {
		return false;
	}
	ret = drv_pwm_write(&s_servo_r_flap, RP3_CH_VALUE_MID);
	if (!ret) {
		return false;
	}

	if (!crsf_q_init(&s_rp3_crsf_q)) {
		return false;
	}

	if (!morb_subscribe(TOPIC_NUM_RP3, rp3_morb_callback)) {
		return false;
	}

	if (!morb_subscribe(TOPIC_NUM_ATTITUDE_CONTROLLER, attitude_controller_morb_callback)) {
		return false;
	}

	return ret;

}

static void rp3_morb_callback(const struct topic_frame *topic_frame)
{
	assert(topic_frame);

	if (topic_frame->topic_num != TOPIC_NUM_RP3) {
		return;
	}

	s_rp3_crsf_q = topic_frame->rp3.crsf_q;

	s_is_rp3_updated = true;


}

static void attitude_controller_morb_callback(const struct topic_frame *topic_frame)
{
	assert(topic_frame);

	if (topic_frame->topic_num != TOPIC_NUM_ATTITUDE_CONTROLLER) {
		return;
	}

	s_pitch_cmd = topic_frame->attitude_controller.pitch_cmd;
	s_roll_cmd = topic_frame->attitude_controller.roll_cmd;
	s_yaw_cmd = topic_frame->attitude_controller.yaw_cmd;

	s_is_attitude_controller_updated = true;
}



static void actuating_loop()
{
	static struct crsf_frame crsf_frame;	// for optimiztion
	static struct crsf_rc_channels rc_channels;
	bool is_rc_channels_updated = false;
//	static struct topic_actuator topic_actuator;

	if (s_is_rp3_updated) {
		while (crsf_q_deq(&s_rp3_crsf_q, &crsf_frame)) {
			if (!crsf_parse_rc_channels(&crsf_frame, &rc_channels)) {
				continue;
			}
			is_rc_channels_updated = true;
		}

		//printf("chan1: %u, chan2: %u, chan3: %u, chan4: %u, chan5: %u, chan6: %u, chan7: %u\n\r", rc_channels.chan1, rc_channels.chan2, rc_channels.chan3, rc_channels.chan4, rc_channels.chan5, rc_channels.chan6, rc_channels.chan7);
		s_is_rp3_updated = false;
	}

	if (is_rc_channels_updated) {
		drv_pwm_write(&s_bldc_l_mot, rp3_get_channel(RP3_CH_THR, &rc_channels));
		drv_pwm_write(&s_bldc_r_mot, rp3_get_channel(RP3_CH_THR, &rc_channels));
		drv_pwm_write(&s_servo_rud, rp3_get_channel(RP3_CH_RUD, &rc_channels));
		drv_pwm_write(&s_servo_lan, rp3_get_channel(RP3_CH_LAN, &rc_channels));

		if (rp3_get_channel(RP3_CH_ATTITUDE_CONTROL, &rc_channels) == RP3_CH_VALUE_MIN) {
			//attitude control off
			drv_pwm_write(&s_servo_ele, RP3_CH_VALUE_MAX - rp3_get_channel(RP3_CH_ELE, &rc_channels) + RP3_CH_VALUE_MIN + ELE_OFFSET);
			drv_pwm_write(&s_servo_l_ail, RP3_CH_VALUE_MAX - rp3_get_channel(RP3_CH_AIL, &rc_channels) + RP3_CH_VALUE_MIN + LEFT_AIL_OFFSET);
			drv_pwm_write(&s_servo_r_ail, RP3_CH_VALUE_MAX - rp3_get_channel(RP3_CH_AIL, &rc_channels) + RP3_CH_VALUE_MIN + RIGHT_AIL_OFFSET);
		}
	}

	if (s_is_attitude_controller_updated) {
		if (rp3_get_channel(RP3_CH_ATTITUDE_CONTROL, &rc_channels) == RP3_CH_VALUE_MAX) {
			//attitude control on
			drv_pwm_write(&s_servo_ele, RP3_CH_VALUE_MID + (long)s_pitch_cmd + ELE_OFFSET);
			drv_pwm_write(&s_servo_l_ail, RP3_CH_VALUE_MID - (long)s_roll_cmd + LEFT_AIL_OFFSET);
			drv_pwm_write(&s_servo_r_ail, RP3_CH_VALUE_MID - (long)s_roll_cmd + RIGHT_AIL_OFFSET);
			//printf("pitch input : %ld, roll_input : %ld\n\r", RP3_CH_VALUE_MID - (long)s_pitch_cmd,  RP3_CH_VALUE_MID - (long)s_roll_cmd);
		}

		s_is_attitude_controller_updated = false;
	}
//
//	if (is_rc_channels_updated || s_is_attitude_controller_updated) {
//		topic_actuator.throttle = rp3_get_channel(RP3_CH_THR, &rc_channels);
//		topic_actuator.l_aileron = rp3_get_channel(RP3_CH_AIL, &rc_channels);
//		topic_actuator.r_aileron = rp3_get_channel(RP3_CH_AIL, &rc_channels);
//		topic_actuator.elevator = rp3_get_channel(RP3_CH_ELE, &rc_channels);
//		topic_actuator.rudder = rp3_get_channel(RP3_CH_RUD, &rc_channels);
//		topic_actuator.steering = rp3_get_channel(RP3_CH_LAN, &rc_channels);
////		topic_actuator.l_flap = rp3_get_channel(RP3_CH_FLAP, &rc_channels);
////		topic_actuator.r_flap = rp3_get_channel(RP3_CH_FLAP, &rc_channels);
//
//
//		morb_publish(TOPIC_NUM_ACTUATOR, &topic_actuator);
//	}


}


void actuator_loop()
{


	actuating_loop();

}




