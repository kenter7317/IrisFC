/*
 * attitude_controller.c
 *
 *  Created on: 2026. 1. 28.
 *      Author: coder0908
 */

#include <stdio.h>
#include "middleware/morb/morb.h"
#include "attitude_controller.h"
#include "lib/pid/pid.h"

static struct cascade_pid s_roll_cpid;
static struct cascade_pid s_pitch_cpid;

static float s_pitch_angle_deg;
static float s_roll_angle_deg;

static float s_y_gyro_dps;
static float s_x_gyro_dps;

static bool s_is_attitude_estimator_updated;

static struct crsf_rc_channels s_attitude_controller_rc_channels;

static float map(float x, float min_in, float max_in, float min_out, float max_out)
{
	return (x - min_in) * (max_out - min_out) / (max_in - min_in) + min_out;
}

static void attitude_estimator_morb_callabck(const struct topic_frame *topic_frame);
static void rp3_morb_callback(const struct topic_frame *topic_frame);
static void icm20602_morb_callabck(const struct topic_frame *topic_frame);


bool attitude_controller_init()
{
	if (!cascade_pid_init(&s_roll_cpid, 9, 2, 0.08, 0, -90, 90, 0.9, 0.05, 0.0001, 0, -80, 80)) {
		return false;
	}
	if (!cascade_pid_init(&s_pitch_cpid, 9, 2, 0.08, 0, -90, 90, 0.9, 0.05, 0.0001, 0, -80, 80)) {
		return false;
	}

	if (!morb_subscribe(TOPIC_NUM_ATTITUDE_ESTIMATOR, attitude_estimator_morb_callabck)) {
		return false;
	}

	if (!morb_subscribe(TOPIC_NUM_RP3, rp3_morb_callback)) {
		return false;
	}

	if (!morb_subscribe(TOPIC_NUM_ICM20602, icm20602_morb_callabck)) {
		return false;
	}

	return true;

}

static void icm20602_morb_callabck(const struct topic_frame *topic_frame)
{
	if (topic_frame->topic_num != TOPIC_NUM_ICM20602) {
		return;
	}

	s_y_gyro_dps = topic_frame->icm20602.y_gyro_dps;
	s_x_gyro_dps = topic_frame->icm20602.x_gyro_dps;
}


static void attitude_estimator_morb_callabck(const struct topic_frame *topic_frame)
{
	if (topic_frame->topic_num != TOPIC_NUM_ATTITUDE_ESTIMATOR) {
		return;
	}

	s_is_attitude_estimator_updated = true;

	s_pitch_angle_deg = topic_frame->attitude_estimator.pitch_angle_deg;
	s_roll_angle_deg = topic_frame->attitude_estimator.roll_angle_deg;

}

static void rp3_morb_callback(const struct topic_frame *topic_frame)
{


	struct crsf_queue crsf_q = topic_frame->rp3.crsf_q;
	struct crsf_frame crsf_frame = {0,};

	while (crsf_q_deq(&crsf_q, &crsf_frame)) {
		if (!crsf_parse_rc_channels(&crsf_frame, &s_attitude_controller_rc_channels)) {
			continue;
		}
	}
}
//191 on , 1792 off
void attitude_controller_loop()
{
	if (s_attitude_controller_rc_channels.chan7 == 172) {
		//attitude control off
		cascade_pid_reset(&s_pitch_cpid);
		cascade_pid_reset(&s_roll_cpid);
		return;
	} else if (s_attitude_controller_rc_channels.chan7 != 1810) {
		return;
	}



	if (!s_is_attitude_estimator_updated) {
		return;
	}
	s_is_attitude_estimator_updated = false;

	float pitch_set_state = map(s_attitude_controller_rc_channels.chan3, 172, 1810, 40, -40);
	float roll_set_state = map(s_attitude_controller_rc_channels.chan2, 172, 1810, -40, 40);

	cascade_pid_calc(&s_pitch_cpid, pitch_set_state, s_pitch_angle_deg, s_y_gyro_dps);
	cascade_pid_calc(&s_roll_cpid, roll_set_state, s_roll_angle_deg, s_x_gyro_dps);

//	printf("pitch : %ld,  roll : %ld\n\r", (long)(s_pitch_cpid.inner.pid_result), (long)(s_roll_cpid.inner.pid_result));
//	printf("outter error_sum : %ld, current error : %ld\n\r", (long)(s_pitch_cpid.outter.error_sum), (long)(pitch_set_state - s_pitch_angle_deg));
	struct topic_attitude_controller topic_att_ctrler = {0,};

	topic_att_ctrler.pitch_cmd = s_pitch_cpid.inner.pid_result;
	topic_att_ctrler.roll_cmd = s_roll_cpid.inner.pid_result;

	morb_publish(TOPIC_NUM_ATTITUDE_CONTROLLER, (void*)&topic_att_ctrler);
}







