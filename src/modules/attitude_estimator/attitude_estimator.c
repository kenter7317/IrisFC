/*
 * attitude_estimator.c
 *
 *  Created on: Nov 19, 2025
 *      Author: coder0908
 */

#include <stdio.h>
#include <assert.h>
#include "platform/platform.h"
#include "middleware/morb/morb.h"
#include "lib/madgwick/madgwick.h"
#include "attitude_estimator.h"

static struct compfilter s_comfilter;

static float s_x_accel_g, s_y_accel_g, s_z_accel_g;
static float s_x_gyro_dps, s_y_gyro_dps, s_z_gyro_dps;
static bool s_is_icm20602_updated;

static float s_groundspeed_mps;

//static float map(float x, float min_in, float max_in, float min_out, float max_out)
//{
//	return (x - min_in) * (max_out - min_out) / (max_in - min_in) + min_out;
//}

static void icm20602_morb_callback(const struct topic_frame *topic_frame);
static void neom8n_morb_callback(const struct topic_frame *topic_frame);


bool attitude_estimator_init()
{
	compfilter_init(&s_comfilter,0.9);

	if (!morb_subscribe(TOPIC_NUM_ICM20602, icm20602_morb_callback)) {
		return false;
	}

	if (!morb_subscribe(TOPIC_NUM_NEOM8N, neom8n_morb_callback)) {

	}

	return true;
}

static void icm20602_morb_callback(const struct topic_frame *topic_frame)
{
	assert(topic_frame);

	if (topic_frame->topic_num != TOPIC_NUM_ICM20602) {
		return;

	}

	s_x_accel_g = topic_frame->icm20602.x_accel_g;
	s_y_accel_g = topic_frame->icm20602.y_accel_g;
	s_z_accel_g = topic_frame->icm20602.z_accel_g;

	s_x_gyro_dps = topic_frame->icm20602.x_gyro_dps;
	s_y_gyro_dps = topic_frame->icm20602.y_gyro_dps;
	s_z_gyro_dps = topic_frame->icm20602.z_gyro_dps;

	s_is_icm20602_updated = true;

}

static void neom8n_morb_callback(const struct topic_frame *topic_frame)
{
	assert(topic_frame);

	if (topic_frame->topic_num != TOPIC_NUM_NEOM8N) {
		return;
	}

	struct ubx_queue ubx_q = topic_frame->neom8n.ubx_q;
	struct ubx_frame ubx_frame ={0,};
	struct ubx_nav_pvt nav_pvt = {0,};

	while (ubx_q_deq(&ubx_q, &ubx_frame)) {
		if (!ubx_parse_nav_pvt(&ubx_frame, &nav_pvt)) {
			continue;
		}

		s_groundspeed_mps = (float)(nav_pvt.groundspeed_mmps) / 1000.0f;
	}


}

void attitude_estimator_loop()
{

	if (!s_is_icm20602_updated) {
		return;
	}

	s_is_icm20602_updated = false;



	madgwick_calc_angle(&s_comfilter, s_x_accel_g, s_y_accel_g, s_z_accel_g, s_x_gyro_dps, s_y_gyro_dps, s_z_gyro_dps, s_groundspeed_mps);


	struct topic_attitude_estimator topic_att_est = {0,};

//	printf("pitch_deg : %ld, roll_deg : %ld, yaw_deg : %ld\n\r", (long)s_comfilter.pitch_deg, (long)s_comfilter.roll_deg, (long)s_comfilter.yaw_deg);


	topic_att_est.pitch_angle_deg = s_comfilter.pitch_deg;
	topic_att_est.roll_angle_deg = s_comfilter.roll_deg;
	topic_att_est.heading_deg = s_comfilter.yaw_deg;

	morb_publish(TOPIC_NUM_ATTITUDE_ESTIMATOR, (void*)&topic_att_est);





}
