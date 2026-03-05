/*
 * ntt24.c
 *
 *  Created on: 2025. 12. 3.
 *      Author: coder0908
 */

#include <assert.h>
#include "lib/math.h"
#include "platform/platform.h"
#include "platform/usart.h"
#include "middleware/morb/morb.h"
#include "rtt24.h"









void battery_morb_callback(const struct topic_frame *topic_frame);
void actuator_morb_callback(const struct topic_frame *topic_frame);
void neom8n_morb_callback(const struct topic_frame *topic_frame);
void icm20602_morb_callback(const struct topic_frame *topic_frame);
void attitude_estimator_morb_callback(const struct topic_frame *topic_frame);



static struct topic_battery s_topic_battery;
static bool s_is_battery_updated;

static struct topic_actuator s_topic_actuator;
static bool s_is_actuator_updated;

static struct topic_neom8n s_topic_neom8n;
static bool s_is_neom8n_updated;

static struct topic_icm20602 s_topic_icm20602;
static bool s_is_icm20602_updated;

static struct topic_attitude_estimator s_topic_attitude_estimator;
static bool s_is_attitude_estimator_updated;

bool rtt24_init()
{
	if (!morb_subscribe(TOPIC_NUM_BATTERY, battery_morb_callback)) {
		return false;
	}
	if (!morb_subscribe(TOPIC_NUM_ACTUATOR, actuator_morb_callback)) {
		return false;
	}
	if (!morb_subscribe(TOPIC_NUM_NEOM8N, neom8n_morb_callback)) {
		return false;
	}
	if (!morb_subscribe(TOPIC_NUM_ICM20602, icm20602_morb_callback)) {
		return false;
	}
	if (!morb_subscribe(TOPIC_NUM_ATTITUDE_ESTIMATOR, attitude_estimator_morb_callback)) {
		return false;
	}

	return true;
}

void battery_morb_callback(const struct topic_frame *topic_frame)
{
	assert(topic_frame);

	if (topic_frame->topic_num != TOPIC_NUM_BATTERY) {
		return;
	}

	s_topic_battery = topic_frame->battery;
	s_is_battery_updated = true;
}

void actuator_morb_callback(const struct topic_frame *topic_frame)
{
	assert(topic_frame);

	if (topic_frame->topic_num != TOPIC_NUM_ACTUATOR) {
		return;
	}

	s_topic_actuator = topic_frame->actuator;
	s_is_actuator_updated = true;
}

void neom8n_morb_callback(const struct topic_frame *topic_frame)
{
	assert(topic_frame);

	if (topic_frame->topic_num != TOPIC_NUM_NEOM8N) {
		return;
	}

	s_topic_neom8n = topic_frame->neom8n;
	s_is_neom8n_updated= true;

}

void icm20602_morb_callback(const struct topic_frame *topic_frame)
{
	assert(topic_frame);

	if (topic_frame->topic_num != TOPIC_NUM_ICM20602) {
		return;
	}

	s_topic_icm20602 = topic_frame->icm20602;
	s_is_icm20602_updated= true;


}

void attitude_estimator_morb_callback(const struct topic_frame *topic_frame)
{
	assert(topic_frame);

	if (topic_frame->topic_num != TOPIC_NUM_ATTITUDE_ESTIMATOR) {
		return;
	}

	s_topic_attitude_estimator = topic_frame->attitude_estimator;
	s_is_attitude_estimator_updated= true;

}




void rtt24_loop()
{
	struct crsf_frame crsf_frame;

	if (s_is_neom8n_updated) {
		struct ubx_frame ubx_frame;
		struct ubx_nav_pvt nav_pvt;

		while (ubx_q_deq(&(s_topic_neom8n.ubx_q), &ubx_frame)) {
			if (!ubx_parse_nav_pvt(&ubx_frame, &nav_pvt)) {
				continue;
			}
			crsf_framing_gps(&crsf_frame, nav_pvt.latitude_100ndeg, nav_pvt.longitude_100ndeg, (nav_pvt.groundspeed_mmps)/(25.0f/9.0f), (nav_pvt.heading_of_motion_10udeg)*100.0f, (nav_pvt.alti_ellipsoid_mm)/1000.0, nav_pvt.satellite);
			HAL_UART_Transmit(&huart1, crsf_frame.frame, crsf_get_frame_length(&crsf_frame), 20);

			crsf_framing_gps_ex(&crsf_frame, nav_pvt.gnss_fix_type, nav_pvt.northward_velocity_mmps/10.0f, nav_pvt.eastward_velocity_mmps/10.0f, nav_pvt.down_velocity_mmps/-10.0f, 0, 0, nav_pvt.alti_ellipsoid_mm/1000.0f, nav_pvt.horizontal_accuracy_mm/10.0f, nav_pvt.vertical_accuracy_mm/10.0f, 0, 0);
			HAL_UART_Transmit(&huart1, crsf_frame.frame, crsf_get_frame_length(&crsf_frame), 20);

			crsf_framing_gps_time(&crsf_frame, nav_pvt.year, nav_pvt.month, nav_pvt.day, nav_pvt.hour, nav_pvt.minuate, nav_pvt.second, 0);
			HAL_UART_Transmit(&huart1, crsf_frame.frame, crsf_get_frame_length(&crsf_frame), 20);
		}

		s_is_neom8n_updated = false;
	}

	if (s_is_icm20602_updated) {

		crsf_framing_rate(&crsf_frame, (int16_t)(s_topic_icm20602.x_gyro_dps*10.0f), (int16_t)(s_topic_icm20602.y_gyro_dps*10.0f), (int16_t)(s_topic_icm20602.z_gyro_dps*10.0f));
		HAL_UART_Transmit(&huart1, crsf_frame.frame, crsf_get_frame_length(&crsf_frame), 20);

		crsf_framing_accel(&crsf_frame, (int16_t)(s_topic_icm20602.x_accel_g*1000.0f), (int16_t)(s_topic_icm20602.y_accel_g*1000.0f), (int16_t)(s_topic_icm20602.z_accel_g*1000.0f));
		HAL_UART_Transmit(&huart1, crsf_frame.frame, crsf_get_frame_length(&crsf_frame), 20);

		s_is_icm20602_updated = false;
	}

	if (s_is_attitude_estimator_updated) {
		crsf_framing_attitude(&crsf_frame, (int16_t)(DEG_TO_RAD(s_topic_attitude_estimator.pitch_angle_deg)*10000.0f), (int16_t)(DEG_TO_RAD(s_topic_attitude_estimator.roll_angle_deg)*10000.0f), (int16_t)(DEG_TO_RAD(s_topic_attitude_estimator.heading_deg)*10000.0f));
		HAL_UART_Transmit(&huart1, crsf_frame.frame, crsf_get_frame_length(&crsf_frame), 20);

		s_is_attitude_estimator_updated = false;
	}

	if (s_is_battery_updated) {
		uint16_t tmp [2] = {s_topic_battery.batt_0 * 1000.0f, s_topic_battery.batt_1 * 1000.0f};
		crsf_framing_voltages(&crsf_frame, 128, tmp, 2);
		HAL_UART_Transmit(&huart1, crsf_frame.frame, crsf_get_frame_length(&crsf_frame), 20);
		s_is_battery_updated = false;
	}
}















