/*
 * topic_list.h
 *
 *  Created on: 2026. 1. 27.
 *      Author: coder0908
 */

#include "lib/ubx/utils/ubx_queue.h"
#include "lib/crsf/utils/crsf_queue.h"

#pragma once

#define TOPIC_CNT	8

enum topic_number {
	TOPIC_NUM_ICM20602 = 0,
	TOPIC_NUM_NEOM8N,
	TOPIC_NUM_RP3,
	TOPIC_NUM_BATTERY,
	TOPIC_NUM_ACTUATOR,
	TOPIC_NUM_HMC5883L,
	TOPIC_NUM_ATTITUDE_ESTIMATOR,
	TOPIC_NUM_ATTITUDE_CONTROLLER
};

struct topic_icm20602 {
	float x_accel_g;
	float y_accel_g;
	float z_accel_g;

	float x_gyro_dps;
	float y_gyro_dps;
	float z_gyro_dps;
};

struct topic_neom8n {
	struct ubx_queue ubx_q;
};

struct topic_rp3 {
	struct crsf_queue crsf_q;
};

struct topic_battery {
	float batt_0;
	float batt_1;
};

// -1000 ~ 1000
struct topic_actuator {
	uint16_t throttle;
	uint16_t l_aileron;
	uint16_t r_aileron;
	uint16_t elevator;
	uint16_t rudder;
	uint16_t steering;
	uint16_t l_flap;
	uint16_t r_flap;
};

struct topic_hmc5883l {

};

struct topic_attitude_estimator {
	float pitch_angle_deg;
	float roll_angle_deg;
	float heading_deg;
};

struct topic_attitude_controller {
	float pitch_cmd;
	float roll_cmd;
	float yaw_cmd;
};


struct topic_frame {
	uint32_t tick;
	enum topic_number topic_num;

	union {
		struct topic_icm20602 icm20602;
		struct topic_neom8n neom8n;
		struct topic_rp3 rp3;
		struct topic_battery battery;
		struct topic_actuator actuator;
		struct topic_hmc5883l hmc5883l;
		struct topic_attitude_estimator attitude_estimator;
		struct topic_attitude_controller attitude_controller;
	};
};




