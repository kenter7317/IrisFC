#ifndef COMPFILTER_H_
#define COMPFILTER_H_

#include <stdbool.h>
#include <stdint.h>

struct compfilter {
	float beta;
	uint32_t prev_time_ms;

	float pitch_deg;
	float roll_deg;
	float yaw_deg;

	float q0, q1, q2, q3;
};

void compfilter_init(struct compfilter *cmpf, float beta);


bool madgwick_calc_angle(struct compfilter *cmpf, float ax, float ay, float az, float gx, float gy, float gz, float speed_mps);

#endif
