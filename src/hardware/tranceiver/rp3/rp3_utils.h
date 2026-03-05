/*
 * rp3_utils.h
 *
 *  Created on: 2026. 1. 30.
 *      Author: coder0908
 */


#pragma once

#include "lib/crsf/crsf.h"

#define RP3_CH_VALUE_MAX 1810
#define RP3_CH_VALUE_MIN 172
#define RP3_CH_VALUE_MID 922


enum rp3_channel {
	RP3_CH_THR = 0,
	RP3_CH_AIL,
	RP3_CH_ELE,
	RP3_CH_RUD,
	RP3_CH_ARM,
	RP3_CH_LAN,
	RP3_CH_ATTITUDE_CONTROL,
	RP3_CH_FLAP
};


uint16_t rp3_get_channel(enum rp3_channel rp3_ch, const struct crsf_rc_channels *rc_channels);
