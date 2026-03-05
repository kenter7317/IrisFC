/*
 * rp3_utils.c
 *
 *  Created on: 2026. 1. 30.
 *      Author: coder0908
 */


#include "rp3_utils.h"



uint16_t rp3_get_channel(enum rp3_channel rp3_ch, const struct crsf_rc_channels *rc_channels)
{
	switch (rp3_ch) {
	case RP3_CH_THR:
		return rc_channels->chan1;
	case RP3_CH_AIL:
		return rc_channels->chan2;
	case RP3_CH_ELE:
		return rc_channels->chan3;
	case RP3_CH_RUD:
		return rc_channels->chan4;
	case RP3_CH_ARM:
		return rc_channels->chan5;
	case RP3_CH_LAN:
		return rc_channels->chan6;
	case RP3_CH_ATTITUDE_CONTROL:
		return rc_channels->chan7;
	case RP3_CH_FLAP:
		return rc_channels->chan8;
	}

	return 0;
}
