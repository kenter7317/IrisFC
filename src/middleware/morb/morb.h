/*
 * morb.h
 *
 *  Created on: 2026. 1. 27.
 *      Author: coder0908
 */

#pragma once

#include <stdbool.h>
#include "middleware/topic/topics.h"



typedef void (*morb_callback_fp)(const struct topic_frame *frame);

bool morb_subscribe(enum topic_number topic_num, morb_callback_fp callback);
bool morb_unsubscribe(enum topic_number topic_num, morb_callback_fp callback);

bool morb_publish(enum topic_number topic_num, void *topic);

