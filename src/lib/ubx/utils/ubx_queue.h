/*
 * ubx_queue.h
 *
 *  Created on: 2025. 12. 10.
 *      Author: coder0908
 */

#pragma once

#include "../ubx.h"

#define UBX_FRAMESLEN_MAX	2

struct ubx_queue {
	struct ubx_frame frames[UBX_FRAMESLEN_MAX];
	uint8_t head;
	uint8_t len;
};

bool ubx_q_init(struct ubx_queue *queue);
bool ubx_q_enq(struct ubx_queue *queue, const struct ubx_frame *frame);
bool ubx_q_deq(struct ubx_queue *queue, struct ubx_frame *frame);
bool ubx_q_clear(struct ubx_queue *queue);
void ubx_q_parse_frames(struct ubx_queue *queue, uint8_t *buf, uint16_t buf_len, uint16_t *read_len);
