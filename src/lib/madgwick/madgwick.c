
#include <math.h>
#include "platform/platform.h"
#include "madgwick.h"

// STM32 등 HAL 라이브러리 사용 시 정의, 아니면 직접 시간 함수 연결

#define DEG_TO_RAD (3.1415926535f / 180.0f)
#define RAD_TO_DEG (180.0f / 3.1415926535f)
#define GRAVITY 9.80665f

void compfilter_init(struct compfilter *cmpf, float beta) {
	cmpf->beta = beta;
	cmpf->q0 = 1.0f; cmpf->q1 = 0.0f; cmpf->q2 = 0.0f; cmpf->q3 = 0.0f;
	cmpf->prev_time_ms = 0;
}

// 역제곱근 최적화 함수
float invSqrt(float x) {
	return 1.0f / sqrtf(x);
}

bool madgwick_calc_angle(struct compfilter *cmpf, float ax, float ay, float az, float gx, float gy, float gz, float speed_mps) {
	uint32_t now = HAL_GetTick();
	if (cmpf->prev_time_ms == 0) {
		cmpf->prev_time_ms = now;
		return false;
	}
	float dt = (float)(now - cmpf->prev_time_ms) / 1000.0f;
	if (dt <= 0.0f || dt > 0.5f) { // 비정상적인 시간 간격 방지
		cmpf->prev_time_ms = now;
		return false;
	}
	cmpf->prev_time_ms = now;

	// 1. 자이로 단위 변환 (deg/s -> rad/s)
	float gx_rad = gx * DEG_TO_RAD;
	float gy_rad = gy * DEG_TO_RAD;
	float gz_rad = gz * DEG_TO_RAD;

	// 2. 고정익 원심력 보정 (Centripetal Compensation)
	// GPS 속도가 유효할 때, 회전 시 발생하는 가속도를 측정값에서 제거
	if (speed_mps > 2.0f) {
		// 선회 시 횡가속도(ay)와 종가속도(az) 보정
		ay -= (gz_rad * speed_mps) / GRAVITY;
		az += (gy_rad * speed_mps) / GRAVITY;
	}

	// 3. 가속도 정규화
	float recipNorm = invSqrt(ax*ax + ay*ay + az*az);
	ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

	// 4. Madgwick 알고리즘 (경사 하강법 단계)
	float q0 = cmpf->q0, q1 = cmpf->q1, q2 = cmpf->q2, q3 = cmpf->q3;
	float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
	float _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;

	// Objective function f(q, a) = q*g*q' - a
	float f1 = _2q1 * q3 - _2q0 * q2 - ax;
	float f2 = _2q0 * q1 + _2q2 * q3 - ay;
	float f3 = 1.0f - _2q1 * q1 - _2q2 * q2 - az;

	// Gradient 계산 (Jacobian J^T * f)
	float s0 = -_2q2 * f1 + _2q1 * f2;
	float s1 =  _2q3 * f1 + _2q0 * f2 - _4q1 * f3;
	float s2 = -_2q0 * f1 + _2q3 * f2 - _4q2 * f3;
	float s3 =  _2q1 * f1 + _2q2 * f2;

	// Gradient 정규화
	recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
	s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

	// 5. 변화량 적분 (자이로 + 가속도계 보정)
	float qDot0 = 0.5f * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad) - cmpf->beta * s0;
	float qDot1 = 0.5f * ( q0 * gx_rad + q2 * gz_rad - q3 * gy_rad) - cmpf->beta * s1;
	float qDot2 = 0.5f * ( q0 * gy_rad - q1 * gz_rad + q3 * gx_rad) - cmpf->beta * s2;
	float qDot3 = 0.5f * ( q0 * gz_rad + q1 * gy_rad - q2 * gx_rad) - cmpf->beta * s3;

	q0 += qDot0 * dt; q1 += qDot1 * dt; q2 += qDot2 * dt; q3 += qDot3 * dt;

	// 쿼터니언 최종 정규화
	recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	cmpf->q0 = q0 * recipNorm; cmpf->q1 = q1 * recipNorm;
	cmpf->q2 = q2 * recipNorm; cmpf->q3 = q3 * recipNorm;

	// 6. 오일러 각 추출 (NED 좌표계 기준)
	q0 = cmpf->q0; q1 = cmpf->q1; q2 = cmpf->q2; q3 = cmpf->q3;

	cmpf->roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;

	float sinp = 2.0f * (q0 * q2 - q3 * q1);
	if (fabsf(sinp) >= 1.0f) cmpf->pitch_deg = copysignf(90.0f, sinp);
	else cmpf->pitch_deg = asinf(sinp) * RAD_TO_DEG;

	cmpf->yaw_deg = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;

	return true;
}
