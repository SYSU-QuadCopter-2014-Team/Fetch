#include "PIDControl.h"

PIDControl::PIDControl(data_t kp, data_t ki, data_t kd, data_t max, data_t ts, data_t I, data_t err0)
	: kp(kp), ki(ki), kd(kd), max(max), ts(ts), I(I), err0(err0) {
}

PIDControl::data_t PIDControl::operator()(data_t err) {
	// 更新顺序问题？
	data_t u = kd * (err - err0) / ts + ki * I + kp * err;
	I += err * ts;
	err0 = err;
	if (u > max)
		u = max;
	else if (u < -max)
		u = -max;
	return u;
}
