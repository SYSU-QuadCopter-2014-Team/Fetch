#include <fstream>
#include <iostream>
#include <cmath>

#include "PIDControl.h"

PIDControl::PIDControl(data_t kp, data_t ki, data_t kd, data_t max, data_t ts, data_t I, data_t err0)
	: kp(kp), ki(ki), kd(kd), max(max), ts(ts), I(I), err0(err0), first(true) {
}

PIDControl::PIDControl(const char *filename, data_t max, data_t ts, data_t I, data_t err0)
	: max(max), ts(ts), I(I), err0(err0), first(true) {
	std::ifstream ifs(filename);
	ifs >> kp >> ki >> kd;
	std::cout << "k read from " << filename << " :\t" << kp << "\t" << ki << "\t" << kd << "\n";
	ifs.close();
}

PIDControl::data_t PIDControl::operator()(data_t err) {
	if (first) {
		first = false;
		err0 = err;
	}
	// 更新顺序问题？
	data_t u = kd * (err - err0) / ts + ki * I + kp * err;
	if (std::fabs(err0) < std::fabs(err) - 0.01)
		I += err * ts;
	err0 = err;
	if (u > max)
		u = max;
	else if (u < -max)
		u = -max;
	return u;
}
