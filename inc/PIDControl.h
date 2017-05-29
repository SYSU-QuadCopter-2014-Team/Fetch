#ifndef PIDCONTROL_H
#define PIDCONTROL_H

class PIDControl {

public:

	typedef float data_t;

	// 参数：PID三个K值，或者三个值的配置文件路径；最大控制量max，时间间隔ts。可选的上次误差量和积分初始值，没有误差值则会使第一次微分为0。
	PIDControl(data_t kp, data_t ki, data_t kd, data_t max, data_t ts, data_t err0, data_t I = 0);
	PIDControl(const char *filename, data_t max, data_t ts, data_t err0, data_t I = 0);
	PIDControl(data_t kp, data_t ki, data_t kd, data_t max, data_t ts);
	PIDControl(const char *filename, data_t max, data_t ts);

	// 每个时间间隔，传入误差值，返回控制量
	data_t operator()(data_t err);

private:

	data_t kd;
	data_t ki;
	data_t kp;
	const data_t max;
	const data_t ts;  // 采样时间（秒）

	data_t I;
	data_t err0;

	bool first;  // 用于第一次控制时使微分控制为0

};

#endif // PIDCONTROL_H
