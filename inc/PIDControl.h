#ifndef PIDCONTROL_H
#define PIDCONTROL_H

class PIDControl {

public:

  typedef double data_t;

  // 参数：PID三个K值，最大控制量max，时间间隔ts。可选的积分初始值和上次误差量。
  PIDControl(data_t kp, data_t ki, data_t kd, data_t max, data_t ts, data_t I = 0, data_t err0 = 0);

  // 每个时间间隔，传入误差值，返回控制量
  data_t operator()(data_t err);

private:

  const data_t kd;
  const data_t ki;
  const data_t kp;
  const data_t max;
  const data_t ts;  // 采样时间（秒）

  data_t I;
  data_t err0;

};

#endif // PIDCONTROL_H
