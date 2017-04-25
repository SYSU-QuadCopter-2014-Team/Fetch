#include "Fetch.h"
#include "Link.h"

using namespace std;

class PIDControl {

public:

  typedef double data_t;

  PIDControl(data_t kp, data_t ki, data_t kd, data_t max, data_t ts, data_t I = 0, data_t err0 = 0)
    : kp(kp), ki(ki), kd(kd), max(max), ts(ts), I(I), err0(err0) {
  }

  data_t operator()(data_t err) {
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

private:

  const data_t kd;
  const data_t ki;
  const data_t kp;
  const data_t max;
  const data_t ts;  // 采样时间（秒）

  data_t I;
  data_t err0;

};

int flyToTarget(CoreAPI* api, Flight* flight, int timeoutInMs, float posThresholdInCm) {

  static const int frequency = 40;
  static const int intervalInMs = 1000 / frequency;

  const uint8_t flag = 0x49;  //Velocity Control

  int elapsedTime = 0;

  // todo 设置PID参数
  PIDControl xController(1, 0, 0, 1, intervalInMs / 1000.0);
  PIDControl yController(1, 0, 0, 1, intervalInMs / 1000.0);
  PIDControl zController(1, 0, 0, 1, intervalInMs / 1000.0);

  while (true) {

    // 获取误差
    DJI::Vector3dData offset = getTargetOffset();
    if (offset.x <= posThresholdInCm && offset.y <= posThresholdInCm && offset.z <= posThresholdInCm)
      break;

    // 计算控制量
    ux = xController(offset.x);
    uy = yController(offset.y);
    uz = zController(offset.z);

    // 发送控制
    flight->setMovementControl(flag, ux, uy, uz, 0);

    usleep(intervalInMs * 1000);
    elapsedTime += intervalInMs;
    if (elapsedTime >= timeoutInMs) {
      std::cout << "Timed out \n";
      return 1;
    }

  }

  return 0;
}
