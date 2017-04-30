#include "Fetch.h"
#include "Link.h"

using namespace std;

class PIDControl {

public:

  typedef double data_t;

  // 参数：PID三个K值，最大控制量max，时间间隔ts。可选的积分初始值和上次错误量。
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

  float32_t posThresholdInM = posThresholdInCm / 100;
  int elapsedTime = 0;

  // todo 设置PID参数
  PIDControl xController(1, 0, 0, 1, intervalInMs / 1000.0);
  PIDControl yController(1, 0, 0, 1, intervalInMs / 1000.0);
  PIDControl zController(1, 0, 0, 1, intervalInMs / 1000.0);

  while (true) {

    // 获取误差
    DJI::Vector3dData error = getTargetOffset();
    if (std::fabs(error.x) <= posThresholdInM && 
        std::fabs(error.y) <= posThresholdInM && 
        std::fabs(error.z) <= posThresholdInM)
      break;

    // 计算控制量
    ux = xController(error.x);
    uy = yController(error.y);
    uz = zController(error.z);

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

//! Helper Functions
/*! Very simple calculation of local NED offset between two pairs of GPS coordinates.
    Accurate when distances are small.
!*/
void localOffsetFromGpsOffset(DJI::Vector3dData& deltaNed,
    PositionData* target,
    PositionData* origin)
{
  double deltaLon = target->longitude - origin->longitude;
  double deltaLat = target->latitude - origin->latitude;
  deltaNed.x =  deltaLat * C_EARTH;
  deltaNed.y = deltaLon * C_EARTH * cos(target->latitude);
  deltaNed.z = target->height - origin->height;
}

// 此函数修改自sample：Liunx Blocking。删除了偏航角部分。
/*! Position Control. Allows you to set an offset from your current
    location. The aircraft will move to that position and stay there. 
    Typical use would be as a building block in an outer loop that does not
    require many fast changes, perhaps a few-waypoint trajectory. For smoother
    transition and response you should convert your trajectory to attitude 
    setpoints and use attitude control or convert to velocity setpoints
    and use velocity control. 
!*/
int moveByPositionOffset(CoreAPI* api, Flight* flight, float32_t xOffsetDesired, float32_t yOffsetDesired, float32_t zOffsetDesired, float32_t yawDesired ,  int timeoutInMs, float yawThresholdInDeg, float posThresholdInCm)
{

  static const int frequency = 40;
  static const int intervalInMs = 1000 / frequency;

  uint8_t flag = 0x91; //Position Control

  float32_t posThresholdInM = posThresholdInCm / 100;
  int elapsedTime = 0;

  // Get current poition
  // PositionData curPosition = api->getBroadcastData().pos;
  PositionData originPosition = api->getBroadcastData().pos;
  DJI::Vector3dData curLocalOffset;
  DJI::Vector3dData error;

  // todo 设置PID参数
  PIDControl xController(1, 0, 0, 1, intervalInMs / 1000.0);
  PIDControl yController(1, 0, 0, 1, intervalInMs / 1000.0);
  PIDControl zController(1, 0, 0, 1, intervalInMs / 1000.0);

  while (true) {

    // 获取误差
    PositionData curPosition = api->getBroadcastData().pos;
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
    error.x = xOffsetDesired - curLocalOffset.x;
    error.y = yOffsetDesired - curLocalOffset.y;
    error.z = zOffsetDesired - curLocalOffset.z;
    if (std::fabs(error.x) <= posThresholdInM && 
        std::fabs(error.y) <= posThresholdInM && 
        std::fabs(error.z) <= posThresholdInM)
      break;

    // 计算控制量
    ux = xController(error.x);
    uy = yController(error.y);
    uz = zController(error.z);

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
