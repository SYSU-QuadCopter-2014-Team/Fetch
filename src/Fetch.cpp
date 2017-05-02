#include "Fetch.h"
#include "PIDControl.h"
#include "Link.h"

using namespace std;

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

/*! 修改自sample：Liunx Blocking
!*/
int moveByPositionOffset(CoreAPI* api, Flight* flight, float32_t xOffsetDesired, float32_t yOffsetDesired, float32_t zOffsetDesired, float32_t yawDesired, int timeoutInMs, float yawThresholdInDeg, float posThresholdInCm)
{

  static const int frequency = 40;
  static const int intervalInMs = 1000 / frequency;

  const uint8_t flag = 0x49;  //Velocity Control

  double yawDesiredRad = DEG2RAD*yawDesired;
  double yawThresholdInRad = DEG2RAD*yawThresholdInDeg;
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
  PIDControl yawController(1, 0, 0, 1, intervalInMs / 1000.0);

  while (true) {

    // 获取误差
    PositionData curPosition = api->getBroadcastData().pos;
    DJI::EulerAngle curEuler = Flight::toEulerAngle(api->getBroadcastData().q);

    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
    error.x = xOffsetDesired - curLocalOffset.x;
    error.y = yOffsetDesired - curLocalOffset.y;
    error.z = zOffsetDesired - curLocalOffset.z;
    double errorYaw = yawDesiredRad - curEuler.yaw
    if (std::fabs(error.x) <= posThresholdInM &&
        std::fabs(error.y) <= posThresholdInM &&
        std::fabs(error.z) <= posThresholdInM &&
        std::fabs(errorYaw) <= yawThresholdInRad)
      break;

    // 计算控制量
    ux = xController(error.x);
    uy = yController(error.y);
    uz = zController(error.z);
    uyaw = yawController(errorYaw);

    // 发送控制
    flight->setMovementControl(flag, ux, uy, uz, uyaw);

    usleep(intervalInMs * 1000);
    elapsedTime += intervalInMs;
    if (elapsedTime >= timeoutInMs) {
      std::cout << "Timed out \n";
      return 1;
    }

  }

  return 0;
}
