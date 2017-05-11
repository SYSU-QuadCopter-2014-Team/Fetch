/*! @brief
 *  抓取的各步骤功能函数
 * */

#ifndef FETCH_H
#define FETCH_H

#include <DJI_Type.h>
#include <DJI_API.h>
#include <DJI_Flight.h>

#define Pi 3.1415926

using namespace DJI::onboardSDK;

// 暂缺鲁棒性：需要停止任务后才能进行新的任务
class Fetch {

public:

	Fetch(float timeoutInS = 30.0f, float posThresholdInCm = 10.0f, float maxSpeedInM = 0.1f);
	void setApi(CoreAPI * value);
	void setFlight(Flight * value);

	// 任务

	void start_position_control(float x, float y, float z, float yaw);
	void start_approaching();

	// 停止当前任务
	void stop();

	// 设置任务参数
	void setParameters(float timeoutInS, float posThresholdInCm, float maxSpeedInM);

private:

	static const int frequency = 40;
	static const int intervalInMs = 1000.0f / frequency;

	Flight *flight;
	CoreAPI *api;

	BroadcastData bd;

	// 任务参数

	float timeoutInS;
	float posThresholdInCm;
	float maxSpeedInM;

	// 任务停止信号
	bool stop_task;

	// record线程

	void record();

	bool record_flag;  // 置信号为1以进行一次记录

	// 保存计算结果用以记录
	float ux, uy, uz;
	float ex, ey, ez;

	//////////////////////////
	// position_control任务
	//////////////////////////

	void moveByPositionOffset(float x, float y, float z, float yaw);

	//////////////////////////
	// approaching任务
	//////////////////////////

	// 线程
	void counter();
	void approaching();

	bool calc_flag;  // 置信号为1以进行一次计算

	//////////////////////////

	// helpers
	void flight_control(uint8_t flag, float x, float y, float z, float yaw);

};

// copy from LinuxFlight.h

#define C_EARTH (double) 6378137.0
#define DEG2RAD 0.01745329252

// copy from DJICommonType.h

//! @note This struct will replace SpaceVector in a future release.
//! Eigen-like naming convention
typedef struct Vector3dData
{
  double x;
  double y;
  double z;
} Vector3dData;

typedef double Angle;

typedef struct EulerAngle
{
  Angle yaw;
  Angle roll;
  Angle pitch;
} EulerAngle;

#endif // FETCH_H
