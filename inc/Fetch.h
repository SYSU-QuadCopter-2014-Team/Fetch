/*! @brief
 *  抓取的各步骤功能函数
 * */

#ifndef FETCH_H
#define FETCH_H

#include <DJI_Type.h>
#include <DJI_API.h>
#include <DJI_Flight.h>

#include <string>

#define Pi 3.1415926

using namespace DJI::onboardSDK;

#define create_declaration(type, name) \
public: void set_##name(type name); \
private: type name;

#define create_definition(classname, type, name) \
void classname::set_##name(type value) { name = value; }

// 暂缺鲁棒性：需要停止任务后才能进行新的任务
class Fetch {

public:

	Fetch(float timeoutInS = 30.0f,
	      float posThresholdInCm = 10.0f, float maxSpeedInM = 0.1f,
	      float yawThresholdInDeg = 10.0f, float maxYawSpeedInDeg = 10.0f);
	void setApi(CoreAPI * value);
	void setFlight(Flight * value);

	// 任务

	void start_position_control(float x, float y, float z, float yaw);
	void start_approaching();

	// 停止当前任务
	void stop();

	// 任务参数
	create_declaration(float, timeoutInS);
	create_declaration(float, posThresholdInCm);
	create_declaration(float, maxSpeedInM);
	create_declaration(float, yawThresholdInDeg);
	create_declaration(float, maxYawSpeedInDeg);

private:

	static const int frequency = 40;
	static const float intervalInS = 1.0f / frequency;
	static const int intervalInMs = 1000 * intervalInS;

	Flight *flight;
	CoreAPI *api;

	BroadcastData bd;

	//////////////////////////
	// 线程

	bool stop_task;  // 任务停止信号
	float ex, ey, ez, eyaw;  // 保存线程的计算结果

	// position_control任务

	void moveByPositionOffset(float x, float y, float z, float yaw);

	// approaching任务

	void runKCF();
	void approaching();

	//////////////////////////

};

//////////////////////////
// copy from LinuxFlight.h

#define C_EARTH (double) 6378137.0
#define DEG2RAD 0.01745329252

//////////////////////////
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

//////////////////////////

#endif // FETCH_H
