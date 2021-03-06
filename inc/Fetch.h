/*! @brief
 *  抓取的各步骤功能函数
 * */

#ifndef FETCH_H
#define FETCH_H

#include <DJI_Type.h>
#include <DJI_API.h>
#include <DJI_Camera.h>
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

	Fetch(float timeoutInS = 180.0f,
	      float posThresholdInCm = 5.0f, float maxSpeedInM = 0.3f,
	      float yawThresholdInDeg = 10.0f, float maxYawSpeedInDeg = 10.0f,
	      float frequency = 40.0f);

	// 任务

	void start_position_control(float x, float y, float z, float yaw);
	void start_height_control(float z);
	void start_approaching();
	void start_auto_fetch();

	// 机械爪命令
	void fetch();
	void release();

	// 停止当前任务
	void stop();

	create_declaration(Flight *, flight);
	create_declaration(CoreAPI *, api);
	create_declaration(Camera *, camera);

	// 任务参数
	create_declaration(float, timeoutInS);
	create_declaration(float, posThresholdInCm);
	create_declaration(float, maxSpeedInM);
	create_declaration(float, yawThresholdInDeg);
	create_declaration(float, maxYawSpeedInDeg);
	create_declaration(float, frequency);
	create_declaration(float, hoverHeight);
	create_declaration(float, fetchHeight);
	create_declaration(float, gimbalOffsetX);

private:

	BroadcastData bd;

	//////////////////////////
	// 线程

	bool stop_task;  // 任务停止信号
	bool finish_approach;

	// KCF
	void runKCF();
	bool target_found;
	float ex, ey, ez, eyaw;  // 保存线程的计算结果
	float tex, tey;  // 目标在图像中心的偏移量

	void moveByPositionOffset(float x, float y, float z, float yaw);  // position_control任务
	void moveByHeightOffset(float z);  // height_control任务，目前使用最大速度的一半
	void approaching();
	void auto_fetch();

	//////////////////////////

	// 发送控制
	void flight_control(uint8_t flag, float x, float y, float z, float yaw);

	void flight_control_sh(float set_height);

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
