/*! @brief
 *  抓取的各步骤功能函数
 * */

#ifndef FETCH_H
#define FETCH_H

#include <DJI_Type.h>
#include <DJICommonType.h>
#include <DJI_API.h>
#include <DJI_Flight.h>

// 暂缺鲁棒性：需要停止任务后才能进行新的任务
class Fetch {

public:

	Fetch(int timeoutInS = 30, float posThresholdInCm = 10.0f, float maxSpeedInM = 0.1f);
	void setApi(CoreAPI * value);
	void setFlight(Flight * value);

	// 任务

	void start_position_control(float x, float y, float z, float yaw);
	void start_approaching();

	// 停止当前任务
	void stop();

	// 设置任务参数
	void setParameters(int timeoutInS, float posThresholdInCm, float maxSpeedInM);

private:

	static const int frequency = 40;
	static const int intervalInMs = 1000.0f / frequency;

	Flight *flight;
	CoreAPI *api;

	// 任务参数

	int timeoutInS;
	float posThresholdInCm;
	float maxSpeedInM;

	// 任务停止信号
	bool stop_task;

	// position_control任务

	void moveByPositionOffset(float x, float y, float z, float yaw);

	// approaching任务

	// 分为3个线程
	void record();
	void counter();
	void approaching();

	bool record_flag;  // 置信号为1以进行一次记录
	bool calc_flag;  // 置信号为1以进行一次计算

	// 保存计算结果用以记录
	float ux, uy, uz;
	float ex, ey, ez;

	// helpers
	void flight_control(uint8_t flag, float x, float y, float z, float yaw);

};

#endif // FETCH_H
