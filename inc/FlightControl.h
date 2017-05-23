/*! @brief
 *  对x,y,z,yaw的PID控制集合
 * */

#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H

#include <DJI_Type.h>
#include <DJI_API.h>
#include <DJI_Flight.h>

#include "Logger.h"
#include "PIDControl.h"

using namespace DJI::onboardSDK;

class FlightControl {

public:

	typedef float data_t;

	// 参数：飞行API指针flight，控制flag，最大速度控制量maxV，最大偏航角控制量maxYaw，时间间隔ts。
	FlightControl(Flight *flight, uint8_t flag, data_t maxV, data_t maxYaw, data_t ts);

	// 传入误差进行控制
	void control(data_t ex, data_t ey, data_t ez, data_t eyaw);

	void stop();

private:

	void send_control(uint8_t flag, float x, float y, float z, float yaw);

	const uint8_t flag;  // Control mode byte

	const Flight *flight;

	PIDControl xController;
	PIDControl yController;
	PIDControl zController;
	PIDControl yawController;

	Logger logger;

};

#endif  // FLIGHTCONTROL_H

