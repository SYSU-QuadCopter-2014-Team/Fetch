#include <time.h>

#include <opencv2/opencv.hpp>

#include <boost/thread.hpp>

#include "Fetch.h"

#include "FlightControl.h"
#include "PIDControl.h"
#include "Logger.h"

#include "TrackWithDistance.h"
#include "estimatePos.h"

using namespace std;

Fetch::Fetch(float timeoutInS, float posThresholdInCm, float maxSpeedInM, float yawThresholdInDeg, float maxYawSpeedInDeg)
	: timeoutInS(timeoutInS), posThresholdInCm(posThresholdInCm), maxSpeedInM(maxSpeedInM), yawThresholdInDeg(yawThresholdInDeg), maxYawSpeedInDeg(maxYawSpeedInDeg)
{
	stop_task = true;
}

void Fetch::setApi(CoreAPI * value) {api = value;}

void Fetch::setFlight(Flight * value) {flight = value;}

void Fetch::start_position_control(float x, float y, float z, float yaw) {
	stop_task = false;
	boost::thread move(boost::bind(&Fetch::moveByPositionOffset, this, x, y, z, yaw));
}

/*member function for tracknig*/
void Fetch::start_approaching()
{
	stop_task = false;
	boost::thread kcf(boost::bind(&Fetch::runKCF, this));
	boost::thread approach(boost::bind(&Fetch::approaching, this));
	//track.detach();
}

void Fetch::stop() {
	stop_task = true;
}

create_definition(Fetch, float, timeoutInS);
create_definition(Fetch, float, posThresholdInCm);
create_definition(Fetch, float, maxSpeedInM);
create_definition(Fetch, float, yawThresholdInDeg);
create_definition(Fetch, float, maxYawSpeedInDeg);

//! Helper Functions
/*! Very simple calculation of local NED offset between two pairs of GPS coordinates.
    Accurate when distances are small.
!*/
void localOffsetFromGpsOffset(Vector3dData& deltaNed,
                              PositionData* target,
                              PositionData* origin)
{
	double deltaLon = target->longitude - origin->longitude;
	double deltaLat = target->latitude - origin->latitude;
	deltaNed.x =  deltaLat * C_EARTH;
	deltaNed.y = deltaLon * C_EARTH * cos(target->latitude);
	deltaNed.z = target->height - origin->height;
}

EulerAngle toEulerAngle(QuaternionData quaternionData)
{
	EulerAngle ans;

	double q2sqr = quaternionData.q2 * quaternionData.q2;
	double t0 = -2.0 * (q2sqr + quaternionData.q3 * quaternionData.q3) + 1.0;
	double t1 = +2.0 * (quaternionData.q1 * quaternionData.q2 + quaternionData.q0 * quaternionData.q3);
	double t2 = -2.0 * (quaternionData.q1 * quaternionData.q3 - quaternionData.q0 * quaternionData.q2);
	double t3 = +2.0 * (quaternionData.q2 * quaternionData.q3 + quaternionData.q0 * quaternionData.q1);
	double t4 = -2.0 * (quaternionData.q1 * quaternionData.q1 + q2sqr) + 1.0;

	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;

	ans.pitch = asin(t2);
	ans.roll = atan2(t3, t4);
	ans.yaw = atan2(t1, t0);

	return ans;
}

float angle(float target, float current) {
	float result = target - current;
	if (result < -180.0f)
		result += 360.0f;
	else if (result > 180.0f)
		result -= 360.0f;
	return result;
}

// 修改自sample：Liunx Blocking
// todo finish the record about yaw
void Fetch::moveByPositionOffset(float x, float y, float z, float yaw) {

	const float posThresholdInM = posThresholdInCm / 100;

	PositionData originPosition = api->getBroadcastData().pos;
	const float targetYaw = angle(toEulerAngle(api->getBroadcastData().q).yaw / DEG2RAD, -yaw);  // 相当于做加法

	Vector3dData curLocalOffset;
	int elapsedTime = 0;
	float ex, ey, ez, eyaw;

	FlightControl controller(flight, maxSpeedInM, maxYawSpeedInDeg, intervalInMs / 1000.0);
	Logger eLog("error.txt");
	Logger vLog("velocity.txt");

	while (true) {

		if (stop_task) break;

		// 计算误差
		bd = api->getBroadcastData();
		PositionData curPosition = bd.pos;
		EulerAngle curEuler = toEulerAngle(bd.q);
		localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);

		ex = x - curLocalOffset.x;
		ey = y - curLocalOffset.y;
		ez = z - curLocalOffset.z;
		eyaw = angle(targetYaw, curEuler.yaw / DEG2RAD);

		printf("err: %6.2f %6.2f %6.2f %6.2f\n", ex, ey, ez, eyaw);
		eLog("%6.2f %6.2f %6.2f %6.2f", ex, ey, ez, eyaw);
		vLog("%6.2f %6.2f %6.2f", bd.v.x, bd.v.y, bd.v.z);

		if (std::fabs(ex) <= posThresholdInM &&
		    std::fabs(ey) <= posThresholdInM &&
		    std::fabs(ez) <= posThresholdInM &&
		    std::fabs(eyaw) <= yawThresholdInDeg)
			break;

		controller.control(ex, ey, ez, eyaw);

		usleep(intervalInMs * 1000);
		elapsedTime += intervalInMs;
		if (elapsedTime >= timeoutInS * 1000) {
			std::cout << timeoutInS << "s Timed out\n";
			break;
		}

	}

	controller.stop();
	std::cout << "moveByPositionOffset thread exit\n";
}

// 线程：使用KCF算法计算目标物体的相对位置。
void Fetch::runKCF() {

	try {

		double plane_angle_yaw, plane_angle_roll, plane_angle_pitch;

		TrackWithDistance twd;
		twd.getInitBBox();

		while (1) {

			if (stop_task) break;

			float *coord = twd.getObjectCoordinate();

			bd = api->getBroadcastData();

			plane_angle_roll  = atan2(2.0 * (bd.q.q3 * bd.q.q2 + bd.q.q0 * bd.q.q1) , 1.0 - 2.0 * (bd.q.q1 * bd.q.q1 + bd.q.q2 * bd.q.q2)) * 180 / Pi;
			plane_angle_pitch = asin(2.0 * (bd.q.q2 * bd.q.q0 - bd.q.q3 * bd.q.q1)) * 180 / Pi;
			plane_angle_yaw   = atan2(2.0 * (bd.q.q3 * bd.q.q0 + bd.q.q1 * bd.q.q2) , - 1.0 + 2.0 * (bd.q.q0 * bd.q.q0 + bd.q.q1 * bd.q.q1)) * 180 / Pi;

			Mat error = ComputeCoordinate(plane_angle_roll, plane_angle_pitch, plane_angle_yaw,
			                              bd.gimbal.roll, bd.gimbal.pitch, bd.gimbal.yaw, bd.pos.height, coord[0], coord[1]);
			ex = error.at<float>(0, 0);
			ey = error.at<float>(1, 0);
			ez = error.at<float>(2, 0);

			// todo sleep一会儿？

		}
		
	} catch (Exception e) {
		stop_task = true;  // 让其他线程退出
	}

	std::cout << "runKCF thread exit\n";
}

void Fetch::approaching()
{
	const float32_t posThresholdInM = posThresholdInCm / 100;

	bool above = false;  // 是否飞到目标上方
	int elapsedTime = 0;

	FlightControl controller(flight, maxSpeedInM, maxYawSpeedInDeg, intervalInMs / 1000.0);
	Logger eLog("error.txt");
	Logger vLog("velocity.txt");

	while (1) {

		if (stop_task) break;

		// 误差由KCF线程实时求出

		printf("err: %6.2f %6.2f %6.2f %6.2f\n", ex, ey, ez, eyaw);
		eLog("%6.2f %6.2f %6.2f %6.2f", ex, ey, ez, eyaw);
		vLog("%6.2f %6.2f %6.2f", bd.v.x, bd.v.y, bd.v.z);

		// 检查任务完成情况
		if (std::fabs(ex) <= posThresholdInM &&
		    std::fabs(ey) <= posThresholdInM) {
			if (!above) {
				cout << "finish fly to above\n";
				above = true;
			}
			if (std::fabs(ez) <= posThresholdInM) {
				cout << "finish approach\n";
				break;
			}
		}

		controller.control(ex, ey, above ? ez : 0, eyaw);

		usleep(intervalInMs * 1000);
		elapsedTime += intervalInMs;
		if (elapsedTime >= timeoutInS * 1000) {
			stop_task = true;  // 让其他线程退出
			std::cout << timeoutInS << "s Timed out\n";
			break;
		}

	}

	controller.stop();
	std::cout << "approaching thread exit\n";
}
