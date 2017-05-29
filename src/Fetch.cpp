#include <time.h>

#include <opencv2/opencv.hpp>

#include <boost/thread.hpp>

#include "Fetch.h"

#include "PIDControl.h"
#include "Logger.h"

#include "TrackWithDistance.h"
#include "estimatePos.h"

using namespace std;

Fetch::Fetch(float timeoutInS, float posThresholdInCm, float maxSpeedInM, float yawThresholdInDeg, float maxYawSpeedInDeg, float frequency)
	: timeoutInS(timeoutInS),
	  posThresholdInCm(posThresholdInCm), maxSpeedInM(maxSpeedInM),
	  yawThresholdInDeg(yawThresholdInDeg), maxYawSpeedInDeg(maxYawSpeedInDeg),
	  frequency(frequency)
{
	stop_task = true;
	target_found = false;
}

void Fetch::start_position_control(float x, float y, float z, float yaw) {
	stop_task = false;
	boost::thread move(boost::bind(&Fetch::moveByPositionOffset, this, x, y, z, yaw));
}

/*member function for tracknig*/
void Fetch::start_approaching()
{
	stop_task = false;
	target_found = false;
	boost::thread kcf(boost::bind(&Fetch::runKCF, this));
	boost::thread approach(boost::bind(&Fetch::approaching, this));
	//track.detach();
}

void Fetch::stop() {
	stop_task = true;
}

create_definition(Fetch, Flight *, flight);
create_definition(Fetch, CoreAPI *, api);
create_definition(Fetch, Camera *, camera);

create_definition(Fetch, float, timeoutInS);
create_definition(Fetch, float, posThresholdInCm);
create_definition(Fetch, float, maxSpeedInM);
create_definition(Fetch, float, yawThresholdInDeg);
create_definition(Fetch, float, maxYawSpeedInDeg);
create_definition(Fetch, float, frequency);

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

	const float intervalInS = 1.0f / frequency;
	const float posThresholdInM = posThresholdInCm / 100;
	const uint8_t flag = 0x49;  // 世界坐标系的速度控制

	PositionData originPosition = api->getBroadcastData().pos;
	const float targetYaw = angle(toEulerAngle(api->getBroadcastData().q).yaw / DEG2RAD, -yaw);  // 相当于做加法

	Vector3dData curLocalOffset;
	float elapsedTime = 0;
	float ex, ey, ez, eyaw;
	float ux, uy, uz, uyaw;

	PIDControl xController("../config/kx", maxSpeedInM, intervalInS);
	PIDControl yController("../config/ky", maxSpeedInM, intervalInS);
	PIDControl zController("../config/kz", maxSpeedInM, intervalInS);
	PIDControl yawController("../config/kyaw", maxYawSpeedInDeg, intervalInS);
	Logger uLog("../log/u");
	Logger eLog("../log/error");
	Logger vLog("../log/velocity");

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
		eLog("%.2f,%.2f,%.2f,%.2f", ex, ey, ez, eyaw);
		vLog("%.2f,%.2f,%.2f", bd.v.x, bd.v.y, bd.v.z);

		if (fabs(ex) <= posThresholdInM &&
		        fabs(ey) <= posThresholdInM &&
		        fabs(ez) <= posThresholdInM &&
		        fabs(bd.v.x) <= posThresholdInM &&
		        fabs(bd.v.y) <= posThresholdInM &&
		        fabs(bd.v.z) <= posThresholdInM &&
		        fabs(eyaw) <= yawThresholdInDeg) {
			cout << "finish move\n";
			break;
		}

		// 计算控制量，发送控制
		ux = xController(ex);
		uy = yController(ey);
		uz = zController(ez);
		uyaw = yawController(eyaw);
		uLog("%.2f,%.2f,%.2f,%.2f", ux, uy, uz, uyaw);
		flight_control(flag, ux, uy, uz, uyaw);

		usleep(intervalInS * 1000000);
		elapsedTime += intervalInS;
		if (elapsedTime >= timeoutInS) {
			cout << timeoutInS << "s Timed out\n";
			break;
		}

	}

	flight_control(0x49, 0, 0, 0, 0);  // stop
	cout << "moveByPositionOffset thread exit\n";
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
			float u = coord[0];
			float v = coord[1];

			bd = api->getBroadcastData();

			plane_angle_roll  = atan2(2.0 * (bd.q.q3 * bd.q.q2 + bd.q.q0 * bd.q.q1) , 1.0 - 2.0 * (bd.q.q1 * bd.q.q1 + bd.q.q2 * bd.q.q2)) * 180 / Pi;
			plane_angle_pitch = asin(2.0 * (bd.q.q2 * bd.q.q0 - bd.q.q3 * bd.q.q1)) * 180 / Pi;
			plane_angle_yaw   = atan2(2.0 * (bd.q.q3 * bd.q.q0 + bd.q.q1 * bd.q.q2) , - 1.0 + 2.0 * (bd.q.q0 * bd.q.q0 + bd.q.q1 * bd.q.q1)) * 180 / Pi;

			Mat error = ComputeCoordinate(plane_angle_roll, plane_angle_pitch, plane_angle_yaw,
			                              bd.gimbal.roll, bd.gimbal.pitch, bd.gimbal.yaw, bd.pos.height, u, v);
			ex = error.at<float>(0, 0);
			ey = error.at<float>(1, 0);
			ez = error.at<float>(2, 0) - 0.5;

			tex = 0;
			tey = 0;

			target_found = true;

			// todo sleep一会儿？

		}

	} catch (Exception e) {
		stop_task = true;  // 让其他线程退出
	}

	cout << "runKCF thread exit\n";
}

void Fetch::approaching()
{
	ex = 0, ey = 0, ez = 0, eyaw = 0, tex = 0, tey = 0;

	const float intervalInS = 1.0f / frequency;
	const float32_t posThresholdInM = posThresholdInCm / 100;
	const uint8_t flag = 0x4B;  // 机体坐标系的速度控制

	bool above = false;  // 是否飞到目标上方
	float elapsedTime = 0;
	DJI::onboardSDK::GimbalSpeedData Gdata;
	float ux, uy, uz, uyaw;

	while (!target_found);

	PIDControl gimbal_yawControl(2, 0, 0, 50, intervalInS);
	PIDControl gimbal_pitchControl(2, 0, 0, 50, intervalInS);
	PIDControl xController("../config/kx", maxSpeedInM, intervalInS);
	PIDControl yController("../config/ky", maxSpeedInM, intervalInS);
	PIDControl zController("../config/kz", maxSpeedInM, intervalInS);
	PIDControl yawController("../config/kyaw", maxYawSpeedInDeg, intervalInS);
	Logger uLog("../log/u");
	Logger eLog("../log/error");
	Logger vLog("../log/velocity");
	Logger teLog("../log/target_error");

	while (1) {

		if (stop_task) break;

		if (!target_found) continue;

		// 误差由KCF线程实时求出

		printf("err: %6.2f %6.2f %6.2f %6.2f\n", ex, ey, ez, eyaw);
		eLog("%.2f,%.2f,%.2f,%.2f", ex, ey, ez, eyaw);
		vLog("%.2f,%.2f,%.2f", bd.v.x, bd.v.y, bd.v.z);
		teLog("%.2f,%.2f", tex, tey);

		// 检查任务完成情况
		if (fabs(ex) <= posThresholdInM &&
		        fabs(ey) <= posThresholdInM &&
		        fabs(bd.v.x) <= posThresholdInM &&
		        fabs(bd.v.y) <= posThresholdInM) {
			if (!above) {
				cout << "finish fly to above\n";
				above = true;
			}
			if (fabs(ez) <= posThresholdInM &&
			        fabs(bd.v.z) <= posThresholdInM) {
				cout << "finish approach\n";
				break;
			}
		}

		// 云台控制。准备使用kcf得到的像素值偏移量
		Gdata.yaw = gimbal_yawControl(tex);
		Gdata.pitch = gimbal_pitchControl(tey);
		Gdata.roll = 0;
		Gdata.reserved = 0x80;
		camera->setGimbalSpeed(&Gdata);

		// 计算控制量，发送控制
		ux = xController(ex);
		uy = yController(ey);
		uz = zController(above ? ez : 0);
		uyaw = yawController(eyaw);
		uLog("%.2f,%.2f,%.2f,%.2f", ux, uy, uz, uyaw);
		flight_control(flag, ux, uy, uz, uyaw);

		usleep(intervalInS * 1000000);
		elapsedTime += intervalInS;
		if (elapsedTime >= timeoutInS) {
			stop_task = true;  // 让其他线程退出
			cout << timeoutInS << "s Timed out\n";
			break;
		}

	}

	flight_control(0x49, 0, 0, 0, 0);  // stop
	cout << "approaching thread exit\n";
}

void Fetch::flight_control(uint8_t flag, float x, float y, float z, float yaw)
{
	FlightData fd;
	fd.flag = flag;
	fd.x = x;
	fd.y = y;
	fd.z = z;
	fd.yaw = yaw;
	flight->setFlight(&fd);
}
