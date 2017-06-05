#include <time.h>

#include <opencv2/opencv.hpp>

#include <boost/thread.hpp>

#include "Fetch.h"

#include "PIDControl.h"
#include "Logger.h"

#include "TrackWithDistance.h"
#include "estimatePos.h"
#include "SetArm.h"

using namespace std;

Fetch::Fetch(float timeoutInS, float posThresholdInCm, float maxSpeedInM, float yawThresholdInDeg, float maxYawSpeedInDeg, float frequency)
	: timeoutInS(timeoutInS),
	  posThresholdInCm(posThresholdInCm), maxSpeedInM(maxSpeedInM),
	  yawThresholdInDeg(yawThresholdInDeg), maxYawSpeedInDeg(maxYawSpeedInDeg),
	  frequency(frequency)
{
	stop_task = true;
	target_found = false;

	hoverHeight = 2;
	fetchHeight = 0.5;

	// 使用之前，先执行命令 sudo insmod /lib/modules/3.10.40/kernel/drivers/usb/serial/cp210x.ko
	// 读入串口路径，若为"-",则默认设置/dev/ttyUSB0, 不一定对。请自己找下
	// 查找命令 ls -l /dev/ttyUSB*
	// char dev[110] = "/dev/ttyUSB0";
	// setdev(dev);
}

void Fetch::start_position_control(float x, float y, float z, float yaw) {
	stop_task = false;
	boost::thread move(boost::bind(&Fetch::moveByPositionOffset, this, x, y, z, yaw));
}

void Fetch::start_height_control(float z) {
	stop_task = false;
	boost::thread move(boost::bind(&Fetch::moveByHeightOffset, this, z));
}

void Fetch::start_approaching()
{
	stop_task = false;
	target_found = false;
	boost::thread kcf(boost::bind(&Fetch::runKCF, this));
	boost::thread approach(boost::bind(&Fetch::approaching, this));
}

void Fetch::start_auto_fetch()
{
	stop_task = false;
	target_found = false;
	boost::thread kcf(boost::bind(&Fetch::runKCF, this));
	boost::thread fetch(boost::bind(&Fetch::auto_fetch, this));
}

// len是爪子尖端之间的距离，0～29
// speed是旋转角速度建议1～10  实际0～20
void Fetch::fetch() {
	double len = 5;
	int speed = 10;
	double height = SetArmbyLen(len, speed);  //没有加上底座高度，底座高度2.8cm
}

void Fetch::release() {
	double len = 29;
	int speed = 10;
	double height = SetArmbyLen(len, speed);
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
create_definition(Fetch, float, hoverHeight);
create_definition(Fetch, float, fetchHeight);
create_definition(Fetch, float, gimbalOffsetX);

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
void Fetch::moveByPositionOffset(float x, float y, float z, float yaw) {

	const float intervalInS = 1.0f / frequency;
	const float posThresholdInM = posThresholdInCm / 100;

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
	Logger uLog("../log/move_u");
	Logger eLog("../log/move_error");
	Logger vLog("../log/move_velocity");

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
		flight_control(0x49, ux, uy, uz, uyaw);  // 世界坐标系的速度控制

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

void Fetch::moveByHeightOffset(float z) {

	const float intervalInS = 1.0f / frequency;
	const float posThresholdInM = posThresholdInCm / 100;

	float originZ = api->getBroadcastData().pos.height;

	float elapsedTime = 0;
	float ez;
	float uz;

	PIDControl zController("../config/kz", maxSpeedInM, intervalInS);
	Logger uLog("../log/height_u");
	Logger eLog("../log/height_error");
	Logger vLog("../log/height_velocity");

	while (true) {

		if (stop_task) break;

		// 计算误差
		bd = api->getBroadcastData();
		float curZ = bd.pos.height;

		ez = z + originZ - curZ;

		printf("err z: %6.2f\n", ez);
		eLog("%.2f", ez);
		vLog("%.2f,%.2f,%.2f", bd.v.x, bd.v.y, bd.v.z);

		if (fabs(ez) <= posThresholdInM &&
		        fabs(bd.v.z) <= posThresholdInM) {
			cout << "finish moveHeight\n";
			break;
		}

		// 计算控制量，发送控制
		uz = zController(ez);
		uLog("%.2f", uz);
		flight_control(0x49, 0, 0, uz, 0);  // 世界坐标系的速度控制

		usleep(intervalInS * 1000000);
		elapsedTime += intervalInS;
		if (elapsedTime >= timeoutInS) {
			cout << timeoutInS << "s Timed out\n";
			break;
		}

	}

	flight_control(0x49, 0, 0, 0, 0);  // stop
	cout << "moveByHeightOffset thread exit\n";
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
			ex = error.at<float>(0, 0) + gimbalOffsetX;
			ey = error.at<float>(1, 0);
			ez = - error.at<float>(2, 0);

			tex = u - 213;
			tey = 120 - v;

			target_found = true;

		}

	} catch (...) {
		stop_task = true;  // 让其他线程退出
		cout << "runKCF thread exception!!!\n";
	}

	cout << "runKCF thread exit\n";
}

void Fetch::approaching()
{

	const float intervalInS = 1.0f / frequency;
	const float32_t posThresholdInM = posThresholdInCm / 100;

	float elapsedTime = 0;
	DJI::onboardSDK::GimbalSpeedData Gdata;
	float ux, uy, uz, uyaw;

	while (!target_found && !stop_task);
	if (stop_task) return;

	PIDControl gimbal_yawControl(4, 0, 0, 100, intervalInS);
	PIDControl gimbal_pitchControl(6, 0, 0, 100, intervalInS);
	PIDControl xController("../config/kx", maxSpeedInM, intervalInS);
	PIDControl yController("../config/ky", maxSpeedInM, intervalInS);
	PIDControl zController("../config/kz", maxSpeedInM, intervalInS);
	PIDControl yawController("../config/kyaw", maxYawSpeedInDeg, intervalInS);
	Logger uLog("../log/approach_u");
	Logger eLog("../log/approach_error");
	Logger vLog("../log/approach_velocity");
	Logger teLog("../log/approach_target_error");

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
		        fabs(ez + hoverHeight) <= posThresholdInM &&
		        fabs(bd.v.x) <= posThresholdInM &&
		        fabs(bd.v.y) <= posThresholdInM &&
		        fabs(bd.v.z) <= posThresholdInM) {
			cout << "finish approach\n";
			break;
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
		uz = zController(ez + hoverHeight);
		uyaw = yawController(eyaw);
		uLog("%.2f,%.2f,%.2f,%.2f", ux, uy, uz, uyaw);
		flight_control(0x4B, ux, uy, uz, uyaw);  // 机体坐标系的速度控制

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

void Fetch::auto_fetch() {

	// 移动使机械爪移动到物体正上方
	approaching();
	if (stop_task) return;

	// todo 停止KCF线程？

	usleep(3 * 1000000);

	// 下降到一定高度
	// moveByPositionOffset(0, 0, ez + fetchHeight, 0);
	moveByHeightOffset(ez + fetchHeight);
	stop_task = true;
	if (stop_task) return;

	// 机械爪抓取
	// fetch();
	usleep(2 * 1000000);

	// 升高一米
	// moveByPositionOffset(0, 0, 1, 0);
	moveByHeightOffset(1);
	if (stop_task) return;

	// 丢掉~~~~
	// release();

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
