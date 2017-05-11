#include <time.h>

#include <opencv2/opencv.hpp>

#include <boost/thread.hpp>

#include "Fetch.h"

#include "TrackWithDistance.h"
#include "estimatePos.h"
#include "PIDControl.h"

using namespace std;

Fetch::Fetch(float timeoutInS, float posThresholdInCm, float maxSpeedInM)
  : timeoutInS(timeoutInS), posThresholdInCm(posThresholdInCm), maxSpeedInM(maxSpeedInM)
{
  record_flag = false;
  stop_task = true;
  calc_flag = false;
}

void Fetch::setApi(CoreAPI * value) {api = value;}

void Fetch::setFlight(Flight * value) {flight = value;}

void Fetch::start_position_control(float x, float y, float z, float yaw) {
  stop_task = false;
  boost::thread record(boost::bind(&Fetch::record, this));
  boost::thread move(boost::bind(&Fetch::moveByPositionOffset, this, x, y, z, yaw));
}

/*member function for tracknig*/
void Fetch::start_approaching()
{
  stop_task = false;
  calc_flag = true;
  boost::thread record(boost::bind(&Fetch::record, this));
  boost::thread counter(boost::bind(&Fetch::counter, this));
  boost::thread approach(boost::bind(&Fetch::approaching, this));
  //track.detach();
}

void Fetch::stop() {
  stop_task = true;
}

void Fetch::setParameters(float timeoutInS, float posThresholdInCm, float maxSpeedInM) {
  this->timeoutInS = timeoutInS;
  this->posThresholdInCm = posThresholdInCm;
  this->maxSpeedInM = maxSpeedInM;
}

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

// 修改自sample：Liunx Blocking
// todo finish the record about yaw
void Fetch::moveByPositionOffset(float x, float y, float z, float yaw) {

  static const double yawThresholdInDeg = 5;

  const uint8_t flag = 0x49;  // Velocity Control
  const float32_t posThresholdInM = posThresholdInCm / 100;

  double yawDesiredRad = DEG2RAD * yaw;
  double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;
  int elapsedTime = 0;
  float uyaw;

  // Get current poition
  PositionData originPosition = api->getBroadcastData().pos;
  Vector3dData curLocalOffset;

  // todo 设置PID参数
  PIDControl xController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);
  PIDControl yController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);
  PIDControl zController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);
  PIDControl yawController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);

  while (true) {

		if (stop_task) {
      std::cout << "\ntask stopped\n";
      flight_control(flag, 0, 0, 0, 0);
      break;
		}

    // 获取误差
    PositionData curPosition = api->getBroadcastData().pos;
    EulerAngle curEuler = toEulerAngle(api->getBroadcastData().q);

    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
    ex = x - curLocalOffset.x;
    ey = y - curLocalOffset.y;
    ez = z - curLocalOffset.z;
    double errorYaw = yawDesiredRad - curEuler.yaw;
    if (std::fabs(ex) <= posThresholdInM &&
        std::fabs(ey) <= posThresholdInM &&
        std::fabs(ez) <= posThresholdInM &&
        std::fabs(errorYaw) <= yawThresholdInRad)
      break;

    // 计算控制量
    ux = xController(ex);
    uy = yController(ey);
    uz = zController(ez);
    uyaw = yawController(errorYaw);

    cout << "ex:" << ex << " ey:" << ey << " ez:" << ez << "\r";
		record_flag = true;

    // 发送控制
    flight_control(flag, ux, uy, uz, uyaw);

    usleep(intervalInMs * 1000);
    elapsedTime += intervalInMs;
    if (elapsedTime >= timeoutInS * 1000) {
      std::cout << "\nTimed out\n";
      flight_control(flag, 0, 0, 0, 0);
      return;
    }

  }

}

void Fetch::record()
{
  struct tm *t;
  time_t tt;

  time(&tt);
  t = localtime(&tt);

  char s[100];
  sprintf(s, "%4d%02d%02d_%02d_%02d_%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
  string time(s);

  std::ofstream ferror((time + "_error.txt").c_str());
  std::ofstream fvelocity((time + "_velocity.txt").c_str());
  std::ofstream fu((time + "_u.txt").c_str());
  /*string filename2 = s + "roll_L.txt";
  string filename3 = s + "pitch_L.txt";
  string filename4 = s + "velocity_L.txt";
  string filename5 = s + "velocity_L_cal.txt";
  string filename6 = s + "targetcenter.txt";
  string filename7 = s + "distance.txt";
  */

  while (1)
  {
    while (!record_flag && !stop_task);
    if (stop_task) break;
    record_flag = false;

    bd = api->getBroadcastData();

    //plane_angle_roll  = atan2(2.0 * (bd.q.q3 * bd.q.q2 + bd.q.q0 * bd.q.q1) , 1.0 - 2.0 * (bd.q.q1 * bd.q.q1 + bd.q.q2 * bd.q.q2)) * 180 /Pi;
    //plane_angle_pitch = asin(2.0 * (bd.q.q2 * bd.q.q0 - bd.q.q3 * bd.q.q1)) * 180 / Pi;

    ferror << ex << " " << ey  << " " << ez << '\n';
    fvelocity << bd.v.x << " " << bd.v.y << " " << bd.v.z << "\n";
    fu << ux << " " << uy  << " " << uz << '\n';
    //froll << plane_angle_roll << '\n';
    //fpitch << plane_angle_pitch << '\n';
  }
  ferror.close();
  fvelocity.close();
  fu.close();
}

void Fetch::counter() {
  int elapsedTime = 0;  // ms
  while (!stop_task) {
    usleep(1000);
    if (!calc_flag) {
      elapsedTime += 1;
      if (elapsedTime >= intervalInMs) {
        calc_flag = true;
        elapsedTime = 0;
      }
    }
  }
}

void Fetch::approaching()
{
  const uint8_t flag = 0x49;  // Velocity Control
  const float32_t posThresholdInM = posThresholdInCm / 100;
	double plane_angle_yaw, plane_angle_roll, plane_angle_pitch;

  time_t startTime, curTime;
  time(&startTime);

  bool above = false;  // 是否飞到目标上方

  // todo 设置PID参数
  PIDControl xController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);
  PIDControl yController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);
  PIDControl zController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);

  TrackWithDistance twd;
  twd.getInitBBox();

  while (1) {

    // waiting for signal

    while (!calc_flag && !stop_task);
    calc_flag = false;

    if (stop_task) {
      std::cout << "\ntask stopped\n";
      flight_control(flag, 0, 0, 0, 0);
      break;
    }

    // calculate error

    float* coord = twd.getObjectCoordinate();

    bd = api->getBroadcastData();

    plane_angle_roll  = atan2(2.0 * (bd.q.q3 * bd.q.q2 + bd.q.q0 * bd.q.q1) , 1.0 - 2.0 * (bd.q.q1 * bd.q.q1 + bd.q.q2 * bd.q.q2)) * 180 / Pi;
    plane_angle_pitch = asin(2.0 * (bd.q.q2 * bd.q.q0 - bd.q.q3 * bd.q.q1)) * 180 / Pi;
    plane_angle_yaw   = atan2(2.0 * (bd.q.q3 * bd.q.q0 + bd.q.q1 * bd.q.q2) , - 1.0 + 2.0 * (bd.q.q0 * bd.q.q0 + bd.q.q1 * bd.q.q1)) * 180 / Pi;

    Mat error = ComputeCoordinate(plane_angle_roll, plane_angle_pitch, plane_angle_yaw,
                                  bd.gimbal.roll, bd.gimbal.pitch, bd.gimbal.yaw, bd.pos.height, coord[0], coord[1]);
    ex = error.at<float>(0, 0);
    ey = error.at<float>(1, 0);
    ez = error.at<float>(2, 0);

    // 检查任务完成情况

    if (std::fabs(ex) <= posThresholdInM &&
        std::fabs(ey) <= posThresholdInM) {
      if (!above) {
        cout << "\nfinish fly to above\n";
        above = true;
      }
      if (std::fabs(ez) <= posThresholdInM) {
        cout << "\nfinish approach\n";
        break;
      }
    }

    // 计算控制量
    ux = xController(ex);
    uy = yController(ey);
    uz = zController(ez);

    cout << "ex:" << ex << " ey:" << ey << " ez:" << ez << "\r";
    record_flag = true;

    // 发送控制
    flight_control(flag, ux, uy, above ? uz : 0, 0);

    time(&curTime);
    if (curTime - startTime >= timeoutInS) {
      stop_task = true;  // tell the other threads to exit
      std::cout << "\nTimed out\n";
      flight_control(flag, 0, 0, 0, 0);
      return;
    }

  }

  return;

  /*
    DJI::onboardSDK::GimbalSpeedData Gdata;
    filterInit();

    cout << "start control!" << endl;

    int stable_count = 0;

    int ret_stop;
    ret_stop = pthread_create(&t_stop,NULL,stop,NULL);
    if(ret_stop != 0)
      cout << "Can't not creat the thread to stop tracking!" << endl;
    else
    {
      while(1)
      {/*
        if(!record_flag && done)break;
        else if(record_flag && !target_lost)
        {
          if(kcfinit)
          {
            if(stable)
            {
            }
            else
            {
              cout << "Follow!" << endl;
              //follow module
              bd = api->getBroadcastData();

              //update data
              q.q0 = bd.q.q0;
              q.q1 = bd.q.q1;
              q.q2 = bd.q.q2;
              q.q3 = bd.q.q3;
              plane_yaw = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1)) * 180 / Pi;
              if(plane_yaw > 90 && bd.gimbal.yaw < -90)
                camera_yaw = 360 - plane_yaw + bd.gimbal.yaw;
              else if(plane_yaw < -90 && bd.gimbal.yaw > 90)
                camera_yaw = -(360 + plane_yaw - bd.gimbal.yaw);
              else
                camera_yaw = bd.gimbal.yaw - plane_yaw;

              camera_pitch = bd.gimbal.pitch;
              plane_height = bd.pos.height;

              gimbalcontrol_data(camera_yaw,camera_pitch);

              //gimbal control
              gimbalmove = true;
              Gdata.yaw = (G_yaw) * 20;
              Gdata.pitch = (G_pitch) * 20;
              Gdata.roll = 0;
              Gdata.reserved = 0x80;
              getGimbal()->setGimbalSpeed(&Gdata);

              //flight control
              target_possition(camera_yaw,camera_pitch,plane_height);
              fdistance << target.d << ' ';

              //get the flight data
              bd = api->getBroadcastData();
              flight_data.velocity_x = bd.v.x;
              flight_data.velocity_y = bd.v.y;

              plane_yaw = atan2(2.0 * (bd.q.q3 * bd.q.q0 + bd.q.q1 * bd.q.q2) , - 1.0 + 2.0 * (bd.q.q0 * bd.q.q0 + bd.q.q1 * bd.q.q1)) * 180 / Pi;
              if(plane_yaw > 90 && bd.gimbal.yaw < -90)
                camera_yaw = 360 - plane_yaw + bd.gimbal.yaw;
              else if(plane_yaw < -90 && bd.gimbal.yaw > 90)
                camera_yaw = -(360 + plane_yaw - bd.gimbal.yaw);
              else
                camera_yaw = bd.gimbal.yaw - plane_yaw;
              flight_data.gimbal_yaw = camera_yaw;
              flight_data.gimbal_pitch = bd.gimbal.pitch;

              if(abs(plane_height - set_height) > 0.3)
              {
                while((set_height - api->getBroadcastData().pos.height) > 0.1 ||
                      (api->getBroadcastData().pos.height - set_height) > 0.1)
                {
                  flight_control(0x5B,0,0,set_height,0);
                  usleep(20000);
                }
              }

              gimbalmove = false;

              kalman_filter();
              fdistance << sqrt(target.x * target.x + target.y * target.y) << '\n';
              SwitchUAV();
              //flight_control(0x4B,0,0,0,control_data.yaw_rate);//test yaw rate
              //flight_control(0x4B,control_data.v_x,0,0,control_data.yaw_rate);//test go forward and go back
              flight_control(0x4B,control_data.v_x,control_data.v_y,0,control_data.yaw_rate);//final test
            }
          }
        }
        else if(target_lost)
        {
          cout << "Detect!" << endl;
          //search module
          //search_target = false;
          //gimbal control
          gimbalmove = false;
          Gdata.yaw = 0;
          Gdata.pitch = 0;
          Gdata.roll = 0;
          Gdata.reserved = 0x80;
          getGimbal()->setGimbalSpeed(&Gdata);
          int count = 0;
        }
      usleep(100000);
      }
    }
  */
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
