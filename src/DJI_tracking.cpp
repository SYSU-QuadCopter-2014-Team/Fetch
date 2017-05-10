#include "DJI_tracking.h"
//#include "../kcfsrc/runtracker.h"
//#include "Flightcontrol.h"
//#include "Gimbalcontrol.h"
#include "../kcfsrc/TrackWithDistance.h"
#include "../kcfsrc/estimatePos.h"
#include "../kcfsrc/PIDControl.h"

#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <sys/time.h>
#include <signal.h>
#include <time.h>
#include <boost/thread.hpp>
#include <pthread.h>
#include <math.h>
#include <unistd.h>
#include <ctime>
#include <sstream>
#include <ctime>

#include <DJI_Type.h>

#ifdef __linux__
#include <sys/select.h>

int kbhit()
{
  struct timeval tv;
  fd_set read_fd;

  /* Do not wait at all, not even a microsecond */
  tv.tv_sec=0;
  tv.tv_usec=0;

  /* Must be done first to initialize read_fd */
  FD_ZERO(&read_fd);

  /* Makes select() ask if input is ready:
   * 0 is the file descriptor for stdin    */
  FD_SET(0,&read_fd);

  /* The first parameter is the number of the
   * largest file descriptor to check + 1. */
  if(select(1, &read_fd,NULL, /*No writes*/NULL, /*No exceptions*/&tv) == -1)
    return 0;  /* An error occured */

  /*  read_fd now holds a bit map of files that are
   * readable. We test the entry for the standard
   * input (file 0). */
  
if(FD_ISSET(0,&read_fd))
    /* Character pending on stdin */
    return 1;

  /* no characters were pending */
  return 0;
}
#endif

using namespace DJI;
using namespace DJI::onboardSDK;
using namespace std;

BroadcastData bd;

bool target_in_center_up = 	 true;
bool target_in_center_down = true;

pthread_t t_stop;

/***********Global variable for get the target possition***************/
double plane_angle_yaw;
double plane_angle_roll;
double plane_angle_pitch;
double uav_v_x;
double uav_v_y;
double camera_yaw;
double camera_pitch;
double plane_yaw;
double plane_height;
int set_height;

string commandList[] = 
{
	"ac",
	"ct1",
	"ct0",
	"gr",
	"gl",
	"gu",
	"gd",
	"bd",
	"tk",
	"gh",
	"ld",
	"fc",
	"sa",
	"st",
	"sh",
	"qs",
	"sp"
};

/*member function for system*/
Tracking_system::Tracking_system(CoreAPI * controlAPI,long int keyID,char * key)
{
	adata.ID = keyID;
	adata.reserved = 2;
	adata.version = SDK_VERSION;
	adata.encKey = key;

	api = controlAPI;

	timeoutInS = 30;
	posThresholdInCm = 10;
	maxSpeedInM = 0.1;

	record_flag = false;
	quit = false;
	stop_task = false;
	calc = false;

	camera = new Camera(api);
	flight = new Flight(api);
}

void Tracking_system::Display_manu()
{
		cout << endl;
    cout << "|------------------DJI onboardSDK command line------------------|" << endl;
		cout << "|                                                               |" << endl;
    cout << "| - available modules                                           |" << endl;
    cout << "| - < 0.ac > active core module                                 |" << endl;
		cout << "| - < 1.ct1 > ct1 to obtain control                             |" << endl;
		cout << "| - < 2.ct0 > ct0 to release control                            |" << endl;
    cout << "| - < 3.gr > gimble right 10 degree                             |" << endl;
    cout << "| - < 4.gl > gimble left  10 degree                             |" << endl;
    cout << "| - < 5.gu > gimble up    10 degree                             |" << endl;
    cout << "| - < 6.gd > gimble down  10 degree                             |" << endl;
		cout << "| - < 7.bd > get the baudrate date                              |" << endl;
		cout << "| - < 8.tk > take off                                           |" << endl;
		cout << "| - < 9.gh > go home                                            |" << endl;
		cout << "| - < 10.ld > landing                                           |" << endl;
		cout << "| - < 11.fc > flight control:relative posistion                 |" << endl;
		cout << "| - < 12.sa > start approaching                                 |" << endl;
		cout << "| - < 13.st > stop task                                         |" << endl;
		cout << "| - < 14.sh > set hight                                         |" << endl;
		cout << "| - < 15.qs > quit system                                       |" << endl;
		cout << "| - < 16.sp > set parameters                                    |" << endl;
    cout << "|                                                               |" << endl;
    cout << "|------------------DJI onboardSDK command line------------------|" << endl;
		cout << endl;
}

void Tracking_system::cmd_match()
{

	bool command_match = false;
	cout << "Input your command: ";
	int command_num;
	string inputCmd;
	getline(cin,inputCmd);
	for(int i = 0;i < sizeof(commandList)/sizeof(commandList[0]);i++)
	{
		if(inputCmd == commandList[i])
		{
			command_num = i;
			command_match = true;
			break;
		}
	}

	if(!command_match)
	{
		cout << "Wrong input!" << endl;
	}
	else
	{
		switch(command_num)
		{
			case 0:
					activeAPI();
					break;
			case 1:
					obtain_control();
					camera_init();
					break;
			case 2:
					release_control();
					break;
			case 3:
					control_gimbal(200,0,0);
					break;
      case 4:
					control_gimbal(-200,0,0);
					break;
      case 5:
					control_gimbal(0,0,200);
					break;
      case 6:
					control_gimbal(0,0,-200);
					break;
			case 7:
					getbd();
					break;
			case 8:
					flight_control_tk();
					break;
			case 9:
					stop_task = true;
					flight_control_gh();
					break;
			case 10:
					stop_task = true;
					flight_control_ld();
					break;
			case 11:
					stop_task = true;
					start_position_control();
					break;
			case 12:
					start_approaching();
					break;
			case 13:
					stop_task = true;
					break;
			case 14:
					flight_control_sh();
					break;
			case 15:
					stop_task = true;
					quit = true;
					break;
			case 16:
					cout << "Input parameters timeoutInS posThresholdInCm maxSpeedInM:";
					cin >> timeoutInS >> posThresholdInCm >> maxSpeedInM;
		}
	}
			
}

/*member function for CoreAPI*/
void Tracking_system::activeAPI(){getApi()->activate(&adata);}

void Tracking_system::obtain_control(){getApi()->setControl(true);}

void Tracking_system::release_control(){getApi()->setControl(false);}

void Tracking_system::getbd()
{
	bd = api->getBroadcastData();
	double cam_yaw;
	
	plane_angle_roll  = atan2(2.0 * (bd.q.q3 * bd.q.q2 + bd.q.q0 * bd.q.q1) , 1.0 - 2.0 * (bd.q.q1 * bd.q.q1 + bd.q.q2 * bd.q.q2)) * 180 /Pi;
  plane_angle_pitch = asin(2.0 * (bd.q.q2 * bd.q.q0 - bd.q.q3 * bd.q.q1)) * 180 / Pi;
  plane_angle_yaw   = atan2(2.0 * (bd.q.q3 * bd.q.q0 + bd.q.q1 * bd.q.q2) , - 1.0 + 2.0 * (bd.q.q0 * bd.q.q0 + bd.q.q1 * bd.q.q1)) * 180 / Pi;

	if(plane_angle_yaw > 90 && bd.gimbal.yaw < -90)
		cam_yaw = 360 - plane_angle_yaw + bd.gimbal.yaw;
	else if(plane_angle_yaw < -90 && bd.gimbal.yaw > 90)
		cam_yaw = -(360 + plane_angle_yaw - bd.gimbal.yaw);
	else
		cam_yaw = bd.gimbal.yaw - plane_angle_yaw;

  cout << endl;
  cout << "System Status: [ Control: " << (int)bd.ctrlInfo.data << " Device: " << (int)bd.ctrlInfo.device << " Status: " 						  			 << (int)bd.ctrlInfo.signature << ']' << endl;
	cout << "Position: [ Long: " << bd.pos.longitude << " Lati: " << bd.pos.latitude << " Alti: " << bd.pos.altitude << " Height: " 
			 << bd.pos.height << ']' << endl;
  cout << "Plane(world):  [ Roll: " << plane_angle_roll << " Pitch: " << plane_angle_pitch << " Yaw: " << plane_angle_yaw << " ]" << endl;
	cout << "Gimbal(world):[ Roll: " << bd.gimbal.roll << " Pitch: " << bd.gimbal.pitch << " Yaw: " << bd.gimbal.yaw << " ]" << endl;
	cout << "camera_yaw:[ yaw: " << cam_yaw << " ]" << endl;
  cout << endl;
}

//void *stop(void *argc)
//{
	//waiting_stop();
	//pthread_exit(0);
//}

/*member function for gimbal control*/
void Tracking_system::control_gimbal(int yaw,int roll,int pitch)
{
	DJI::onboardSDK::GimbalSpeedData a;
	a.yaw      = yaw;
	a.roll     = roll;
	a.pitch    = pitch;
	a.reserved = 0x80;
	getGimbal()->setGimbalSpeed(&a); 
}

void Tracking_system::start_position_control() {
	float x, y, z, yaw;
	cout << "Input position offset x y z yaw:";
	cin >> x >> y >> z >> yaw;
	boost::thread record(boost::bind(&Tracking_system::flight_control_RP,this,x,y,z,yaw));
}

/*member function for tracknig*/
void Tracking_system::start_approaching()
{
	stop_task = false;
	calc = true;
	//record_flag = true;
	//track_count++;
	boost::thread record(boost::bind(&Tracking_system::record,this));
	boost::thread counter(boost::bind(&Tracking_system::counter,this));
	boost::thread approach(boost::bind(&Tracking_system::approaching,this));
	//track.detach();
	//while(!done);
	//while(0);
	//done = false;
	//w_p = 0;
	//Ev_p[0] = 0;Ev_p[1] = 0;
}

void Tracking_system::record()
{
	struct tm *t;
	time_t tt;

	time(&tt);
	t = localtime(&tt);

	char s[100];
	sprintf(s, "%4d%02d%02d_%02d_%02d_%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
	string time(s);

	string filename1 = time + "_error.txt";
	string filename2 = time + "_velocity.txt";
	string filename3 = time + "_velocity.txt";
	std::ofstream ferror(filename1.c_str());
	std::ofstream fvelocity(filename2.c_str());
	std::ofstream fu(filename3.c_str());
	/*string filename2 = s + "roll_L.txt";
	string filename3 = s + "pitch_L.txt";
	string filename4 = s + "velocity_L.txt";
	string filename5 = s + "velocity_L_cal.txt";
	string filename6 = s + "targetcenter.txt";
	string filename7 = s + "distance.txt";
	char name1[100];
	char name2[100];
	char name3[100];
	char name4[100];
	char name5[100];
	char name6[100];
	char name7[100];
	filename1.copy(name1,filename1.length(),0);
	filename2.copy(name2,filename2.length(),0);
	filename3.copy(name3,filename3.length(),0);
  filename4.copy(name4,filename4.length(),0);
	filename5.copy(name5,filename5.length(),0);
	filename6.copy(name6,filename6.length(),0);
  filename7.copy(name7,filename7.length(),0);
	*(name1 + filename1.length()) = '\0';
	*(name2 + filename2.length()) = '\0';
	*(name3 + filename3.length()) = '\0';
	*(name4 + filename4.length()) = '\0';
	*(name2 + filename5.length()) = '\0';
	*(name3 + filename6.length()) = '\0';
	*(name4 + filename7.length()) = '\0';

	fyaw_er.open(name1);
	froll.open(name2);
	fpitch.open(name3);
	fvelocity.open(name4);
	fvelocity_cal.open(name5);
	//ftargetcenter.open(name6);
	fdistance.open(name7);
*/

	while(1)
	{
		while (!record_flag && !stop_task);
		if(stop_task) break;
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
/*
	fyaw_er.close();
	froll.close();
	fpitch.close();
	fvelocity.close();
	fvelocity_cal.close();
	//ftargetcenter.close();
	fdistance.close();
*/
}

void Tracking_system::counter() {
	int elapsedTime = 0;  // ms
	while (!stop_task) {
		usleep(1000);
		if (!calc) {
			elapsedTime += 1;
			if (elapsedTime >= intervalInMs) {
				calc = true;
				elapsedTime = 0;
			}
		}
	}
}

void Tracking_system::approaching()
{
  const uint8_t flag = 0x49;  //Velocity Control
  const float32_t posThresholdInM = posThresholdInCm / 100;

	time_t startTime, curTime;
	time(&startTime);

	bool above = false;  // whether fly to above

  // todo 设置PID参数
  PIDControl xController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);
  PIDControl yController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);
  PIDControl zController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);

	TrackWithDistance twd;
	//twd.getInitBBox();

	while (1) {

		// waiting for signal

		while (!calc && !stop_task);
		calc = false;

		if (stop_task) {
      std::cout << "\ntask stopped\n";
			flight_control(flag, 0, 0, 0, 0);
			break;
		}

		// calculate error

		float* coord = twd.getObjectCoordinate();

		bd = api->getBroadcastData();

		plane_angle_roll  = atan2(2.0 * (bd.q.q3 * bd.q.q2 + bd.q.q0 * bd.q.q1) , 1.0 - 2.0 * (bd.q.q1 * bd.q.q1 + bd.q.q2 * bd.q.q2)) * 180 /Pi;
		plane_angle_pitch = asin(2.0 * (bd.q.q2 * bd.q.q0 - bd.q.q3 * bd.q.q1)) * 180 / Pi;
		plane_angle_yaw   = atan2(2.0 * (bd.q.q3 * bd.q.q0 + bd.q.q1 * bd.q.q2) , - 1.0 + 2.0 * (bd.q.q0 * bd.q.q0 + bd.q.q1 * bd.q.q1)) * 180 / Pi;

		Mat error = ComputeCoordinate(plane_angle_roll, plane_angle_pitch, plane_angle_yaw,
		bd.gimbal.roll, bd.gimbal.pitch, bd.gimbal.yaw, bd.pos.height,coord[0], coord[1]);
		ex = error.at<float>(0, 0);
		ey = error.at<float>(1, 0);
		ez = error.at<float>(2, 0);

		cout << "ex:" << ex << " ey:" << ey << " ez:" << ez << "\r";
		record_flag = true;

		// check situation

		if (std::fabs(ex) <= posThresholdInM &&
				std::fabs(ey) <= posThresholdInM) {
			if (!above) {
				cout << "\nfinish fly to above\n";
				above = true;
			}
			if (std::fabs(ez) <= posThresholdInM)
				break;
		}

		// 计算控制量
		ux = xController(ex);
		uy = yController(ey);
		uz = zController(ez);

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

/*member function for flight control*/

void Tracking_system::flight_control_gh(){getFlight()->task((Flight::TASK)1);}

void Tracking_system::flight_control_tk(){getFlight()->task((Flight::TASK)4);}

void Tracking_system::flight_control_ld(){getFlight()->task((Flight::TASK)6);}

// todo
#define C_EARTH 0
#define DEG2RAD 0

struct Vector3dData {
	float x, y, z;
};

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

// todo finish the calculation of yaw 
void Tracking_system::flight_control_RP(float x, float y, float z, float yaw) {

	static const double yawThresholdInDeg = 5;

	const uint8_t flag = 0x49;  //Velocity Control

  double yawDesiredRad = DEG2RAD*yaw;
  double yawThresholdInRad = DEG2RAD*yawThresholdInDeg;
  const float32_t posThresholdInM = posThresholdInCm / 100;
  int elapsedTime = 0;
	float ux, uy, uz, uyaw;

  // Get current poition
  // PositionData curPosition = api->getBroadcastData().pos;
  PositionData originPosition = api->getBroadcastData().pos;
  Vector3dData curLocalOffset;
  Vector3dData error;

  // todo 设置PID参数
  PIDControl xController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);
  PIDControl yController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);
  PIDControl zController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);
  PIDControl yawController(1, 0, 0, maxSpeedInM, intervalInMs / 1000.0);

  while (true) {

    // 获取误差
    PositionData curPosition = api->getBroadcastData().pos;
    //DJI::EulerAngle curEuler = Flight::toEulerAngle(api->getBroadcastData().q);

    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
    error.x = x - curLocalOffset.x;
    error.y = y - curLocalOffset.y;
    error.z = z - curLocalOffset.z;
    //double errorYaw = yawDesiredRad - curEuler.yaw
    if (std::fabs(error.x) <= posThresholdInM &&
        std::fabs(error.y) <= posThresholdInM &&
        std::fabs(error.z) <= posThresholdInM //&&
        //std::fabs(errorYaw) <= yawThresholdInRad
				)
      break;

    // 计算控制量
    ux = xController(error.x);
    uy = yController(error.y);
    uz = zController(error.z);
    //uyaw = yawController(errorYaw);

    // 发送控制
    flight_control(flag, ux, uy, uz, //uyaw
										0);

    elapsedTime += intervalInMs;
    if (elapsedTime >= timeoutInS * 1000) {
      std::cout << "\nTimed out\n";
			flight_control(flag, 0, 0, 0, 0);
      return;
    }

  }
}

void Tracking_system::flight_control(uint8_t flag,float x,float y,float z,float yaw)
{
	FlightData fd;
	fd.flag = flag;
	fd.x = x;
	fd.y = y;
	fd.z = z;
	fd.yaw = yaw;
	getFlight()->setFlight(&fd);
}

/*void Tracking_system::flight_control_SC()
{
	
}*/

void Tracking_system::flight_control_sh()
{
	cout << "Enter data like this: height" << endl;;

	char * data = new char[20];
	cin.getline(data,20);
	int data_num = sscanf(data,"%d",&set_height);
	
	if(data_num != 1)
		cout << "Wrong input!" << endl;
	else
		while((set_height - api->getBroadcastData().pos.height) > 0.1 || (api->getBroadcastData().pos.height - set_height) > 0.1)
		{
			flight_control(0x5B,0,0,set_height,0);
			usleep(20000);
		}
}


void Tracking_system::setApi(CoreAPI * value){api = value;}

CoreAPI * Tracking_system::getApi() const{return api;}

void Tracking_system::setGimbal(Camera * value){camera = value;}

Camera * Tracking_system::getGimbal() const{return camera;}

void Tracking_system::setFlight(Flight * value){flight = value;}

Flight * Tracking_system::getFlight() const{return flight;}


























