#include "DJI_tracking.h"
//#include "../kcfsrc/runtracker.h"
//#include "Flightcontrol.h"
//#include "Gimbalcontrol.h"
#include "../kcfsrc/TrackWithDistance.h"

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

#ifdef __linux__
#include <sys/select.h>

int kbhit()
{
	struct timeval tv;
	fd_set read_fd;

	/* Do not wait at all, not even a microsecond */
	tv.tv_sec = 0;
	tv.tv_usec = 0;

	/* Must be done first to initialize read_fd */
	FD_ZERO(&read_fd);

	/* Makes select() ask if input is ready:
	 * 0 is the file descriptor for stdin    */
	FD_SET(0, &read_fd);

	/* The first parameter is the number of the
	 * largest file descriptor to check + 1. */
	if (select(1, &read_fd, NULL, /*No writes*/NULL, /*No exceptions*/&tv) == -1)
		return 0;  /* An error occured */

	/*  read_fd now holds a bit map of files that are
	 * readable. We test the entry for the standard
	 * input (file 0). */

	if (FD_ISSET(0, &read_fd))
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
Tracking_system::Tracking_system(CoreAPI * controlAPI, long int keyID, char * key)
{
	adata.ID = keyID;
	adata.reserved = 2;
	adata.version = SDK_VERSION;
	adata.encKey = key;

	api = controlAPI;

	quit = false;

	camera = new Camera(api);
	flight = new Flight(api);

	fetch.set_api(api);
	fetch.set_flight(flight);
	fetch.set_camera(camera);
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
	getline(cin, inputCmd);
	for (int i = 0; i < sizeof(commandList) / sizeof(commandList[0]); i++)
	{
		if (inputCmd == commandList[i])
		{
			command_num = i;
			command_match = true;
			break;
		}
	}

	if (!command_match)
	{
		cout << "Wrong input!" << endl;
	}
	else
	{
		float x, y, z, yaw, input;
		int num = 0;
		switch (command_num)
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
			control_gimbal(200, 0, 0);
			break;
		case 4:
			control_gimbal(-200, 0, 0);
			break;
		case 5:
			control_gimbal(0, 0, 200);
			break;
		case 6:
			control_gimbal(0, 0, -200);
			break;
		case 7:
			getbd();
			break;
		case 8:
			flight_control_tk();
			break;
		case 9:
			flight_control_gh();
			break;
		case 10:
			flight_control_ld();
			break;
		case 11:
			cout << "Input position offset x y z yaw:";
			cin >> x >> y >> z >> yaw;
			fetch.start_position_control(x, y, z, yaw);
			cin.sync();
			cin.clear();
			break;
		case 12:
			fetch.start_approaching();
			break;
		case 13:
			fetch.stop();
			break;
		case 14:
			flight_control_sh();
			break;
		case 15:
			fetch.stop();
			quit = true;
			break;
		case 16:
			cout << "select parameter:\n1.timeoutInS\n2.posThresholdInCm\n3.maxSpeedInM\n4.yawThresholdInDeg\n5.maxYawSpeedInDeg\n6.frequency\n";
			cin >> num >> input;
			switch (num) {
			case 1:
				fetch.set_timeoutInS(input);
				break;
			case 2:
				fetch.set_posThresholdInCm(input);
				break;
			case 3:
				fetch.set_maxSpeedInM(input);
				break;
			case 4:
				fetch.set_yawThresholdInDeg(input);
				break;
			case 5:
				fetch.set_maxYawSpeedInDeg(input);
				break;
			case 6:
				fetch.set_frequency(input);
				break;
			}
			cin.sync();
			cin.clear();
		}
	}

}

/*member function for CoreAPI*/
void Tracking_system::activeAPI() {getApi()->activate(&adata);}

void Tracking_system::obtain_control() {getApi()->setControl(true);}

void Tracking_system::release_control() {getApi()->setControl(false);}

void Tracking_system::getbd()
{
	bd = api->getBroadcastData();
	double cam_yaw;

	plane_angle_roll  = atan2(2.0 * (bd.q.q3 * bd.q.q2 + bd.q.q0 * bd.q.q1) , 1.0 - 2.0 * (bd.q.q1 * bd.q.q1 + bd.q.q2 * bd.q.q2)) * 180 / Pi;
	plane_angle_pitch = asin(2.0 * (bd.q.q2 * bd.q.q0 - bd.q.q3 * bd.q.q1)) * 180 / Pi;
	plane_angle_yaw   = atan2(2.0 * (bd.q.q3 * bd.q.q0 + bd.q.q1 * bd.q.q2) , - 1.0 + 2.0 * (bd.q.q0 * bd.q.q0 + bd.q.q1 * bd.q.q1)) * 180 / Pi;

	if (plane_angle_yaw > 90 && bd.gimbal.yaw < -90)
		cam_yaw = 360 - plane_angle_yaw + bd.gimbal.yaw;
	else if (plane_angle_yaw < -90 && bd.gimbal.yaw > 90)
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
void Tracking_system::control_gimbal(int yaw, int roll, int pitch)
{
	DJI::onboardSDK::GimbalSpeedData a;
	a.yaw      = yaw;
	a.roll     = roll;
	a.pitch    = pitch;
	a.reserved = 0x80;
	getGimbal()->setGimbalSpeed(&a);
}

/*member function for flight control*/

void Tracking_system::flight_control_gh() {getFlight()->task((Flight::TASK)1);}

void Tracking_system::flight_control_tk() {getFlight()->task((Flight::TASK)4);}

void Tracking_system::flight_control_ld() {getFlight()->task((Flight::TASK)6);}

void Tracking_system::flight_control(uint8_t flag, float x, float y, float z, float yaw)
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
	cin.getline(data, 20);
	int data_num = sscanf(data, "%d", &set_height);

	if (data_num != 1)
		cout << "Wrong input!" << endl;
	else
		while ((set_height - api->getBroadcastData().pos.height) > 0.1 || (api->getBroadcastData().pos.height - set_height) > 0.1)
		{
			flight_control(0x5B, 0, 0, set_height, 0);
			usleep(20000);
		}
}


void Tracking_system::setApi(CoreAPI * value) {api = value;}

CoreAPI * Tracking_system::getApi() const {return api;}

void Tracking_system::setGimbal(Camera * value) {camera = value;}

Camera * Tracking_system::getGimbal() const {return camera;}

void Tracking_system::setFlight(Flight * value) {flight = value;}

Flight * Tracking_system::getFlight() const {return flight;}


























