#ifndef DJI_TRACKING_H
#define DJI_TRACKING_H
#include <DJI_Config.h>
#include <DJI_Version.h>
#include <DJI_Flight.h>
#include <DJI_Follow.h>
#include <DJI_HotPoint.h>
#include <DJI_WayPoint.h>
#include <DJI_VirtualRC.h>
#include <DJI_Camera.h>
#include <DJI_API.h>

#include <iostream>
#include <string>

#define Pi 3.1415926

namespace DJI
{
	namespace onboardSDK
	{			 
		class Tracking_system
		{
			public:
				Tracking_system(CoreAPI * controlAPI,long int keyID,char * key);
				void cmd_match();
				void Display_manu();

			public:
				ActivateData adata;
				void activeAPI();
				void obtain_control();
				void release_control();
				void setApi(CoreAPI * value);
				CoreAPI * getApi() const;	

			public:				
				void control_gimbal(int yaw,int roll,int pitch);
				void setGimbal(Camera * value);
				Camera * getGimbal() const;

			public:
				void flight_control(uint8_t flag,float x,float y,float z,float yaw);
				void flight_control_gh();
				void flight_control_tk();
				void flight_control_ld();
				void flight_control_RP(float x, float y, float z, float yaw);
				void flight_control_sh();
				//void flight_control_SC();
				void setFlight(Flight * value);
				Flight * getFlight() const;

			public:
				void start_approaching();
				void start_position_control();

			public:
				void getbd();
				bool quit;

			private:
				Flight *          flight;
				Camera *          camera;
				CoreAPI *         api;

				static const int frequency = 40;
				static const int intervalInMs = 1000.0f / frequency;

				// parameters
				int timeoutInS;
				float posThresholdInCm;
				float maxSpeedInM;

				bool record_flag;  // signal to record once
				bool stop_task;
				bool calc;  // signal of the counter to run the code per frame

				float ux, uy, uz;
				float ex, ey, ez;  // error to record

				void record();
				void counter();
				void approaching();
		};
	}
}
#endif
