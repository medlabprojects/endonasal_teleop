// The code that's commented out was from a previous version where the communication time was tested
#include <iostream>
#include <sstream>
#include <math.h>
#include <HD\hd.h>
#include <HDU\hduVector.h>
#include <HDU\hduMatrix.h>
#include "stdafx.h"
#include "ros.h"
#include <string>
#include <stdio.h>
#include <windows.h>
#include <map>
#include <vector>
#include <functional>
#include <memory>
#include <signal.h>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <iterator>
#include "Windows.h"
#include <mutex>

// ROS messages
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"

using namespace std;
using namespace ros;
using std::string;

double position[3] = { 2, 2, 2 };
double transform[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
double quaternion[4] = { 0, 0, 0, 0 };
int force_flag = 1;
int button1_int;
int switch1_int;

int launches = 0;

void printline()
{
	std::cout << "****************************************************************************\n\n";

	return;
}
typedef struct
{
	HDboolean switch1;
	HDint button1;
} DeviceData;

static DeviceData omniDeviceData;

HDfloat currentForce[3] = { 0, 0, 0 };

LARGE_INTEGER starttime, Frequency, endtime, elapsedtime;

HHD hHD;
HHD hHD1;

void init_haptic_device()
{
	hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	HDErrorInfo error;
	error = hdGetError();

	if (HD_DEVICE_ERROR(error))  // checking the device status
	{
		string sss = hdGetErrorString(error.errorCode);
		printline();
		std::cout << "Error in device initialization. \n\n";
		std::cout << sss << "\n\n";
	}
	else
	{
		printline();
		if (launches < 1) { std::cout << "Device Successfully Connected and Initialized. \nPhantom Omni should now be reading and writing ROS messages.\n\n"; }
		else { std::cout << "Device will now restart...\n\n" << "Restart Number " << launches << " completed sucessfully. \nPhantom Omni should now be reading and writing ROS messages.\n\n"; }
		launches++;
	}

	hdEnable(HD_FORCE_OUTPUT); // enable force output

        HDboolean bForcesOn = hdIsEnabled(HD_FORCE_OUTPUT);// check enable force output status

	if (bForcesOn) { }
	else { printline();  std::cout << "An error has been encountered when enabling forces. \n" << std::endl;}

	hdStartScheduler();

	HHD hHD1 = hdGetCurrentDevice();
}

HDCallbackCode HDCALLBACK DeviceStateCallback(void *pUserdata)
{
	try
	{
		int nButtons = 0;

		hdBeginFrame(hHD1);

		hdSetFloatv(HD_CURRENT_FORCE, currentForce);
		
		hdGetBooleanv(HD_CURRENT_SAFETY_SWITCH, &omniDeviceData.switch1);

		hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);

		omniDeviceData.button1 =
			(nButtons & HD_DEVICE_BUTTON_4) ? HD_TRUE : HD_FALSE;

		hdGetDoublev(HD_CURRENT_POSITION, position);
		hdGetDoublev(HD_CURRENT_TRANSFORM, transform);

		hdEndFrame(hHD1);

		// Convert from matrix to quaternion

		double R00 = transform[0]; double R01 = transform[1]; double R02 = transform[2];
		double R10 = transform[4]; double R11 = transform[5]; double R12 = transform[6];
		double R20 = transform[8]; double R21 = transform[9]; double R22 = transform[10];

		quaternion[3] = 0.5*sqrt(1 + R00 + R11 + R22);
		quaternion[0] = (R21 - R12) / (4 * quaternion[3]);
		quaternion[1] = (R02 - R20) / (4 * quaternion[3]);
		quaternion[2] = (R10 - R01) / (4 * quaternion[3]);

		HDErrorInfo error;
		error = hdGetError();

		if (HD_DEVICE_ERROR(error))  // checking the device status
		{
			printline();
			std::cout << "An exception has occurred in communication with the Omni Controller. \nAn error from the controller API follows...\n\n";
			std::string sss = hdGetErrorString(error.errorCode);
			std::cout << sss << "\n\n";
			throw 1;
		}
	}
	catch (int param)
	{
		init_haptic_device();
	}

	return HD_CALLBACK_DONE;

}

void force_callback(const geometry_msgs::Vector3 & force_information)
{
	// Set file global to the current subscribed force.
	currentForce[0] = force_information.x;
	currentForce[1] = force_information.y;
	currentForce[2] = force_information.z;

	return;

}

int _tmain(int argc, _TCHAR * argv[])
{
	// ROS node initialization
	NodeHandle nh;
	char *ros_master = "192.168.0.1";
	printline();
	printf("Connecting to server at %s\n\n", ros_master);
	nh.initNode(ros_master);

	// Setting up publishers
	std_msgs::Int8 button_msg;
	Publisher button_state_pub("Buttonstates", &button_msg);
	nh.advertise(button_state_pub);

	geometry_msgs::Pose ROS_pos;
	Publisher omni_position_pub ("Omnipos", &ROS_pos); // this publishes the ROS_pos messages to the Omnipos topic
	nh.advertise(omni_position_pub);

	// Setting up subscriber 
	ros::Subscriber<geometry_msgs::Vector3> omni_force_sub("Omniforce", &force_callback);
	nh.subscribe(omni_force_sub);
	nh.subscribe(omni_force_sub);

	init_haptic_device();

	while (1){
		
		hdScheduleSynchronous(DeviceStateCallback, (void*)0, HD_DEFAULT_SCHEDULER_PRIORITY);

		// Prepare twist to be sent to omnipos topic through rosserial
		ROS_pos.position.x = position[0];
		ROS_pos.position.y = position[1];
		ROS_pos.position.z = position[2];
		ROS_pos.orientation.x = quaternion[0];
		ROS_pos.orientation.y = quaternion[1];
		ROS_pos.orientation.z = quaternion[2];
		ROS_pos.orientation.w = quaternion[3];

		// There's some nonsense going on here, but it works...
		switch1_int = omniDeviceData.switch1 + '0' - 48;
		button_msg.data = switch1_int;

		// Publish to the ROS network
		button_state_pub.publish(&button_msg);
		omni_position_pub.publish(&ROS_pos);
		nh.spinOnce();
	}

	return 0;
}
