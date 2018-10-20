#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <iostream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
using namespace std;

int main(int argc, char** argv){
	ros::init(argc, argv, "needle_broadcaster");
	ros::NodeHandle n;
        ros::Publisher position_pub = n.advertise<std_msgs::Float64MultiArray>("needle_position", 5,true);

	double tmp= 0;
	while (ros::ok()){
		std_msgs::Float64MultiArray position;
		position.data.clear();
		position.data.push_back(0);
		position.data.push_back(0);
		position.data.push_back(0);

		
                cout << "User input:" << endl;
                cin >>  tmp;
		position.data.push_back(tmp);
                cout << "User input:" << endl;
                cin >>  tmp;
		position.data.push_back(tmp);
		cout << "User input:" << endl;
                cin >>  tmp;
		position.data.push_back(tmp);
		position_pub.publish(position);
		cout << "Sent. To continue, enter any number." << endl;
		cin >> tmp;
		tmp=0;
		ros::spinOnce();
		sleep(1);
	}

	return 0;
}
