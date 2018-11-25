#include <QCoreApplication>
#include "Kinematics.h"
#include "BasicFunctions.h"
#include "Tube.h"
#include <iostream>
#include <random>
#include <vector>
#include <QVector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "spline.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <ros/console.h>

// custom messages defined in /msg
#include <endonasal_teleop/matrix8.h>

bool new_message=0;
endonasal_teleop::matrix8 Arr;
tf::Transform t;
int length=0; // Number of points/frames
int ID=0;
double gap=0.8;
uint32_t shape = visualization_msgs::Marker::CYLINDER;
int rosVisRate = 1500;

void Callback(const endonasal_teleop::matrix8& msg)
{
    length = 0;
    // If a message arrives, the while loop that plots the curve will start
    new_message=1;
    Arr = msg;
    // Count the number of frames arrving with the msg
    // Assumimg length would not exceed the size of the array --> TODO: memory inefficient w/ msg w/ 8 fields, all at 500xfloat64
    while ((sqrt(Arr.A4[length]*Arr.A4[length]+Arr.A5[length]*Arr.A5[length]+Arr.A6[length]*Arr.A6[length]+Arr.A7[length]*Arr.A7[length])-0)>0.0001)
    {
        length=length+1;
    }

    return;
}
visualization_msgs::Marker ComputeMarkerPoseAndColor(int i)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();

  //maker sent with the same namespace and id will overwrite
  marker.ns = "basic_shapes";
  marker.id = ID;
  marker.type = shape;

  if(length<i)
  {
      marker.action = visualization_msgs::Marker::DELETE;
  }
  else
  {
      // Set marker action. ADD/DELETE/DELETEALL
      marker.action = visualization_msgs::Marker::ADD;

      // Calulate the distance between this and the next marker

      if (i<length-1)
      {
          gap=sqrt((Arr.A1[i]-Arr.A1[i+1])*(Arr.A1[i]-Arr.A1[i+1])+(Arr.A2[i]-Arr.A2[i+1])*(Arr.A2[i]-Arr.A2[i+1])+(Arr.A3[i]-Arr.A3[i+1])*(Arr.A3[i]-Arr.A3[i+1]));
      }

      if (gap<0.00001)
      {
          gap=0.00001;
      }

      // Set the pose of the marker
      // Since the cylinder marker "grows" in both direction, need to displace the first marker
      if (i==0){
          // basically (0,0,1) rotated by the quaternion then times half the gap is the displaced amount
          marker.pose.position.x = Arr.A1[i]+gap/2*(2*Arr.A4[i]*Arr.A6[i]-2*Arr.A7[i]*Arr.A5[i]);
          marker.pose.position.y = Arr.A2[i]+gap/2*(2*Arr.A4[i]*Arr.A5[i]+2*Arr.A6[i]*Arr.A7[i]);
          marker.pose.position.z = Arr.A3[i]+gap/2*(1+2*Arr.A5[i]*Arr.A5[i]-2*Arr.A6[i]*Arr.A6[i]);
      }
      else
      {
          marker.pose.position.x = Arr.A1[i];
          marker.pose.position.y = Arr.A2[i];
          marker.pose.position.z = Arr.A3[i];
      }

      marker.pose.orientation.w = Arr.A4[i]; // convention wxyz
      marker.pose.orientation.x = Arr.A5[i];
      marker.pose.orientation.y = Arr.A6[i];
      marker.pose.orientation.z = Arr.A7[i];


      //                  // generaly make the length of the cylinders twice the gaps
      // for the first cylinder, the length should equal to the gap
      // make the length of the last marker arbitrarily small
      if (i==length)
      {
          marker.scale.z = 0.00000005;
      }
      else if (i==1)
      {
          marker.scale.z = gap;
      }
      else
      {
          marker.scale.z = gap*2;
      }
      // Set the color of the marker
      if (Arr.A8[i]-1 < 0.0001)
      {
          marker.color.r = 0.0f;  // inner tube = green
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;
          marker.color.a = 1.0;

          marker.scale.x = 1.168e-3; // width of inner tube
          marker.scale.y = 1.168e-3;
      }
      else if (Arr.A8[i]-2 < 0.0001)
      {
          marker.color.r = 1.0f; // middle tube = red
          marker.color.g = 0.0f;
          marker.color.b = 0.0f;
          marker.color.a = 1.0;

          marker.scale.x = 1.684e-3; // width of middle tube
          marker.scale.y = 1.684e-3;
      }
      else
      {
          marker.color.r = 0.0f; // outer tube = blue;
          marker.color.g = 0.0f;
          marker.color.b = 1.0f;
          marker.color.a = 1.0;

          marker.scale.x = 2.324e-3; // width of outer tube
          marker.scale.y = 2.324e-3;
      }
  }
  return marker;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "workspace_display");
    ros::NodeHandle n;

    // use the tf library to broadcast tf frames to Rviz
    static tf::TransformBroadcaster br;

    // publishers
    ros::Publisher shape_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
//    ros::Publisher seg_pub = n.advertise<visualization_msgs::Marker>("segment_visual",1000);

    // subscribers
    ros::Subscriber sub = n.subscribe("needle_position", 1000, Callback);
    ros::Rate r(rosVisRate);

//    int seg_ID=10000;
    int prevLength=length;


    while (ros::ok())
    {
        // STL VISUALIZATION
//        visualization_msgs::Marker seg;
//        seg.header.frame_id = "world";
//        seg.header.stamp = ros::Time::now();

//        seg.ns = "seg_namespace";
//        seg.id = seg_ID;

//        seg.type = visualization_msgs::Marker::MESH_RESOURCE;
//        seg.action = visualization_msgs::Marker::ADD;

//        seg.pose.position.x = -0.001;
//        seg.pose.position.y = -0.4;
//        seg.pose.position.z = 0.2;

//        //change x and y to rotate around x-axis
//        seg.pose.orientation.x = 0.3;
//        seg.pose.orientation.y = -1.0;
//        seg.pose.orientation.z = 0.3;
//        seg.pose.orientation.w = -1.0;

//        seg.scale.x = 1.0;
//        seg.scale.y = 1.0;
//        seg.scale.z = 1.0;

//        seg.color.a = 1.0;
//        seg.color.r = 1.0;
//        seg.color.g = 0.0;
//        seg.color.b = 0.0;

//        //seg.mesh_resource = "package://endonasal_teleop/src/everythingSmoothedShrink.stl";

//        seg.lifetime = ros::Duration();
//        seg_pub.publish(seg);

        if (new_message) // if needle pose is updated, plot it
        {
            for (int i=0; i <prevLength; i++)
            {
                visualization_msgs::Marker marker = ComputeMarkerPoseAndColor(i);

                if (marker.pose.position.z > 0) // only publish markers in front of plate
                {
                  // Publish marker
                  std::cout<<"published"<<std::endl;
                  shape_pub.publish(marker);
                  ID=ID+1;
                  if (ID==length)
                  {
                      ID=0;
                      length=0;
                      new_message=false;
                  }
                }
            }

            prevLength = length;
        }

        // send transform
        br.sendTransform(tf::StampedTransform(t,ros::Time::now(),"world","tip_pose"));

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
