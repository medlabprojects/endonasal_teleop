// endonasal_teleop.cpp
// This is the main script that will run the endonasal system. 
// This is currently configured to launch 3 CTR3Robots

// NOTE: most of this code is from the experiment resolved_rates.cpp file, and used for reference for the ResolvedRatesController.cpp class
// TODO: refactor this to:
//  --> initialize all ROS-related things (cleanup topics,messages & nodes as most of that message traffic is accounted for by the new classes
//  --> initialize 3 instances of the CTR3Robot class -> read from parameter server or xml file?
//  --> initialize 3 instances of the ResolvedRatesController class and link to the appropriate CTR3Robot
//  --> initialize however many controllers we have (omni nodes?) and do processing in THIS script to handle transformations, and feed ResovledRatesController class the desTwist
//  --> handle visualization by pulling from CTR3Robot class's current interpolated backbone and publishing pts as markers or tubes? (Snare system using tubes instead of markers..)
// -> for all references to math/robotics math functions, use namespace RobotMath::deg2rad() for ex. and just #include "RobotMath.h"
// -> still need to implement haptic damper on omninode side..needs faster update rate to feel good

// Qt headers
#include <QCoreApplication>
#include <QVector>

// Cannula kinematics headers
//#include <Kinematics.h>
#include <BasicFunctions.h>
#include <Tube.h>

#include "MedlabTypes.h"
#include "RoboticsMath.h"
#include "CTR3Robot.h"
#include "ResolvedRatesController.h"

// Eigen headers
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>

//XML parsing headers
#include "rapidxml.hpp"

// Message & service headers
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Int32.h>
#include "std_msgs/Int8.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <endonasal_teleop/matrix6.h>
#include <endonasal_teleop/matrix8.h>
#include <endonasal_teleop/config3.h>
#include <endonasal_teleop/vector7.h>
#include <endonasal_teleop/kinout.h>
#include <endonasal_teleop/getStartingConfig.h>
#include <endonasal_teleop/getStartingKin.h>
#include <geometry_msgs/Vector3.h>

#include "medlab_motor_control_board/McbEncoders.h"

// Misc. headers
#include <Mtransform.h>
#include <bits/stdc++.h>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "spline.h"
#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <cmath>

// ---------------------------------------------------------------
// DEF GLOBALS
// ---------------------------------------------------------------

// Motor Enable
int motorControlState = 0;

// Visuzalization
endonasal_teleop::matrix8 markersMsg;

double rosLoopRate = 100.0;

// Omni
Eigen::Matrix4d omniPose;
Eigen::Matrix4d prevOmni;
Eigen::Matrix4d curOmni;
Eigen::Matrix4d Tregs;
Eigen::Matrix4d OmniDeltaOmniCoords;
Eigen::Matrix4d prevOmniInv;
Eigen::Matrix4d omniDeltaCannulaCoords;
Eigen::Matrix4d omniFrameAtClutch;
double omniScaleFactor = 0.30;

Eigen::Matrix4d robotTipFrame;
Eigen::Matrix4d robotTipFrameAtClutch; //clutch-in position of cannula
Eigen::Vector3d pTipCur;
Eigen::Vector4d qTipCur;
Eigen::Vector4d qBishopCur;
Eigen::Vector3d alphaCur;

Eigen::Vector3d pTip;
Eigen::Vector4d qTip;
Eigen::Vector4d qBishop;
Eigen::Vector3d alpha;
Eigen::Matrix3d Rtip;
Eigen::Matrix3d RBishop;

geometry_msgs::Pose tempMsg;
void omniCallback(const geometry_msgs::Pose &msg)
{
	tempMsg = msg;

	Eigen::Vector4d qOmni;
	qOmni << tempMsg.orientation.w, tempMsg.orientation.x, tempMsg.orientation.y, tempMsg.orientation.z;
        Eigen::Matrix3d ROmni = RoboticsMath::quat2rotm(qOmni);

	Eigen::Vector3d pOmni;
	pOmni << tempMsg.position.x, tempMsg.position.y, tempMsg.position.z;

	omniPose.fill(0);
	omniPose.topLeftCorner(3, 3) = ROmni;
	omniPose.topRightCorner(3, 1) = pOmni;
	omniPose(3, 3) = 1.0;

}

int buttonState = 0;
int buttonStatePrev = 0;
bool justClutched = false;
void omniButtonCallback(const std_msgs::Int8 &buttonMsg)
{
	buttonStatePrev = buttonState;
	buttonState = static_cast<int>(buttonMsg.data);
	if (buttonState == 1 && buttonStatePrev == 0)
	{
		justClutched = true;
	}
}

RoboticsMath::Vector6d InputDeviceTwist(Eigen::Matrix4d omniDeltaOmniPenCoords)
{
  // TODO: need to be able to parse device number


  Eigen::Vector3d zeroVec;
  zeroVec.fill(0);

    omniDeltaOmniPenCoords.block(0,3,3,1) = omniDeltaOmniPenCoords/1.0E3;
    omniDeltaOmniPenCoords.block(0,3,3,1) = omniScaleFactor*omniDeltaOmniPenCoords.block(0,3,3,1);
    Eigen::Matrix4d RPrevOmni = RoboticsMath::assembleTransformation(prevOmni.block(0,0,3,3),zeroVec);
    Eigen::Matrix4d omniDeltaOmniBaseCoords = (Mtransform::Inverse(RPrevOmni.transpose())*omniDeltaOmniPenCoords*RPrevOmni.transpose());

    Eigen::Matrix3d Rd = curOmni.block<3,3>(0,0);
    Eigen::Matrix3d Rc = prevOmni.block<3,3>(0,0);
    Eigen::Matrix3d Re = Rd*Rc.transpose();
    Re = RoboticsMath::orthonormalize(Re);

    // desTwist should use OmniBaseCoords for linar velocity, and OmniPenCoords for angular velocity
    RoboticsMath::Vector6d omniTwistOmniPenCoords;
    omniTwistOmniPenCoords(0) = omniDeltaOmniPenCoords(0,3); // vx
    omniTwistOmniPenCoords(1) = omniDeltaOmniPenCoords(1,3); // vy
    omniTwistOmniPenCoords(2) = omniDeltaOmniPenCoords(2,3); // vz
    omniTwistOmniPenCoords(3) = omniDeltaOmniPenCoords(2,1); // wx
    omniTwistOmniPenCoords(4) = omniDeltaOmniPenCoords(0,2); // wy
    omniTwistOmniPenCoords(5) = omniDeltaOmniPenCoords(1,0); // wz

    RoboticsMath::Vector6d omniTwistOmniBaseCoords;
    omniTwistOmniBaseCoords(0) = omniDeltaOmniBaseCoords(0,3);
    omniTwistOmniBaseCoords(1) = omniDeltaOmniBaseCoords(1,3);
    omniTwistOmniBaseCoords(2) = omniDeltaOmniBaseCoords(2,3);
    omniTwistOmniBaseCoords(3) = omniDeltaOmniBaseCoords(2,1);
    omniTwistOmniBaseCoords(4) = omniDeltaOmniBaseCoords(0,2);
    omniTwistOmniBaseCoords(5) = omniDeltaOmniBaseCoords(1,0);

    RoboticsMath::Vector6d desTwist;
    desTwist.head(3) = omniTwistOmniBaseCoords.head(3);
    desTwist.tail(3) = omniTwistOmniPenCoords.tail(3);

    return desTwist;
}

int main(int argc, char *argv[])
{

  // Omni Registration Vars
  Eigen::Matrix4d OmniReg = Eigen::Matrix4d::Identity();
  Eigen::MatrixXd RotY = Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitY()).toRotationMatrix();
  Mtransform::SetRotation(OmniReg,RotY);
  //Eigen::Matrix4d OmniRegInv = Mtransform::Inverse(OmniReg);
  Eigen::Vector3d zeroVec;
  zeroVec.fill(0);

  // Start this ROS node on network
  ros::init(argc, argv, "endonasal_teleop");
  ros::NodeHandle node;

  // -------------------------------------------------------------------------------
  // CTR3ROBOT 1 DEFINITION

  medlab::CTR3RobotParams robot1Params;
  robot1Params.E = 60E9;
  robot1Params.G = 60E9 / 2.0 / 1.33;
  robot1Params.L1 = 222.5E-3;
  robot1Params.Lt1 = robot1Params.L1 - 42.2E-3;
  robot1Params.k1 = 1.0/63.5E-3;
  robot1Params.OD1 = 1.165E-3;
  robot1Params.ID1 = 1.067E-3;
  robot1Params.L2 = 163E-3;
  robot1Params.Lt2 = robot1Params.L2 - 38.0E-3;
  robot1Params.k2 = 1.0/51.2E-3;
  robot1Params.OD2 = 2.0574E-3;
  robot1Params.ID2 = 1.6002E-3;
  robot1Params.L3 = 104.4E-3;
  robot1Params.Lt3 = robot1Params.L3 - 21.3E-3;
  robot1Params.k3 = 1.0/71.4E-3;
  robot1Params.OD3 = 2.540E-3;
  robot1Params.ID3 = 2.248E-3;

  typedef CTR::Tube< CTR::Functions::constant_fun< CTR::Vector<2>::type> >  TubeType;

  // Curvature of each tube
  CTR::Functions::constant_fun< CTR::Vector<2>::type > k_fun1( (robot1Params.k1)*Eigen::Vector2d::UnitX() );
  CTR::Functions::constant_fun< CTR::Vector<2>::type > k_fun2( (robot1Params.k2)*Eigen::Vector2d::UnitX() );
  CTR::Functions::constant_fun< CTR::Vector<2>::type > k_fun3( (robot1Params.k3)*Eigen::Vector2d::UnitX() );

  // Define tubes
  TubeType T1 = CTR::make_annular_tube( robot1Params.L1, robot1Params.Lt1, robot1Params.OD1, robot1Params.ID1, k_fun1, robot1Params.E, robot1Params.G );
  TubeType T2 = CTR::make_annular_tube( robot1Params.L2, robot1Params.Lt2, robot1Params.OD2, robot1Params.ID2, k_fun2, robot1Params.E, robot1Params.G );
  TubeType T3 = CTR::make_annular_tube( robot1Params.L3, robot1Params.Lt3, robot1Params.OD3, robot1Params.ID3, k_fun3, robot1Params.E, robot1Params.G );

  // Assemble cannula
  auto cannula = std::make_tuple( T1, T2, T3 );

  // Initialize robot with controller
  ResolvedRatesController rr1(cannula,robot1Params);
  rr1.init();

  // -------------------------------------------------------------------------------
  //        std::cout << "pTip" << robot1.currKinematics.Ptip << std::endl << std::endl;
  //        std::cout << "qTip" << robot1.currKinematics.Qtip << std::endl << std::endl;
  //        std::cout << "Stability" << robot1.currKinematics.Stability << std::endl << std::endl;
  //        std::cout << "Jtip" << robot1.currKinematics.Jtip << std::endl << std::endl;

  //        McbRos* motorBoard1;
  //        std::string motorBoard1NodeName = ui_.lineEdit_nodeName->text().toStdString();
  //        motorBoard1->init(motorBoard1NodeName);

  // -------------------------------------------------------------------------------
  // SUBSCRIBERS
  ros::Subscriber omniButtonSub = node.subscribe("Buttonstates", 1, omniButtonCallback);
  ros::Subscriber omniPoseSub = node.subscribe("Omnipos", 1, omniCallback);

  // PUBLISHERS
  ros::Publisher needle_pub = node.advertise<endonasal_teleop::matrix8>("needle_position",10);
  ros::Publisher pubEncoderCommand1 = node.advertise<medlab_motor_control_board::McbEncoders>("MCB1/encoder_command", 1); // EC13
  ros::Publisher pubEncoderCommand2 = node.advertise<medlab_motor_control_board::McbEncoders>("MCB4/encoder_command", 1); // EC16
  //clients
  // ROS RATE
  ros::Rate r(rosLoopRate);

  // -------------------------------------------------------------------------------
  // LOOP!
  // -------------------------------------------------------------------------------
  while (ros::ok())
  {

    // get Omni inputs
    // decide which are active/mapped to which tools
    // compute desired twist in whatever frame
    // send ResolvedRates.step(desiredTwist) output to mapped motors >> mcb->setDesiredPosition()

    curOmni = omniPose;
    pTip = pTipCur;
    qTip = qTipCur;
    qBishop = qBishopCur;
    alpha = alphaCur;
    Rtip = RoboticsMath::quat2rotm(qTip);
    RBishop = RoboticsMath::quat2rotm(qBishop);
    robotTipFrame = RoboticsMath::assembleTransformation(Rtip,pTip);

    if (buttonState == 1)   // while clutched in
    {
      if (motorControlState == 1)  // If trying to actually drive the hardware
      {
        // [q_vec_new] = rr1.step(q_vec_curr,omni_desTwist,controlType);
        // pubEncoderCommand1.publish(enc1);...etc.
      }

      if (justClutched == true) // Set reference to this first frame
      {
        omniFrameAtClutch = omniPose;
        prevOmni = omniPose;
        Tregs = RoboticsMath::assembleTransformation(RBishop.transpose(),zeroVec); // TODO: need to make sure this gets updated
        justClutched = false;
      }

      Eigen::Matrix4d omniDeltaOmniPenCoords = Mtransform::Inverse(prevOmni)*curOmni;

      RoboticsMath::Vector6d desTwist;
      desTwist = InputDeviceTwist(omniDeltaOmniPenCoords);

    }

    //// Publish visualizations to rviz

    CTR3Robot robot1 = rr1.GetRobot();
    medlab::InterpRet robot1Backbone = robot1.GetInterpolatedBackbone();

    int nPts = robot1.GetNPts();
    int nInterp = robot1.GetNInterp();

    for (int j=0; j<(nInterp+nPts); j++)
    {
      markersMsg.A1[j]=robot1Backbone.p(0,j); // X
      markersMsg.A2[j]=robot1Backbone.p(1,j); // Y
      markersMsg.A3[j]=robot1Backbone.p(2,j); // Z
      markersMsg.A4[j]=robot1Backbone.q(0,j); // w
      markersMsg.A5[j]=robot1Backbone.q(1,j); // x
      markersMsg.A6[j]=robot1Backbone.q(2,j); // y
      markersMsg.A7[j]=robot1Backbone.q(3,j); // z

      RoboticsMath::Vector6d robot1CurQVec = robot1.GetCurrQVec();
      Eigen::Vector3d robot1CurBeta = robot1CurQVec.bottomRows(3);
      if (robot1CurBeta[1] > robot1Backbone.s[j] ||
          robot1Params.L2+robot1CurBeta[1] < robot1Backbone.s[j])
      {
        markersMsg.A8[j] = 1; // green
      }
      else if ((robot1CurBeta[1] <= robot1Backbone.s[j] && robot1CurBeta[2] > robot1Backbone.s[j]) ||
               (robot1Params.L3 + robot1CurBeta[2] < robot1Backbone.s[j] && robot1Params.L2 + robot1CurBeta[1] >= robot1Backbone.s[j]))
      {
        markersMsg.A8[j] = 2; // red
      }
      else
      {
        markersMsg.A8[j] = 3; // blue
      }
    }

    needle_pub.publish(markersMsg);

    // sleep
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
