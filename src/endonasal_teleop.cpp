// endonasal_teleop.cpp
// This is the main script that will run the endonasal system. 
// This is currently configured to launch 3 CTR3Robots

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

// Motor Enable
int motorControlState = 0; // TODO: handle this with the state machine (ACTIVE STATE)

// Visuzalization
endonasal_teleop::matrix8 markersMsg;

double rosLoopRate = 200.0;

// Omni Vars
Eigen::Matrix4d omniPose;
Eigen::Matrix4d prevOmni;
Eigen::Matrix4d curOmni;
double omniScaleFactor = 0.30;
Eigen::Vector3d zeroVec = Eigen::Vector3d::Zero();

// RobotParams
//medlab::CTR3RobotParams robot1Params;
//medlab::CTR3RobotParams robot2Params;
//medlab::CTR3RobotParams robot3Params;

// TODO: omni message has been reformatted to stream Matrix4d & button msgs -> modify to read this in
geometry_msgs::Pose tempMsg;
void omniCallback(const geometry_msgs::Pose &msg) // TODO: refactor this to not use tempMsg...
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

// TODO: add to ResolvedRates class
// ::setInputDeviceTransform(Matrix4d transform)
// Vector6d ResolvedRates::transformInputDeviceTwist(Eigen::Matrix4d inputDeviceCoords){return RoboticsMath::Vector6d::Zero()};
//RoboticsMath::Vector6d ResolvedRates::InputDeviceTwist(Eigen::Matrix4d omniDeltaOmniPenCoords)
RoboticsMath::Vector6d InputDeviceTwistUpdate()
{
  // This uses the global curOmni and prevOmni vars to update desTwist
  Eigen::Matrix4d omniDeltaOmniPenCoords = Mtransform::Inverse(prevOmni)*curOmni;

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

endonasal_teleop::matrix8 GenerateRobotVisualizationMarkers(CTR3Robot robot)
{
  //// Publish visualizations to rviz

  medlab::InterpRet robotBackbone = robot.GetInterpolatedBackbone();

  int nPts = robot.GetNPts();
  int nInterp = robot.GetNInterp();

  for (int j=0; j<(nInterp+nPts); j++)
  {
    markersMsg.A1[j]=robotBackbone.p(0,j); // X,Y,Z
    markersMsg.A2[j]=robotBackbone.p(1,j);
    markersMsg.A3[j]=robotBackbone.p(2,j);
    markersMsg.A4[j]=robotBackbone.q(0,j); // w,x,y,z
    markersMsg.A5[j]=robotBackbone.q(1,j);
    markersMsg.A6[j]=robotBackbone.q(2,j);
    markersMsg.A7[j]=robotBackbone.q(3,j);

    medlab::CTR3RobotParams robotParams = robot.GetCurRobotParams();
    RoboticsMath::Vector6d robotCurQVec = robot.GetCurrQVec();
    Eigen::Vector3d robotCurBeta = robotCurQVec.bottomRows(3);
    if (robotCurBeta[1] > robotBackbone.s[j] ||
        robotParams.L2+robotCurBeta[1] < robotBackbone.s[j])
    {
      markersMsg.A8[j] = 1; // green
    }
    else if ((robotCurBeta[1] <= robotBackbone.s[j] && robotCurBeta[2] > robotBackbone.s[j]) ||
             (robotParams.L3 + robotCurBeta[2] < robotBackbone.s[j] && robotParams.L2 + robotCurBeta[1] >= robotBackbone.s[j]))
    {
      markersMsg.A8[j] = 2; // red
    }
    else
    {
      markersMsg.A8[j] = 3; // blue
    }
  }

  return markersMsg;
}

medlab::CTR3RobotParams GetRobot1ParamsFromServer()
{
  // LOAD PARAMETER SERVER
  if (ros::param::has("/Endonasal_Teleop_Param_Server/"))
  {

    std::string L1;
    std::string L1Curved;
    std::string OD1;
    std::string ID1;
    std::string E;
    std::string k1r;
    std::string PsiL1Home;
    std::string Beta1Home;

    std::string L2;
    std::string L2Curved;
    std::string OD2;
    std::string ID2;
    std::string k2r;
    std::string PsiL2Home;
    std::string Beta2Home;

    std::string L3;
    std::string L3Curved;
    std::string OD3;
    std::string ID3;
    std::string k3r;
    std::string PsiL3Home;
    std::string Beta3Home;

    // PARSE ROBOT 1
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube1/L1",L1);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube1/L1Curved",L1Curved);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube1/OD1",OD1);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube1/ID1",ID1);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube1/E",E);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube1/k1r",k1r);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube1/PsiL1Home",PsiL1Home); // TODO: set qHome from these..
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube1/Beta1Home",Beta1Home);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube2/L2",L2);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube2/L2Curved",L2Curved);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube2/OD2",OD2);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube2/ID2",ID2);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube2/k2r",k2r);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube2/PsiL2Home",PsiL2Home);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube2/Beta2Home",Beta2Home);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube3/L3",L3);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube3/L3Curvedt",L3Curved);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube3/OD3",OD3);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube3/ID3",ID3);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube3/k3r",k3r);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube3/PsiL3Home",PsiL3Home);
    ros::param::get("/Endonasal_Teleop_Param_Server/R1Tube3/Beta3Home",Beta3Home);

    std::cout << "OD3" << std::endl;

    char* stopString;
    double L1d = std::strtod(L1.c_str(),&stopString);
    double L1Curvedd = std::strtod(L1Curved.c_str(),&stopString);
    double OD1d = std::strtod(OD1.c_str(),&stopString);
    double ID1d = std::strtod(ID1.c_str(),&stopString);
    double Ed = std::strtod(E.c_str(),&stopString);
    double k1rd = std::strtod(k1r.c_str(),&stopString);
    double PsiL1Homed = std::strtod(PsiL1Home.c_str(),&stopString);
    double Beta1Homed = std::strtod(Beta1Home.c_str(),&stopString);

    double L2d = std::strtod(L2.c_str(),&stopString);
    double L2Curvedd = std::strtod(L2Curved.c_str(),&stopString);
    double OD2d = std::strtod(OD2.c_str(),&stopString);
    double ID2d = std::strtod(ID2.c_str(),&stopString);
    double k2rd = std::strtod(k2r.c_str(),&stopString);
    double PsiL2Homed = std::strtod(PsiL2Home.c_str(),&stopString);
    double Beta2Homed = std::strtod(Beta2Home.c_str(),&stopString);

    double L3d = std::strtod(L3.c_str(),&stopString);
    double L3Curvedd = std::strtod(L3Curved.c_str(),&stopString);
    double OD3d = std::strtod(OD3.c_str(),&stopString);
    double ID3d = std::strtod(ID3.c_str(),&stopString);
    double k3rd = std::strtod(k3r.c_str(),&stopString);
    double PsiL3Homed = std::strtod(PsiL3Home.c_str(),&stopString);
    double Beta3Homed = std::strtod(Beta3Home.c_str(),&stopString);

    medlab::CTR3RobotParams robot1Params;
    robot1Params.L1 = L1d;
    robot1Params.Lt1 = L1d - L1Curvedd;
    robot1Params.OD1 = OD1d;
    robot1Params.ID1 = ID1d;
    robot1Params.E = Ed;
    robot1Params.k1 = 1.0/k1rd;
    robot1Params.L2 = L2d;
    robot1Params.Lt2 = L2d - L2Curvedd;
    robot1Params.OD2 = OD2d;
    robot1Params.ID2 = ID2d;
    robot1Params.k2 = 1.0/k2rd;
    robot1Params.L3 = L3d;
    robot1Params.Lt3 = L3d - L3Curvedd;
    robot1Params.OD3 = OD3d;
    robot1Params.ID3 = ID3d;
    robot1Params.k3 = 1.0/k3rd;

    return robot1Params;
  }

}

int main(int argc, char *argv[])
{
  // ----------------- MAIN LOOP STRUCTURE -----------------

      // while (okay)
      // nextState = stepState(curState);
      // endwhile

      // --> INIT state
      //    --> at the end, segue to IDLE
      // --> IDLE State
      //    --> listen for event triggers to SIM or ACTIVE
      // --> ACTIVE State
      //    --> enable motors and listen to input devices
      //    --> listen for trigger back to IDLE
      // --> SIM State
      //    --> disable motors and listen to input devices
      //    --> listen for trigger back to IDLE
      //


  // TODO: the code below should be run in the INIT STATE
  // ---------------------------------------------------------------------------------------
  // --------------------------------------------------------------------------------------- TODO: <initAllRobots()>

  // Start this ROS node on network
  ros::init(argc, argv, "endonasal_teleop");
  ros::NodeHandle node;

  // -------------------------------------------------------------------------- TODO: <getParamsFromServer()>
  // <CTR3ROBOT 1 DEFINITION/> --------------------------------
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
//  medlab::CTR3RobotParams robot1Params = GetRobot1ParamsFromServer(); // TODO: this doesn't work..but I think it's close

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

  RoboticsMath::Vector6d robot1Home;
  robot1Home << 3.0, 0.0, 0.0, -160.9E-3, -127.2E-3, -86.4E-3;

  Eigen::Matrix4d robot1BaseFrame = Eigen::Matrix4d::Identity();
  robot1BaseFrame(0,3) = 10.0E-3;
  // </CTR3ROBOT 1 DEFINITION> ---------------------------------


//  // <CTR3ROBOT 2 DEFINITION> ----------------------------------
//  Eigen::Matrix4d robot2BaseFrame = Eigen::Matrix4d::Identity();
//  robot2BaseFrame(0,3) = -10.0E-3;
//  // </CTR3ROBOT 2 DEFINITION> ---------------------------------


//  // <CTR3ROBOT 3 DEFINITION> ----------------------------------
//  Eigen::Matrix4d robot3BaseFrame = Eigen::Matrix4d::Identity();
//  robot3BaseFrame(0,2) = -10.0E-3;
//  // </CTR3ROBOT 3 DEFINITION> ---------------------------------


  // Initialize robot with controller
  ResolvedRatesController rr1(cannula,robot1Params,robot1Home,robot1BaseFrame);
  rr1.init();

//  ResolvedRatesController rr2(cannula,robot1Params,robot1Home,robot2BaseFrame);
//  rr2.init();

//  ResolvedRatesController rr3(cannula,robot1Params,robot1Home,robot3BaseFrame);
//  rr3.init();

  //        McbRos* motorBoard1;
  //        std::string motorBoard1NodeName = ui_.lineEdit_nodeName->text().toStdString();
  //        motorBoard1->init(motorBoard1NodeName);

  // ------------------------------------------------------------------</getParamsFromServer()>
  // ---------------------------------------------------------------------------------------
  // --------------------------------------------------------------------------------------- </initAllRobots()>


  // SUBSCRIBERS
  ros::Subscriber omniButtonSub = node.subscribe("Buttonstates", 1, omniButtonCallback);
  ros::Subscriber omniPoseSub = node.subscribe("Omnipos", 1, omniCallback);

  // PUBLISHERS
  ros::Publisher needle_pub = node.advertise<endonasal_teleop::matrix8>("needle_position",10);
  ros::Publisher pubEncoderCommand1 = node.advertise<medlab_motor_control_board::McbEncoders>("MCB1/encoder_command", 1); // EC13
  ros::Publisher pubEncoderCommand2 = node.advertise<medlab_motor_control_board::McbEncoders>("MCB4/encoder_command", 1); // EC16

  // ROS RATE
  ros::Rate r(rosLoopRate);

  while (ros::ok())
  {

    std::cout << robot1Params.ID1 << std::endl;
    std::cout << robot1Params.OD1 << std::endl;

    // -------------------- FOR A GIVEN ACTIVE OMNI/RRC:-------------------------------

    // TODO: need to link an omni device to an instance of ResolvedRatesController (rr.SetInputDevice())
    // All of this will be in either SIM or ACTIVE states
    curOmni = omniPose; // Omni Update

    if (buttonState == 1)   // while clutched in
    {
      if (motorControlState == 1)  // If trying to actually drive the hardware
      {
        // [q_vec_new] = rr1.step(q_vec_curr,omni_desTwist,controlType);
        // pubEncoderCommand1.publish(enc1);...etc.
      }

      if (justClutched == true) // Set reference to this first frame
      {
        prevOmni = omniPose;
        justClutched = false;
      }

      RoboticsMath::Vector6d desTwist;
      desTwist = InputDeviceTwistUpdate();
      RoboticsMath::Vector6d newQ;
      newQ = rr1.step(desTwist);
      prevOmni = curOmni;

      // TODO: update vizualization if in ACTIVE or SIM STATES
      // TODO: write to motors if in ACTIVE STATE


    }

    // TODO: this is slow, need to implement only update if new kinematics
    // TODO: we should have one large message with all markers for all robots?

    needle_pub.publish(GenerateRobotVisualizationMarkers(rr1.GetRobot())); // TODO: need to still publish non-active robots

    // ---------------------------------------------------------------------------------

    // sleep
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
