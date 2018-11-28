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

// Finite State Machine
#include "endonasalTeleopFSM.h"

// Qt headers
#include <QCoreApplication>
#include <QVector>
#include <QString>

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

// Phantom Omni
#include "PhantomOmniRos.h"

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

Eigen::Matrix4d robotTipFrame;
Eigen::Matrix4d robotTipFrameAtClutch; //clutch-in position of cannula
Eigen::Vector3d pTipCur;
Eigen::Vector4d qTipCur;
Eigen::Vector4d qBishopCur;
Eigen::Vector3d alphaCur;

Eigen::Matrix4d curOmni;
Eigen::Matrix4d prevOmni;
Eigen::Matrix4d omniPose;
double omniScaleFactor;

// TODO: omni message has been reformatted to stream Matrix4d & button msgs -> modify to read this in
geometry_msgs::Pose tempMsg;
void omniCallback(const geometry_msgs::Pose &msg) // TODO: refactor this to not use tempMsg...
{
//	tempMsg = msg;

//	Eigen::Vector4d qOmni;
//	qOmni << tempMsg.orientation.w, tempMsg.orientation.x, tempMsg.orientation.y, tempMsg.orientation.z;
//        Eigen::Matrix3d ROmni = RoboticsMath::quat2rotm(qOmni);

//	Eigen::Vector3d pOmni;
//	pOmni << tempMsg.position.x, tempMsg.position.y, tempMsg.position.z;

//	omniPose.fill(0);
//	omniPose.topLeftCorner(3, 3) = ROmni;
//	omniPose.topRightCorner(3, 1) = pOmni;
//	omniPose(3, 3) = 1.0;

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
//RoboticsMath::Vector6d PhantomOmniRos::InputDeviceTwistUpdate()
//{
//  // This uses the global curOmni and prevOmni vars to update desTwist
//  Eigen::Matrix4d omniDeltaOmniPenCoords = Mtransform::Inverse(prevOmni)*curOmni;

//  Eigen::Vector3d zeroVec;
//  zeroVec.fill(0);

//  omniDeltaOmniPenCoords.block(0,3,3,1) = omniDeltaOmniPenCoords/1.0E3;
//  omniDeltaOmniPenCoords.block(0,3,3,1) = omniScaleFactor*omniDeltaOmniPenCoords.block(0,3,3,1);
//  Eigen::Matrix4d RPrevOmni = RoboticsMath::assembleTransformation(prevOmni.block(0,0,3,3),zeroVec);
//  Eigen::Matrix4d omniDeltaOmniBaseCoords = (Mtransform::Inverse(RPrevOmni.transpose())*omniDeltaOmniPenCoords*RPrevOmni.transpose());

//  Eigen::Matrix3d Rd = curOmni.block<3,3>(0,0);
//  Eigen::Matrix3d Rc = prevOmni.block<3,3>(0,0);
//  Eigen::Matrix3d Re = Rd*Rc.transpose();
//  Re = RoboticsMath::orthonormalize(Re);

//  // desTwist should use OmniBaseCoords for linar velocity, and OmniPenCoords for angular velocity
//  RoboticsMath::Vector6d omniTwistOmniPenCoords;
//  omniTwistOmniPenCoords(0) = omniDeltaOmniPenCoords(0,3); // vx
//  omniTwistOmniPenCoords(1) = omniDeltaOmniPenCoords(1,3); // vy
//  omniTwistOmniPenCoords(2) = omniDeltaOmniPenCoords(2,3); // vz
//  omniTwistOmniPenCoords(3) = omniDeltaOmniPenCoords(2,1); // wx
//  omniTwistOmniPenCoords(4) = omniDeltaOmniPenCoords(0,2); // wy
//  omniTwistOmniPenCoords(5) = omniDeltaOmniPenCoords(1,0); // wz

//  RoboticsMath::Vector6d omniTwistOmniBaseCoords;
//  omniTwistOmniBaseCoords(0) = omniDeltaOmniBaseCoords(0,3);
//  omniTwistOmniBaseCoords(1) = omniDeltaOmniBaseCoords(1,3);
//  omniTwistOmniBaseCoords(2) = omniDeltaOmniBaseCoords(2,3);
//  omniTwistOmniBaseCoords(3) = omniDeltaOmniBaseCoords(2,1);
//  omniTwistOmniBaseCoords(4) = omniDeltaOmniBaseCoords(0,2);
//  omniTwistOmniBaseCoords(5) = omniDeltaOmniBaseCoords(1,0);

//  RoboticsMath::Vector6d desTwist;
//  desTwist.head(3) = omniTwistOmniBaseCoords.head(3);
//  desTwist.tail(3) = omniTwistOmniPenCoords.tail(3);

//  return desTwist;
//}

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

medlab::CTR3RobotParams GetRobot1ParamsFromServer(QString robotNamespace)
{

    double L1;
    double L1Curved;
    double OD1;
    double ID1;
    double E;
    double k1r;
    double PsiL1Home;
    double Beta1Home;

    double L2;
    double L2Curved;
    double OD2;
    double ID2;
    double k2r;
    double PsiL2Home;
    double Beta2Home;

    double L3;
    double L3Curved;
    double OD3;
    double ID3;
    double k3r;
    double PsiL3Home;
    double Beta3Home;

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

    std::cout << "OD3" << OD3 << std::endl;

    // TODO: set robot1Params
    medlab::CTR3RobotParams robot1Params;


    return robot1Params;
//  }

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
//  medlab::CTR3RobotParams robot1Params;
//  robot1Params.E = 60E9;
//  robot1Params.G = 60E9 / 2.0 / 1.33;
//  robot1Params.L1 = 222.5E-3;
//  robot1Params.Lt1 = robot1Params.L1 - 42.2E-3;
//  robot1Params.k1 = 1.0/63.5E-3;
//  robot1Params.OD1 = 1.165E-3;
//  robot1Params.ID1 = 1.067E-3;
//  robot1Params.L2 = 163E-3;
//  robot1Params.Lt2 = robot1Params.L2 - 38.0E-3;
//  robot1Params.k2 = 1.0/51.2E-3;
//  robot1Params.OD2 = 2.0574E-3;
//  robot1Params.ID2 = 1.6002E-3;
//  robot1Params.L3 = 104.4E-3;
//  robot1Params.Lt3 = robot1Params.L3 - 21.3E-3;
//  robot1Params.k3 = 1.0/71.4E-3;
//  robot1Params.OD3 = 2.540E-3;
//  robot1Params.ID3 = 2.248E-3;
  medlab::CTR3RobotParams robot1Params = GetRobot1ParamsFromServer(QString("Robot1NameSpace")); // TODO: this doesn't work..but I think it's close

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

//    std::cout << robot1Params.ID1 << std::endl;
//    std::cout << robot1Params.OD1 << std::endl;

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
