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
#include "Kinematics.h"
#include "BasicFunctions.h"
#include "Tube.h"

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

/*
 / NAMESPACES
using namespace rapidxml;
using namespace Eigen;
using namespace CTR;
using namespace CTR::Functions;
using std::tuple;
using namespace std;

// TYPEDEFS
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef tuple < Tube< constant_fun< Vector2d > >,
				Tube< constant_fun< Vector2d > >,
				Tube< constant_fun< Vector2d > > > Cannula3;
typedef constant_fun<Eigen::Vector2d> CurvFun;
typedef std::tuple< Tube<CurvFun>, Tube<CurvFun>, Tube<CurvFun> > CannulaT;
typedef DeclareOptions< Option::ComputeJacobian, Option::ComputeGeometry, Option::ComputeStability, Option::ComputeCompliance>::options OType;

struct Configuration3
{
	Eigen::Vector3d	PsiL;
	Eigen::Vector3d	Beta;
	Eigen::Vector3d Ftip;
	Eigen::Vector3d	Ttip;
};

// GLOBAL VARIABLES NEEDED FOR RESOLVED RATES
geometry_msgs::Vector3 omniForce;

Eigen::Vector3d ptipcur; // use for continually updated message value
Eigen::Vector4d qtipcur;
Eigen::Vector3d alphacur;
Matrix6d Jcur;
bool new_kin_msg = 0;
*/

RoboticsMath::Matrix4d omniPose;
RoboticsMath::Matrix4d prevOmni;
RoboticsMath::Matrix4d curOmni;
RoboticsMath::Matrix4d robotTipFrameAtClutch; //clutch-in position of cannula
double rosLoopRate = 100.0;


/*
// SERVICE CALL FUNCTION DEFINITION ------------------------------

bool startingConfig(endonasal_teleop::getStartingConfig::Request &req, endonasal_teleop::getStartingConfig::Response &res)
{
	std::cout << "Retrieving the starting configuration..." << std::endl << std::endl;

	Configuration3 qstart;
	qstart.PsiL = Eigen::Vector3d::Zero();
	qstart.Beta << -160.9e-3, -127.2e-3, -86.4e-3;
	qstart.Ftip = Eigen::Vector3d::Zero();
	qstart.Ttip = Eigen::Vector3d::Zero();

	for (int i = 0; i<3; i++)
	{
		res.joint_q[i] = qstart.PsiL[i];
		res.joint_q[i + 3] = qstart.Beta[i];
		res.joint_q[i + 6] = qstart.Ftip[i];
		res.joint_q[i + 9] = qstart.Ttip[i];
	}

	return true;
}


// MESSAGE CALLBACK FUNCTION DEFINITIONS ---------------------------

endonasal_teleop::kinout tmpkin;
void kinCallback(const endonasal_teleop::kinout kinmsg)
{
	tmpkin = kinmsg;

	// pull out position
	ptipcur[0] = tmpkin.p[0];
	ptipcur[1] = tmpkin.p[1];
	ptipcur[2] = tmpkin.p[2];

	// pull out orientation (quaternion)
	qtipcur[0] = tmpkin.q[0];
	qtipcur[1] = tmpkin.q[1];
	qtipcur[2] = tmpkin.q[2];
	qtipcur[3] = tmpkin.q[3];

	// pull out the base angles of the tubes (alpha in rad)	
	alphacur[0] = tmpkin.alpha[0];
	alphacur[1] = tmpkin.alpha[1];
	alphacur[2] = tmpkin.alpha[2];

	// pull out Jacobian
	for (int i = 0; i<6; i++)
	{
		Jcur(0, i) = tmpkin.J1[i];
		Jcur(1, i) = tmpkin.J2[i];
		Jcur(2, i) = tmpkin.J3[i];
		Jcur(3, i) = tmpkin.J4[i];
		Jcur(4, i) = tmpkin.J5[i];
		Jcur(5, i) = tmpkin.J6[i];
	}
}

std_msgs::Bool tmpFKM;
void kinStatusCallback(const std_msgs::Bool &fkmMsg)
{
	tmpFKM = fkmMsg;
	if (tmpFKM.data == true)
	{
		new_kin_msg = 1;
	}
}*/

geometry_msgs::Pose tempMsg;
void omniCallback(const geometry_msgs::Pose &msg)
{
	tempMsg = msg;
	//    prevOmni = curOmni;

	Eigen::Vector4d qOmni;
	qOmni << tempMsg.orientation.w, tempMsg.orientation.x, tempMsg.orientation.y, tempMsg.orientation.z;
        Eigen::Matrix3d ROmni = RoboticsMath::quat2rotm(qOmni);

	Eigen::Vector3d pOmni;
	pOmni << tempMsg.position.x, tempMsg.position.y, tempMsg.position.z;

	omniPose.fill(0);
	omniPose.topLeftCorner(3, 3) = ROmni;
	omniPose.topRightCorner(3, 1) = pOmni;
	omniPose(3, 3) = 1.0;

	//    curOmni = omniPose;
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
//void zero_force()
//{
//	omniForce.x = 0.0;
//	omniForce.y = 0.0;
//	omniForce.z = 0.0;
//}

int main(int argc, char *argv[])
{
	/*******************************************************************************
	INITIALIZE ROS NODE
	********************************************************************************/
	ros::init(argc, argv, "resolved_rates");
	ros::NodeHandle node;
	/*******************************************************************************
	DECLARATIONS & CONSTANT DEFINITIONS
	********************************************************************************/
	// TELEOP PARAMETERS
//	double scale_factor = 0.10;
//	double lambda_tracking = 10.0; 	// originally 1.0			// TODO: tune these gains
//	double lambda_damping = 50.0;    // originally 5.0
//	double lambda_jointlim = 100.0;  // originally 10.0

//									 // motion tracking weighting matrix (task space):
//	Matrix6d W_tracking = Eigen::Matrix<double, 6, 6>::Zero();
//	W_tracking(0, 0) = lambda_tracking*1.0e6;
//	W_tracking(1, 1) = lambda_tracking*1.0e6;
//	W_tracking(2, 2) = lambda_tracking*1.0e6;
//	W_tracking(3, 3) = 0.1*lambda_tracking*(180.0 / M_PI / 2.0)*(180.0 / M_PI / 2.0);
//	W_tracking(4, 4) = 0.1*lambda_tracking*(180.0 / M_PI / 2.0)*(180.0 / M_PI / 2.0);
//	W_tracking(5, 5) = lambda_tracking*(180.0 / M_PI / 2.0)*(180.0 / M_PI / 2.0);

//	std::cout << "W_tracking = " << std::endl << W_tracking << std::endl << std::endl;

//	// damping weighting matrix (actuator space):
//	Matrix6d W_damping = Eigen::Matrix<double, 6, 6>::Zero();
//	double thetadeg = 2.0; // degrees to damp as much as 1 mm
//	W_damping(0, 0) = lambda_damping*(180.0 / thetadeg / M_PI)*(180.0 / thetadeg / M_PI);
//	W_damping(1, 1) = lambda_damping*(180.0 / thetadeg / M_PI)*(180.0 / thetadeg / M_PI);
//	W_damping(2, 2) = lambda_damping*(180.0 / thetadeg / M_PI)*(180.0 / thetadeg / M_PI);
//	W_damping(3, 3) = lambda_damping*1.0e6;
//	W_damping(4, 4) = lambda_damping*1.0e6;
//	W_damping(5, 5) = lambda_damping*1.0e6;

//	std::cout << "W_damping = " << std::endl << W_damping << std::endl << std::endl;

	// conversion from beta to x:
//	Eigen::Matrix3d dbeta_dx;
//	dbeta_dx << 1, 1, 1,
//		0, 1, 1,
//		0, 0, 1;

//	Eigen::Matrix<double, 6, 6> dqbeta_dqx;
//	dqbeta_dqx.fill(0);
//	dqbeta_dqx.block(0, 0, 3, 3) = MatrixXd::Identity(3, 3);
//	dqbeta_dqx.block(3, 3, 3, 3) = dbeta_dx;

//	// ROBOT POSE VARIABLES
//	Eigen::Vector3d ptip;
//	Eigen::Vector4d qtip;
//	Eigen::Matrix3d Rtip;
//	Eigen::Vector3d alpha;
//	Vector6d q_vec;
//	Vector6d delta_qx;
//	Matrix4d omniFrameAtClutch;
//	Matrix4d robotTipFrameAtClutch; //clutch-in position of cannula
//	Matrix4d robotTipFrame; //from tip pose
//	Matrix6d J;
//	Matrix4d robotDesFrameDelta;
//	Vector6d robotDesTwist;
//	Eigen::Vector3d L;

//	// OMNI POSE VARIABLES
//	Matrix4d Tregs;
//	Matrix4d omniDelta_omniCoords;
//	Matrix4d prevOmniInv;
//	Matrix4d omniDelta_cannulaCoords; //change in Omni position between timesteps in inertial frame

//									  // OMNI REGISTRATION (constant)
//	Matrix4d OmniReg = Matrix4d::Identity();
//	Eigen::MatrixXd rotationY = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
//	Mtransform::SetRotation(OmniReg, rotationY);

//	Matrix4d OmniRegInv = Mtransform::Inverse(OmniReg);

//	// MESSAGES TO BE SENT
//	endonasal_teleop::config3 q_msg;
//	std_msgs::Bool rrUpdateStatusMsg;

//	// MISC.
//	Eigen::Vector3d zerovec;
//	zerovec.fill(0);
//	double theta;
//	double acosArg;
//	double trace;
//	Eigen::Matrix3d logR;
//	double logRmag;
//	Vector6d delta_q;
//	Eigen::Matrix<double, 6, 6> A;
//	Eigen::Matrix<double, 6, 6> Jstar;

	/*******************************************************************************
	SET UP PUBLISHERS, SUBSCRIBERS, SERVICES & CLIENTS
	********************************************************************************/

	// subscribers
	ros::Subscriber omniButtonSub = node.subscribe("Buttonstates", 1, omniButtonCallback);
	ros::Subscriber omniPoseSub = node.subscribe("Omnipos", 1, omniCallback);
        //ros::Subscriber kinSub = node.subscribe("kinematics_output", 1, kinCallback);
        //ros::Subscriber kinematics_status_pub = node.subscribe("kinematics_status", 1, kinStatusCallback);

	// publishers
        //ros::Publisher rr_status_pub = node.advertise<std_msgs::Bool>("rr_status", 1000);
        //ros::Publisher jointValPub = node.advertise<endonasal_teleop::config3>("joint_q", 1000);
        //ros::Publisher omniForcePub = node.advertise<geometry_msgs::Vector3>("Omniforce", 1000);
	ros::Publisher pubEncoderCommand1 = node.advertise<medlab_motor_control_board::McbEncoders>("MCB1/encoder_command", 1); // EC13
	ros::Publisher pubEncoderCommand2 = node.advertise<medlab_motor_control_board::McbEncoders>("MCB4/encoder_command", 1); // EC16

																															//clients
        //ros::ServiceClient startingKinClient = node.serviceClient<endonasal_teleop::getStartingKin>("get_starting_kin");

	// rate
	ros::Rate r(rosLoopRate);

	/*******************************************************************************
	LOAD PARAMETERS FROM XML FILES
	********************************************************************************/

//	if (ros::param::has("/CannulaExample1/EndEffectorType"))
//	{
//		std::cout << "I found that parameter you wanted. let's try to do something useful with it" << std::endl;
//	}

//	std::string EEtype;
//	ros::param::get("/CannulaExample1/EndEffectorType", EEtype);

//	std::cout << "EEtype is now: " << EEtype << std::endl;

	//    // based on an example located at: https://gist.github.com/JSchaenzle/2726944

	//    std::cout<<"Parsing xml file..."<<std::endl;
	//    xml_document<> doc;
	//    std::ifstream cannulaFile ("/home/remireaa/catkin_ws/src/endonasal_teleop/config/cannula_example1.xml"); // input file stream (in std library)

	//    // Read xml file into a vector
	//    std::vector<char> buffer( (std::istreambuf_iterator<char>(cannulaFile)), std::istreambuf_iterator<char>() );
	//    if (buffer.size() > 0)
	//    {
	//        std::cout << "Successfully loaded xml file." << std::endl;
	//    }
	//    buffer.push_back('\0');

	//    // Parse buffer into doc using the xml file parsing library
	//    doc.parse<0>(&buffer[0]);

	//    // Process information in cannula xml file
	//    xml_node<> *cannula_node = doc.first_node("Cannula");
	//    std::cout << "This is an active cannula robot with " << cannula_node->first_attribute("NumberOfTubes")->value() << " tubes." << std::endl;

	//    // Iterate over tubes within cannula node:
	//    std::cout << "right before loop..." << std::endl;
	//    for(xml_node<> *tube_node = cannula_node->first_node("Tube"); tube_node; tube_node = tube_node->next_sibling())
	//    {
	//        std::cout << "here i am" << std::endl;
	//    }

	/*******************************************************************************
	COMPUTE KINEMATICS FOR STARTING CONFIGURATION
	********************************************************************************/

//	Configuration3 qstart;
//	qstart.PsiL = Eigen::Vector3d::Zero();
//	qstart.Beta << -160.9e-3, -127.2e-3, -86.4e-3;		// TODO: need to set these and get corresponding counts for initialization
//	qstart.Ftip = Eigen::Vector3d::Zero();
//	qstart.Ttip = Eigen::Vector3d::Zero();

//	q_vec << qstart.PsiL(0), qstart.PsiL(1), qstart.PsiL(2), qstart.Beta(0), qstart.Beta(1), qstart.Beta(2);

//	L << 222.5e-3, 163e-3, 104.4e-3;

//	// Call getStartingKin service:
//	endonasal_teleop::getStartingKin get_starting_kin;
//	get_starting_kin.request.kinrequest = true;
//	ros::service::waitForService("get_starting_kin", -1);
//	zero_force();

//	if (startingKinClient.call(get_starting_kin))
//	{
//		for (int i = 0; i<3; i++)
//		{
//			ptipcur(i) = get_starting_kin.response.p[i];
//		}

//		for (int i = 0; i<4; i++)
//		{
//			qtipcur(i) = get_starting_kin.response.q[i];
//		}

//		for (int i = 0; i<6; i++)
//		{
//			Jcur(0, i) = get_starting_kin.response.J1[i];
//			Jcur(1, i) = get_starting_kin.response.J2[i];
//			Jcur(2, i) = get_starting_kin.response.J3[i];
//			Jcur(3, i) = get_starting_kin.response.J4[i];
//			Jcur(4, i) = get_starting_kin.response.J5[i];
//			Jcur(5, i) = get_starting_kin.response.J6[i];
//		}

//		new_kin_msg = 1;

//		std::cout << "Starting pose and Jacobian received." << std::endl << std::endl;
//	}
//	else
//	{
//		std::cout << "Failed to fetch starting pose and Jacobian." << std::endl << std::endl;
//		return 1;
//	}

//	// Check that the kinematics got called once
//	std::cout << "ptip at start = " << std::endl << ptipcur << std::endl << std::endl;
//	std::cout << "qtip at start = " << std::endl << qtipcur << std::endl << std::endl;
//	std::cout << "J at start = " << std::endl << Jcur << std::endl << std::endl;

//	Eigen::Vector3d dhPrev;
//	dhPrev.fill(0);

//	prevOmni = omniPose;

	while (ros::ok())
	{
//		if (new_kin_msg == 1)
//		{
//			//new_kin_msg = 0;
//			// take a "snapshot" of the current values from the kinematics and Omni for this loop iteration
//			curOmni = omniPose;
//			ptip = ptipcur;
//			qtip = qtipcur;
//			alpha = alphacur;
//			J = Jcur;
//			Rtip = quat2rotm(qtip);
//			robotTipFrame = assembleTransformation(Rtip, ptip);

//			// send commands to motorboards
//			double offset_trans_inner = 0; // -46800.0;
//			double offset_trans_middle = 0; // -36080.0;
//			double offset_trans_outer = 0; // -290.0;
//			double scale_rot = 16498.78; 		// counts/rad
//			double scale_trans = 6802.16*1e3;		// counts/m
//			double scale_trans_outer = 2351.17*1e3;	// counts/m

//			medlab_motor_control_board::McbEncoders enc1;
//			medlab_motor_control_board::McbEncoders enc2;

//			//		enc1.count[0] = (int)(q_msg.joint_q[3] * scale_trans - offset_trans_inner); // inner translation
//			enc1.count[0] = (int)((q_vec[3] - qstart.Beta[0]) * scale_trans); // inner translation
//			enc1.count[1] = (int)(alpha[0] * scale_rot); // inner rotation
//			enc1.count[2] = (int)(alpha[2] * scale_rot); // outer rotation
//														 //		enc1.count[3] = (int)(q_msg.joint_q[4] * scale_trans - offset_trans_middle); // middle translation
//			enc1.count[3] = (int)((q_vec[4] - qstart.Beta[1]) * scale_trans);  // middle translation
//			enc1.count[4] = (int)(alpha[1] * scale_rot); // middle rotation
//			enc1.count[5] = 0; // accessory

//							   //		enc2.count[0] = (int)(q_msg.joint_q[5] * scale_trans_outer - offset_trans_outer); // external translation
//			enc2.count[0] = 0; // (int)((q_vec[5] - qstart.Beta[2]) * scale_trans_outer); 	// outer translation
//			enc2.count[1] = 0;
//			enc2.count[2] = (int)((q_vec[5] - qstart.Beta[2]) * scale_trans_outer); 	// outer translation;
//			enc2.count[3] = 0;
//			enc2.count[4] = 0;
//			enc2.count[5] = 0;

//			pubEncoderCommand1.publish(enc1);
//			pubEncoderCommand2.publish(enc2);

//			if (buttonState == 1) //must clutch in button for any motions to happen
//			{
//				//std::cout << "J(beta) = " << std::endl << J << std::endl << std::endl;
//				std::cout << "ptip = " << std::endl << ptip.transpose() << std::endl << std::endl;
//				//std::cout << "qtip = " << std::endl << qtip.transpose() << std::endl << std::endl;

//				Matrix4d ROmniFrameAtClutch; // Rotation of the Omni frame tip at clutch in

//											 //furthermore, if this is the first time step of clutch in, we need to save the robot pose & the omni pose
//				if (justClutched == true)
//				{
//					robotTipFrameAtClutch = robotTipFrame;
//					omniFrameAtClutch = omniPose;
//					ROmniFrameAtClutch = assembleTransformation(omniFrameAtClutch.block(0, 0, 3, 3), zerovec);
//					Tregs = assembleTransformation(Rtip.transpose(), zerovec);
//					std::cout << "robotTipFrameAtClutch = " << std::endl << robotTipFrameAtClutch << std::endl << std::endl;
//					std::cout << "omniFrameAtClutch = " << std::endl << omniFrameAtClutch << std::endl << std::endl;
//					justClutched = false; // next time, skip this step
//				}

//				// find change in omni position and orientation from the clutch pose
//				omniDelta_omniCoords = Mtransform::Inverse(ROmniFrameAtClutch.transpose())*Mtransform::Inverse(omniFrameAtClutch)*curOmni*ROmniFrameAtClutch.transpose();
//				//omniDelta_omniCoords = Mtransform::Inverse(omniFrameAtClutch)*curOmni;
//				// std::cout << "omniDelta_omniCoords = " << std::endl << omniDelta_omniCoords << std::endl << std::endl;

//				// expressed in cannula base frame coordinates
//				omniDelta_cannulaCoords = OmniRegInv*omniDelta_omniCoords*OmniReg;

//				// convert position units mm -> m
//				omniDelta_cannulaCoords.block(0, 3, 3, 1) = omniDelta_cannulaCoords.block(0, 3, 3, 1) / 1000.0;

//				// scale position through scaling ratio
//				omniDelta_cannulaCoords.block(0, 3, 3, 1) = scale_factor*omniDelta_cannulaCoords.block(0, 3, 3, 1);

//				// scale orientation through scaling ratio (if it is large enough)
//				trace = omniDelta_cannulaCoords(0, 0) + omniDelta_cannulaCoords(1, 1) + omniDelta_cannulaCoords(2, 2);
//				acosArg = 0.5*(trace - 1);
//				if (acosArg>1.0) { acosArg = 1.0; }
//				if (acosArg<-1.0) { acosArg = -1.0; }
//				theta = acos(acosArg);
//				if (fabs(theta)>1.0e-3)
//				{
//					Eigen::Matrix3d Rdelta;
//					Rdelta = omniDelta_cannulaCoords.block(0, 0, 3, 3);
//					logR = theta / (2 * sin(theta))*(Rdelta - Rdelta.transpose());
//					logRmag = logR(2, 1)*logR(2, 1) + logR(1, 0)*logR(1, 0) + logR(0, 2)*logR(0, 2);
//					logRmag *= 0.8;
//					logR *= 0.8;
//					if (logRmag > 1.0e-3)
//					{
//						Rdelta = Eigen::MatrixXd::Identity(3, 3) + sin(logRmag) / logRmag*logR + (1 - cos(logRmag)) / (logRmag*logRmag)*logR*logR;
//						Mtransform::SetRotation(omniDelta_cannulaCoords, Rdelta);
//					}
//				}

//				// std::cout << "omniDelta_cannulaCoords = " << std::endl << omniDelta_cannulaCoords << std::endl << std::endl;
//				//std::cout << "robotTipFrame = " << std::endl << robotTipFrame << std::endl << std::endl;

//				// compute the desired robot motion from the omni motion
//				robotDesFrameDelta = Mtransform::Inverse(robotTipFrame) * robotTipFrameAtClutch * Tregs * omniDelta_cannulaCoords * Mtransform::Inverse(Tregs);

//				// convert to twist coordinates ("wedge" operator)
//				///*
//				robotDesTwist[0] = robotDesFrameDelta(0, 3); //v_x
//				robotDesTwist[1] = robotDesFrameDelta(1, 3); //v_y
//				robotDesTwist[2] = robotDesFrameDelta(2, 3); //v_z
//				robotDesTwist[3] = robotDesFrameDelta(2, 1); //w_x
//				robotDesTwist[4] = robotDesFrameDelta(0, 2); //w_y
//				robotDesTwist[5] = robotDesFrameDelta(1, 0); //w_z
//															 //*/

//															 /*
//															 robotDesTwist[0]=0.0; //v_x
//															 robotDesTwist[1]=0.0; //v_y
//															 robotDesTwist[2]=0.0; //v_z
//															 robotDesTwist[3]=0.0; //w_x
//															 robotDesTwist[4]=0.0; //w_y
//															 robotDesTwist[5]=0.1; //w_z
//															 */

//				std::cout << "robotDesTwist = " << std::endl << robotDesTwist.transpose() << std::endl << std::endl;
//				robotDesTwist = scaleOmniVelocity(robotDesTwist);
//				std::cout << "scaled robotDesTwist = " << std::endl << robotDesTwist.transpose() << std::endl << std::endl;

//				// position control only:
//				//                Eigen::Vector3d delta_x = robotDesTwist.head(3);
//				//                Eigen::Matrix<double,3,6> Jpos = J.topRows(3);
//				//                A = (Jpos.transpose()*Jpos + 1e-4*Eigen::MatrixXd::Identity(6,6));
//				//                Jstar = A.inverse()*Jpos.transpose();

//				//                // position and orientation control:
//				//                A = J.transpose()*J + 1e-6*Eigen::MatrixXd::Identity(6,6);
//				//                Jstar = A.inverse()*J.transpose();

//				//                //main resolved rates update

//				//                delta_q = J.colPivHouseholderQr().solve(robotDesTwist);
//				//                delta_q = Jstar*delta_x;
//				//                delta_q = Jstar*robotDesTwist;
//				//                q_vec = q_vec + delta_q;

//				// same as what's on the bimanual
//				/*Eigen::Matrix<double,6,6> K = J.transpose()*W_tracking*J + W_damping;
//				Vector6d V = J.transpose()*W_tracking*robotDesTwist;
//				delta_q = K.partialPivLu().solve(V);
//				delta_q = saturateJointVelocities(delta_q,rosLoopRate);
//				q_vec = q_vec + delta_q;
//				*/
//				//std::cout << "delta_q = "  << std::endl << delta_q.transpose() << std::endl << std::endl;

//				// Check if valid betas
//				//Vector6d qx_vec = transformBetaToX(q_vec,L);
//				//qx_vec.tail(3) = limitBetaValsSimple(qx_vec.tail(3),L);
//				//q_vec = transformXToBeta(qx_vec,L);

//				//                // Transformation from qbeta to qx:
//				Eigen::Matrix<double, 6, 6> Jx = J*dqbeta_dqx;
//				//std::cout << "Jx = " << std::endl << Jx << std::endl << std::endl;
//				Vector6d qx_vec = transformBetaToX(q_vec, L);

//				// Joint limit avoidance weighting matrix
//				weightingRet Wout = getWeightingMatrix(qx_vec.tail(3), dhPrev, L, lambda_jointlim);
//				Eigen::Matrix<double, 6, 6> W_jointlim = Wout.W;
//				dhPrev = Wout.dh; // and save this dh value for next time

//								  // Resolved rates update
//				Eigen::Matrix<double, 6, 6> A = Jx.transpose()*W_tracking*Jx + W_damping + W_jointlim;
//				Vector6d b = Jx.transpose()*W_tracking*robotDesTwist;
//				//Eigen::Matrix<double,6,6> A = Jx.transpose()*W_tracking*Jx;
//				//Vector6d b = Jx.transpose()*W_tracking*robotDesTwist;
//				delta_qx = A.partialPivLu().solve(b);
//				//std::cout <<"delta_qx: "<< delta_qx.transpose() << std::endl << std::endl;

//				//delta_qx = saturateJointVelocities(delta_qx, rosLoopRate);
//				qx_vec = qx_vec + delta_qx;

//				// Correct joint limit violations
//				qx_vec.tail(3) = limitBetaValsSimple(qx_vec.tail(3), L);

//				//                //            Eigen::Matrix<double,6,6> K1 = Jx.transpose()*W_tracking*Jx + W_damping;
//				//                //            Vector6d V = Jx.transpose()*W_tracking*robotDesTwist;
//				//                //            Vector6d delta_qx = K1.partialPivLu().solve(V);
//				//                qx_vec = qx_vec + delta_qx;

//				//                std::cout << "delta_qx = " << delta_qx << std::endl << std::endl;
//				//                std::cout << "qx_vec = " << qx_vec << std::endl << std::endl;

//				// Transform qbeta back from qx
//				q_vec = transformXToBeta(qx_vec, L);
//				//std::cout << "q_vec = " << std::endl << q_vec.transpose() << std::endl;
//				std::cout << "----------------------------------------------------" << std::endl << std::endl;

//				prevOmni = curOmni;

//				for (int h = 0; h<6; h++)
//				{
//					q_msg.joint_q[h] = q_vec(h);
//					q_msg.joint_q[h + 6] = 0;
//				}
//				rrUpdateStatusMsg.data = true;
//			}

//			else    // if the button isn't clutched, just send out the current joint values to kinematics
//			{
//				for (int h = 0; h<6; h++)
//				{
//					q_msg.joint_q[h] = q_vec(h);
//					q_msg.joint_q[h + 6] = 0;
//				}
//				rrUpdateStatusMsg.data = true;
//			}

//			// publish
//			jointValPub.publish(q_msg);
//			rr_status_pub.publish(rrUpdateStatusMsg);
//			omniForcePub.publish(omniForce);


//		}

		// sleep
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
