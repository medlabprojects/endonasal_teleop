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

// Visuzalization
endonasal_teleop::matrix8 markersMsg;

// Omni
Eigen::Matrix4d omniPose;
Eigen::Matrix4d prevOmni;
Eigen::Matrix4d curOmni;
Eigen::Matrix4d robotTipFrameAtClutch; //clutch-in position of cannula
double rosLoopRate = 100.0;

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

int main(int argc, char *argv[])
{

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
        // Inputs: make_annular_tube( L, Lt, OD, ID, k_fun, E, G );
        TubeType T1 = CTR::make_annular_tube( robot1Params.L1, robot1Params.Lt1, robot1Params.OD1, robot1Params.ID1, k_fun1, robot1Params.E, robot1Params.G );
        TubeType T2 = CTR::make_annular_tube( robot1Params.L2, robot1Params.Lt2, robot1Params.OD2, robot1Params.ID2, k_fun2, robot1Params.E, robot1Params.G );
        TubeType T3 = CTR::make_annular_tube( robot1Params.L3, robot1Params.Lt3, robot1Params.OD3, robot1Params.ID3, k_fun3, robot1Params.E, robot1Params.G );

        // Assemble cannula
        auto cannula = std::make_tuple( T1, T2, T3 );

        // Initialize robot with controller
        ResolvedRatesController rr1(cannula,robot1Params);
        rr1.init();

        //
        // -------------------------------------------------------------------------------


//        std::cout << "pTip" << robot1.currKinematics.Ptip << std::endl << std::endl;
//        std::cout << "qTip" << robot1.currKinematics.Qtip << std::endl << std::endl;
//        std::cout << "Stability" << robot1.currKinematics.Stability << std::endl << std::endl;
//        std::cout << "Jtip" << robot1.currKinematics.Jtip << std::endl << std::endl;

//        McbRos* motorBoard1;
//        std::string motorBoard1NodeName = ui_.lineEdit_nodeName->text().toStdString();
//        motorBoard1->init(motorBoard1NodeName);

//	Eigen::Vector3d dhPrev;
//	dhPrev.fill(0);

//	prevOmni = omniPose;

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


          // Maybe all of this should be nested inside a check for new joint message so we don't bog down com

          ////
          //// Publish visualizations to rviz
          ////

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
            if (robot1CurBeta[1] > robot1Backbone.s[j] || robot1Params.L2+robot1CurBeta[1] < robot1Backbone.s[j])
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


