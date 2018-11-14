#pragma once
#include <Eigen/Dense>
#include <vector>

#include "Kinematics.h"
#include "BasicFunctions.h"
#include "Tube.h"
#include "Utility.h"
#include "spline.h"
#include "RoboticsMath.h"
#include "MedlabTypes.h"

class CTR3Robot
{

public:
	CTR3Robot();
        //CTR3Robot(const medlab::CTR3RobotParams params);
	~CTR3Robot();
	void init();

	bool SetCannula(const medlab::CTR3RobotParams params); // setting a pointer is better practice? shared pointers?
	medlab::Cannula3 GetCannula(); // should this return the current cannula tuple or just params? 

	bool SetCurrStateVector(medlab::CTR3ModelStateVector stateVector);
	medlab::CTR3ModelStateVector GetCurrStateVector();

	bool SetCurrQVec(RoboticsMath::Vector6d qVec);
	RoboticsMath::Vector6d GetCurrQVec();

	bool SetInterpolatedBackbone(medlab::InterpRet interpolatedBackbone);
	medlab::InterpRet GetInterpolatedBackbone();

        medlab::KinOut callKinematicsWithDenseOutput(RoboticsMath::Vector6d newStateVector);
	Eigen::MatrixXd forwardKinematics(auto kin);

	medlab::InterpRet interpolateBackbone(Eigen::VectorXd sRef, Eigen::MatrixXd poseDataRef, int nPts);

private:

        //std::string cannulaName_;
	medlab::Cannula3 cannula_;  // cannula object fed to kinematics call
	medlab::CTR3RobotParams currCannulaParams_; // params that define the cannula3
	medlab::CTR3ModelStateVector currStateVector_; // state vector fed to kinematics call //TODO: store full q_vector and function to condense
	RoboticsMath::Vector6d currQVec_; // condensed state vector [psiL, beta]
	RoboticsMath::Vector6d qHome_; // home configuration (joint space [psiL, beta])
	medlab::InterpRet currInterpolatedBackbone_; // interpolated cannula
	int lastPosInterp_;
	medlab::KinOut currKinematics_;  // kinout from kinematics call, transformed into base frame & interpolated

};
