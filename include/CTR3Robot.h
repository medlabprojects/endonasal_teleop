#pragma once
//#include <Eigen/Dense>
//#include <vector>

//#include "Kinematics.h"
#include "BasicFunctions.h"
#include "Tube.h"
#include "Utility.h"
#include "spline.h"
#include "RoboticsMath.h"
#include "MedlabTypes.h"

class CTR3Robot
{

public:
  CTR3Robot(medlab::Cannula3 cannula);
  bool init();
  bool init(RoboticsMath::Vector6d qHome);

  medlab::Cannula3 GetCannula();

  bool SetCurrKinematicsInput(medlab::CTR3KinematicsInputVector kinematicsInput);
  medlab::CTR3KinematicsInputVector GetCurrKinematicsInput();

  bool SetCurrQVec(RoboticsMath::Vector6d qVec);
  RoboticsMath::Vector6d GetCurrQVec();

  bool SetInterpolatedBackbone(medlab::InterpRet interpolatedBackbone);
  medlab::InterpRet GetInterpolatedBackbone();

  medlab::KinOut callKinematicsWithDenseOutput(medlab::CTR3KinematicsInputVector newKinematicsInput);
  Eigen::MatrixXd forwardKinematics(auto kin);

  medlab::InterpRet interpolateBackbone(Eigen::VectorXd sRef, Eigen::MatrixXd poseDataRef, int nPts);

  medlab::KinOut currKinematics;  // kinout from kinematics call, transformed into base frame & interpolated

private:
  medlab::Cannula3 cannula_;  // cannula object fed to kinematics call
  medlab::CTR3RobotParams currCannulaParams_; // params that define the cannula3
  medlab::CTR3KinematicsInputVector currKinematicsInput_; // state vector fed to kinematics call //TODO: store full q_vector and function to condense
  RoboticsMath::Vector6d currQVec_; // condensed state vector [psiL, beta]
  RoboticsMath::Vector6d qHome_; // home configuration (joint space [psiL, beta])
  medlab::InterpRet currInterpolatedBackbone_; // interpolated cannula
};
