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
  CTR3Robot(medlab::Cannula3 cannula, medlab::CTR3RobotParams params,
            RoboticsMath::Vector6d qHome, Eigen::Matrix4d baseFrame);
  bool init();

  medlab::Cannula3 GetCannula();
  medlab::CTR3RobotParams GetCurRobotParams();
  medlab::CTR3KinematicsInputVector GetCurrKinematicsInputVector();
  RoboticsMath::Vector6d GetCurrQVec();
  RoboticsMath::Vector6d GetQHome();
  medlab::InterpRet GetInterpolatedBackbone();
  int GetNPts();
  int GetNInterp();

  medlab::KinOut callKinematicsWithDenseOutput(medlab::CTR3KinematicsInputVector newKinematicsInput);
  Eigen::MatrixXd forwardKinematics(auto kin);
  medlab::InterpRet interpolateBackbone(Eigen::VectorXd sRef, Eigen::MatrixXd poseDataRef, int nPts);
  medlab::WeightingRet computeStabilityWeightingMatrix(RoboticsMath::Vector6d qVec, double S, double sThreshold, double alphaS);
  medlab::KinOut currKinematics;  // kinout from kinematics call, transformed into base frame & interpolated
  RoboticsMath::Matrix6d WStability;
  Eigen::VectorXd vS;
  Eigen::Matrix4d BaseFrame_WORLD;

private:
  medlab::Cannula3 cannula_;  // cannula tuple object fed to Hunter's kinematics
  medlab::CTR3RobotParams currCannulaParams_; // params that define the cannula3
  medlab::CTR3KinematicsInputVector currKinematicsInputVector_; // full input vector fed to kinematics call
  RoboticsMath::Vector6d currQVec_; // condensed kinematics input vector [psiL, beta]
  RoboticsMath::Vector6d qHome_; // home configuration (joint space [psiL, beta])
  medlab::InterpRet currInterpolatedBackbone_; // interpolated cannula [sxn,pxn,qxn]
  int nPts_; // number of points along arclength returned by kinematics
  int nInterp_; // number of points to interpolate on backbone
  Eigen::Matrix<double, 8, Eigen::Dynamic> markersOut_; // Matrix for storing marker output to rviz (used in endonasal_teleop)

};
