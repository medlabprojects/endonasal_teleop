#pragma once

#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include <Tube.h>
#include <BasicFunctions.h>
#include <Options.h>
#include <Utility.h>
#include "spline.h"
#include "RoboticsMath.h"

namespace medlab
{
typedef CTR::Functions::constant_fun< Eigen::Vector2d > CurvFun;
typedef std::tuple< CTR::Tube<CurvFun>, CTR::Tube<CurvFun>, CTR::Tube<CurvFun> > Cannula3; // CTR3Robot architecture
typedef CTR::Tube< CurvFun > TubeType;
typedef CTR::DeclareOptions < CTR::Option::ComputeJacobian, CTR::Option::ComputeGeometry, CTR::Option::ComputeStability, CTR::Option::ComputeCompliance >::options OType;

struct CTR3RobotParams {

  // Material Properties
  double E;
  double G;

  // Tube 1 Geometry
  double L1;
  double Lt1;
  double k1;
  double OD1;
  double ID1;

  // Tube 2 Geometry
  double L2;
  double Lt2;
  double k2;
  double OD2;
  double ID2;

  // Tube 3 Geometry
  double L3;
  double Lt3;
  double k3;
  double OD3;
  double ID3;
};

struct CTR3KinematicsInputVector { // format of the state vector fed into Kinematics_with_dense_output()
  Eigen::Vector3d PsiL;
  Eigen::Vector3d Beta;
  Eigen::Vector3d Ftip;
  Eigen::Vector3d Ttip;
};

struct InterpRet {      // Interpolated CTR3 Backbone
  Eigen::VectorXd s;
  Eigen::MatrixXd p;
  Eigen::MatrixXd q;
};

struct WeightingRet { // Joint Limits Weighting Matrix (Configuration Dependent)
  RoboticsMath::Matrix6d W;
  Eigen::VectorXd dh;
};

struct KinOut {  // used by ResolvedRates.init() -> Online Resolved Rates Loop
  Eigen::Vector3d Ptip;
  Eigen::Vector4d Qtip;
  Eigen::Vector4d Qbishop;
  Eigen::Vector3d Alpha;
  RoboticsMath::Matrix6d Jtip; // Jbody
  double Stability;
};


}
