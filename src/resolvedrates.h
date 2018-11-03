#ifndef RESOLVEDRATES_H
#define RESOLVEDRATES_H

#include <QCoreApplication>
#include <QVector>

#include "Kinematics.h"
#include "BasicFunctions.h"
#include "Tube.h"
#include "spline.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <ros/console.h>

#include <Mtransform.h>
#include <bits/stdc++.h>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <cmath>


using namespace Eigen;
using namespace CTR;
using namespace CTR::Functions;
using namespace std;
using std::tuple;

typedef Eigen::Matrix<double,4,4> Matrix4d;
typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,8,1> Vector8d;
typedef Eigen::Matrix<double,7,1> Vector7d;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef tuple < Tube< constant_fun< Vector2d > >,
                Tube< constant_fun< Vector2d > >,
                Tube< constant_fun< Vector2d > > > Cannula3; // CTR3Robot architecture
typedef constant_fun<Eigen::Vector2d> CurvFun;
typedef std::tuple< Tube<CurvFun>, Tube<CurvFun> > CannulaT;
typedef DeclareOptions< Option::ComputeJacobian, Option::ComputeGeometry, Option::ComputeStability, Option::ComputeCompliance>::options OType;


struct CTR3RobotParams {  // These are used in defineRobot();
  // Number of Tubes
  typedef Tube< constant_fun< Vector<2>::type> > T1_type;
  typedef Tube< constant_fun< Vector<2>::type> > T2_type;
  typedef Tube< constant_fun< Vector<2>::type> > T3_type;

  // Material Properties
  double E = 60E9;
  double G = 60E9 / 2.0 / 1.33;

  // Tube 1 Geometry
  double L1 = 222.5E-3;
  double Lt1 = L1 - 42.2E-3;
  double k1 = (1.0/63.5E-3);
  double OD1 = 1.165E-3;
  double ID1 = 1.067E-3;

  // Tube 2 Geometry
  double L2 = 163E-3;
  double Lt2 = L2 - 3E-3;
  double k2 = (1.0/51.2E-3);
  double OD2 = 2.0574E-3;
  double ID2 = 1.6002E-3;

  // Tube 3 Geometry
  double L3 = 104.4E-3;
  double Lt3 = L3 - 21.4E-3;
  double k3 = (1.0/71.4E-3);
  double OD3 = 2.540E-3;
  double ID3 = 2.2479E-3;
};

struct CTR3ModelStateVector { // format of the state vector fed into Kinematics_with_dense_output()
  Eigen::Vector3d PsiL_;
  Eigen::Vector3d Beta_;
  Eigen::Vector3d Ftip_;
  Eigen::Vector3d Ttip_;
};

struct KinOut {  // used by ResolvedRates.init() -> Online Resolved Rates Loop
  Eigen::Vector3d p_;
  Eigen::Vector4d q_;
  Eigen::Vector3d p2_;
  Eigen::Vector4d q2_;
  Eigen::Vector3d alpha_;
  Eigen::Vector3d psiBeta_;
  Matrix6d J1_;
  Matrix6d J2_;
  Matrix6d J3_;
  Matrix6d J4_;
  Matrix6d J5_;
  Matrix6d J6_;
};

struct InterpRet { // Interpolated CTR3
  Eigen::VectorXd s;
  Eigen::MatrixXd p;
  Eigen::MatrixXd q;
};

struct WeightingRet { // Joint Limits Weighting Matrix (Configuration Dependent)
  Eigen::Matrix6d W;
  Eigen::Vector3d dh;
};


class ResolvedRates
{
public:

  ResolvedRates();
  ~ResolvedRates();
  bool init(std::string Name);
  bool setRRGains(double ScaleFactor, double LambdaTracking, double LambdaDamping);
  bool setInputDeviceTransform(Matrix4d TRegistration);

private:

  // Input Device
  int ButtonState_ = 0;
  int ButtonStatePrev_ = 0;
  bool JustClutched_ = false;
  Matrix4d TRegistration_;
  Matrix4d ControllerDelta_ControllerCoords_;
  Matrix4d PrevControllerInv_;
  Matrix4d ControllerDelta_CannulaCoords_;
  Matrix4d InputDevicePose_;
  Matrix4d IDFrameAtClutch_;
  Matrix4d RobotTipFrameAtClutch_;
  Matrix4d RobotTipFrameCur_;

  // Resolved Rates Loop
  CTR3ModelStateVector QFullVec_;
  Vector6d QVec_;
  CTR3RobotParams Params_;
  //double ScaleRot_; // transmission scaling for motors (remove?)
  //double ScaleTrans_;
  //double ScaleTransOuter_;
  Eigen::Vector3d PTipCur_; // <<-- duplicates?
  Eigen::Vector4d QTipCur_;
  Eigen::Vector3d AlphaCur_;
  Matrix6d JCur_;
  Matrix6d WTracking_;
  Matrix6d WDamping_;
  Matrix6d WJointLims_;
  Eigen::Matrix3d DBetaDx_;
  Matrix6d DqBetaDqX_;
  Vector6d DeltaQx_;

  // Kinematics/Visualization
  Eigen::Vector3d PTip_;
  Eigen::Vector3d PTip2_;
  Eigen::Vector4d QTip_;
  Eigen::Vector4d QTip2_;
  Eigen::Matrix3d RTip_;
  Eigen::Vector3d Alpha_;
  int lastPosInterp_;
  InterpRet InterpolatedBackboneCur_;

  // Math Vars
  Eigen::Vector3d Zerovec_;
  double Theta_;
  double ACosArg_;
  double Trace_;
  Eigen::Matrix3d LogR_;
  double LogRMag_;
  Vector6d DeltaQ_;
  Matrix6d A_;
  Matrix6d JStar_;

  // Math Methods
  double deg2rad(double degrees);
  double sgn(double x);
  double vectornorm(Eigen::Vector3d v);
  Eigen::Matrix4d assembleTransformation(Eigen::Matrix3d Rot, Eigen::Vector3d Trans);
  Eigen::Matrix3d orthonormalize(Eigen::Matrix3d R);
  Eigen::Matrix3d quat2rotm(Eigen::Vector4d Quat);
  Eigen::Vector4d rotm2quat(Eigen::Matrix3d R);
  Eigen::Matrix3d hat3(Eigen::Vector3d X);
  Eigen::Matrix6d MAdjoint(Eigen::Matrix4d T);
  Eigen::Matrix6d Adjoint_pq(Eigen::Vector3d p, Eigen::Vector4d q);
  Eigen::Matrix4d inverseTransform(Eigen::Matrix4d T);
  Eigen::Matrix7d collapseTransform(Eigen::Matrix4d T);
  Eigen::Vector4d slerp(Eigen::Vector4d Qa, Eigen::Vector4d Qb, double t);
  Eigen::Matrix<double,4,Eigen::Dynamic> quatInterp(Eigen::Matrix<double,4,Eigen::Dynamic> refQuat,
                                                    Eigen::VectorXd refArcLengths,
                                                    Eigen::VectorXd interpArcLengths);
  void getCofactor(double A[6][6], double Temp[6][6], int P, int Q, int N);
  double determinant(double A[6][6], int N);
  void adjoint(double A[6][6], double Adj[6][6]);
  void inverse(double A[6][6], double Inverse[6][6]);

  // Robotics Methods
  auto defineRobot(CTR3RobotParams params);
  auto callKinematics(CTR3ModelStateVector newStateVector);
  auto forwardKinematics(auto kin);
  InterpRet interpolateBackbone(Eigen::VectorXd sRef, Eigen::MatrixXd poseDataRef, int nPts);
  Vector6d saturateJointVelocities(Vector6d DeltaQx, int NodeFreq);
  Vector6d transformBetaToX(Vector6d Qbeta, Eigen::Vector3d L);
  Vector6d transformXToBeta(Vector6d Qx, Eigen::Vector3d L);
  double dhFunction(double Xmin, double Xmax, double X);
  Eigen::Vector3d limitBetaValsSimple(Eigen::Vector3d XIn, Eigen::Vector3d L);
  Eigen::Vector3d limitBetaValsHardware(Eigen::Vector3d BetaIn);
  Eigen::Vector3d limitBetaValsBimanualAlgorithm(Eigen::Vector3d BetaIn, Eigen::Vector3d LIn);
  double getVMag(const Eigen::VectorXd& E, double VMax, double VMin, double EMax, double EMin,
                 double ConvergeRadius);
  WeightingRet getWeightingMatrix(Eigen::Vector3d X,
                                  Eigen::Vector3d dhPrev,
                                  Eigen::Vector3d L, double lambda);
  Eigen::Vector3d scaleInputDeviceVelocity(Vector3d DesTwistDelta);

};



#endif // RESOLVEDRATES_H
