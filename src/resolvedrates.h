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
using std::tuple;
using namespace std;

typedef Eigen::Matrix<double,4,4> Matrix4d;
typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,7,1> Vector7d;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef tuple < Tube< constant_fun< Vector2d > >,
                Tube< constant_fun< Vector2d > >,
                Tube< constant_fun< Vector2d > > > Cannula3;
typedef constant_fun<Eigen::Vector2d> CurvFun;
typedef std::tuple< Tube<CurvFun>, Tube<CurvFun> > CannulaT;
typedef DeclareOptions< Option::ComputeJacobian, Option::ComputeGeometry, Option::ComputeStability, Option::ComputeCompliance>::options OType;


struct CTR3ModelStateVector {
  Eigen::Vector3d PsiL_;
  Eigen::Vector3d Beta_;
  Eigen::Vector3d Ftip_;
  Eigen::Vector3d Ttip_;
};

struct KinOut {
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

struct InterpRet {
  Eigen::VectorXd s;
  Eigen::MatrixXd p;
  Eigen::MatrixXd q;
};

struct WeightingRet {
  Eigen::Matrix6d W;
  Eigen::Vector3d dh;
};


class ResolvedRates
{
public:

  ResolvedRates();
  ~ResolvedRates();
  bool init(std::string Name);
  bool setRRGains(double ScaleFactor, double LambdaTracking, double LambdaDamping, double LambdaJointLims);
  bool setInputDeviceTransform(Matrix4d TRegistration);


private:

  // Setup Methods
  bool setupWeightingMatrices();
  bool setupInputDeviceRegistration();

  bool setQStart();
  bool setL();


  // Input Device
  int ButtonState_ = 0;
  int ButtonStatePrev_ = 0;
  bool JustClutched_ = false;
  Matrix4d TRegistration;
  Matrix4d ControllerDelta_ControllerCoords_;
  Matrix4d PrevControllerInv_;
  Matrix4d ControllerDelta_CannulaCoords_;
  Matrix4d InputDevicePose_;
  Matrix4d IDFrameAtClutch_;
  Matrix4d RobotTipFrameAtClutch_;
  Matrix4d RobotTipFrameCur_;

  // Cannula Vars
  CTR3ModelStateVector QStart_;
  double ScaleRot_;
  double ScaleTrans_;
  double ScaleTransOuter_;
  Eigen::Vector3d PTipCur_;
  Eigen::Vector4d QTipCur_;
  Eigen::Vector3d AlphaCur_;
  Matrix6d JCur_;
  Eigen::Vector3d PTip_;
  Eigen::Vector4d QTip_;
  Eigen::Matrix3d RTip_;
  Eigen::Vector3d Alpha_;

  Matrix6d WTracking_;
  Matrix6d WDamping_;
  Matrix6d WJointLims_;
  Eigen::Matrix3d DBetaDx_;
  Matrix6d DqBetaDqX_;

  Vector6d QVec;
  Vector6d DeltaQx;

  Eigen::Vector3d L_; // lengths of tubes

  // Kinematics
  int lastPosInterp_;

  // Math Vars
  Eigen::Vector3d Zerovec_;
  double Theta_;
  double ACosArg_;
  double Trace_;
  Eigen::Matrix3d LogR;
  double LogRMag;
  Vector6d DeltaQ;
  Matrix6d A;
  Matrix6d JStar;

  // Math Methods
  double deg2rad(double degrees);
  double vectornorm(Eigen::Vector3d v);
  Eigen::Matrix3d orthonormalize(Eigen::Matrix3d R);
  Eigen::Matrix4d assembleTransformation(Eigen::Matrix3d Rot, Eigen::Vector3d Trans);
  Eigen::Matrix3d quat2rotm(Eigen::Vector4d Quat);
  Eigen::Vector4d rotm2quat(Eigen::Matrix3d R);
  Eigen::Matrix3d hat3(Eigen::Vector3d X);
  Eigen::Matrix6d MAdjoint(Eigen::Matrix4d T);
  Eigen::Matrix6d Adjoint_pq(Eigen::Vector3d p, Eigen::Vector4d q);
  Eigen::Matrix4d inverseTransform(Eigen::Matrix4d T);
  double sgn(double x);
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
  auto kinematicsCall(auto kinCall);
  auto forwardKinematics(auto kin);
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
