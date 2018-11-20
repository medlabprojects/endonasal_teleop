#include "CTR3Robot.h"
#include <vector>
#include <tuple>
#include "RoboticsMath.h"
#include "MedlabTypes.h"
#include <Kinematics.h>

CTR3Robot::CTR3Robot(medlab::Cannula3 cannula, medlab::CTR3RobotParams params):
  cannula_(cannula), // Set cannula3
  currCannulaParams_(params)
{

}

bool CTR3Robot::init()
{
  // init with home position at zero
  RoboticsMath::Vector6d qHome(RoboticsMath::Vector6d::Zero());
  return init(qHome);
}

bool CTR3Robot::init(RoboticsMath::Vector6d qHome)
{
  bool success = false;

  nInterp_ = 200;
  qHome_ = qHome;                                                       // Store qHome
  //  qHome << 0.0, 0.0, 0.0, -160.9E-3, -127.2E-3, -86.4E-3; //TODO: this needs to be updated for new tubes
  currKinematicsInputVector_.PsiL = qHome_.head(3);			// Store input vector as qHome
  currKinematicsInputVector_.Beta = qHome_.tail(3);
  currKinematicsInputVector_.Ftip = Eigen::Vector3d::Zero();
  currKinematicsInputVector_.Ttip = Eigen::Vector3d::Zero();
  currQVec_ << qHome_;                                                  // Store condensed input vector
  currKinematics = callKinematicsWithDenseOutput(currKinematicsInputVector_); // currInterpolatedBackbone_ set in here

  success = true;

  return success;
}

medlab::Cannula3 CTR3Robot::GetCannula()
{
  return cannula_;
}

medlab::CTR3RobotParams CTR3Robot::GetCurRobotParams()
{
  return currCannulaParams_;
}

bool CTR3Robot::SetCurrKinematicsInputVector(medlab::CTR3KinematicsInputVector kinematicsInputVector)
{
  currKinematicsInputVector_ = kinematicsInputVector;
  return true;
}
medlab::CTR3KinematicsInputVector CTR3Robot::GetCurrKinematicsInputVector()
{
  return currKinematicsInputVector_;
}

bool CTR3Robot::SetCurrQVec(RoboticsMath::Vector6d qVec)
{
  currQVec_ = qVec;
  return true;
}
RoboticsMath::Vector6d CTR3Robot::GetCurrQVec()
{
  return currQVec_;
}

bool CTR3Robot::SetInterpolatedBackbone(medlab::InterpRet interpolatedBackbone)
{
  currInterpolatedBackbone_ = interpolatedBackbone;
  return true;
}
medlab::InterpRet CTR3Robot::GetInterpolatedBackbone()
{
  return currInterpolatedBackbone_;
}

int CTR3Robot::GetNPts() {
  return nPts_;
}

int CTR3Robot::GetNInterp() {
  return nInterp_;
}

medlab::KinOut CTR3Robot::callKinematicsWithDenseOutput(medlab::CTR3KinematicsInputVector newKinematicsInput)
{

  medlab::KinOut kinoutput;
  auto ret1 = CTR::Kinematics_with_dense_output(cannula_, newKinematicsInput, medlab::OType());

  RoboticsMath::Matrix6d J;
  J = CTR::GetTipJacobianForTube1(ret1.y_final);

  double Stability;
  Stability = CTR::GetStability(ret1.y_final);

  nPts_ = ret1.arc_length_points.size();
  double* ptr = &ret1.arc_length_points[0];
  Eigen::Map<Eigen::VectorXd> s(ptr, nPts_);

  Eigen::MatrixXd poseData = CTR3Robot::forwardKinematics(ret1);

  medlab::InterpRet interpResults = interpolateBackbone(s, poseData, nInterp_);

  Eigen::MatrixXd poseDataOut(8, nInterp_ + nPts_);
  poseDataOut = Eigen::MatrixXd::Zero(8, nInterp_ + nPts_);
  Eigen::RowVectorXd ones(nInterp_ + nPts_);
  ones.fill(1);
  Eigen::VectorXd sOut = interpResults.s;
  poseDataOut.topRows(3) = interpResults.p;
  poseDataOut.middleRows<4>(3) = interpResults.q;
  poseDataOut.bottomRows(1) = ones;

  Eigen::Vector3d pTip;
  Eigen::Vector4d qBishop;
  Eigen::Matrix3d RBishop;
  Eigen::Matrix3d Rtip;
  Eigen::Vector4d qTip;


  pTip = ret1.pTip;
  qBishop = ret1.qTip;
  RBishop = RoboticsMath::quat2rotm(qBishop);
  Eigen::Matrix3d rotate_psiL = Eigen::Matrix3d::Identity();
  rotate_psiL(0,0) = cos(newKinematicsInput.PsiL(0));
  rotate_psiL(0,1) = -sin(newKinematicsInput.PsiL(0));
  rotate_psiL(1,0) = sin(newKinematicsInput.PsiL(0));
  rotate_psiL(1,1) = cos(newKinematicsInput.PsiL(0));
  Rtip = RBishop*rotate_psiL;
  qTip = RoboticsMath::rotm2quat(Rtip);

  // Parse kinret into currKinematics_
  kinoutput.Ptip[0] = pTip[0];
  kinoutput.Ptip[1] = pTip[1];
  kinoutput.Ptip[2] = pTip[2];
  kinoutput.Qtip[0] = qTip[0];
  kinoutput.Qtip[1] = qTip[1];
  kinoutput.Qtip[2] = qTip[2];
  kinoutput.Qtip[3] = qTip[3];
  kinoutput.Qbishop[0] = qBishop[0];
  kinoutput.Qbishop[1] = qBishop[1];
  kinoutput.Qbishop[2] = qBishop[2];
  kinoutput.Qbishop[3] = qBishop[3];
  kinoutput.Alpha[0] = ret1.y_final.Psi[0];
  kinoutput.Alpha[1] = ret1.y_final.Psi[1];
  kinoutput.Alpha[2] = ret1.y_final.Psi[2];
  kinoutput.Stability = Stability;
  kinoutput.Jtip = J;

  currKinematics = kinoutput;

  return kinoutput;
}

Eigen::MatrixXd CTR3Robot::forwardKinematics(auto kin)
{

  // Pick out arc length points
  int nPts = kin.arc_length_points.size();
  double* ptr = &kin.arc_length_points[0];
  Eigen::Map<Eigen::VectorXd> s(ptr, nPts);

  double zeroIndex;
  Eigen::VectorXd zeroIndexVec;
  Eigen::Vector3d pZero;
  Eigen::Vector4d qZero;
  Eigen::Matrix4d gZero;
  Eigen::Matrix4d gStarZero;
  Eigen::Matrix4d gStarL;

  int count = 0;
  for (int i=0; i < s.size(); ++i)
  {
    if (s(i) == 0) {
      zeroIndexVec.resize(count+1);
      zeroIndexVec(count) = (double) i;
      count ++;
    }
  }

  zeroIndex = zeroIndexVec(count-1);
  pZero = kin.dense_state_output.at( zeroIndex ).p;
  qZero = kin.dense_state_output.at( zeroIndex ).q;
  gZero = RoboticsMath::assembleTransformation(RoboticsMath::quat2rotm(qZero),pZero);
  gStarZero = RoboticsMath::assembleTransformation(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  gStarL = gStarZero*RoboticsMath::inverseTransform(gZero);

  Eigen::MatrixXd poseData(8,nPts);
  for (int i =0; i < nPts; ++i)
  {
    Eigen::Vector3d pi = kin.dense_state_output.at( i ).p;
    Eigen::Vector4d qi = kin.dense_state_output.at( i ).q;
    Eigen::Matrix4d gi = RoboticsMath::assembleTransformation(RoboticsMath::quat2rotm(qi),pi);
    Eigen::Matrix4d gStari = gStarL*gi;
    RoboticsMath::Vector8d xi;
    xi.fill(0);
    xi.head<7>() = RoboticsMath::collapseTransform(gStari);
    xi(7) = 1.0;
    poseData.col(i) = xi;
  }

  return poseData;
}

medlab::InterpRet CTR3Robot::interpolateBackbone(Eigen::VectorXd sRef, Eigen::MatrixXd poseDataRef, int nPts)
{
  Eigen::Matrix<double, 4, Eigen::Dynamic> qRef;
  qRef = poseDataRef.middleRows<4>(3);

  // Create a zero to one list for ref arc lengths
  int nRef = sRef.size();
  double totalArcLength = sRef(0) - sRef(nRef - 1);
  Eigen::VectorXd sRef0Vec(nRef);
  sRef0Vec.fill(sRef(nRef-1));
  Eigen::VectorXd zeroToOne = (1 / totalArcLength)*(sRef - sRef0Vec);

  // Create a zero to one vector including ref arc lengths & interp arc lengths (evenly spaced)
  int nPtsTotal = nPts + nRef;

  Eigen::VectorXd xxLinspace(nPts);
  xxLinspace.fill(0.0);
  xxLinspace.setLinSpaced(nPts, 1.0, 0.0);
  Eigen::VectorXd xxUnsorted(nPtsTotal);
  xxUnsorted << xxLinspace, zeroToOne;
  std::sort(xxUnsorted.data(), xxUnsorted.data() + xxUnsorted.size());
  Eigen::VectorXd xx = xxUnsorted.reverse(); //Rich's interpolation functions call for descending order

  // List of return arc lengths in the original scaling/offset
  Eigen::VectorXd xxSRef0Vec(nPtsTotal);
  xxSRef0Vec.fill(sRef(nRef-1));
  Eigen::VectorXd sInterp = totalArcLength*xx + xxSRef0Vec;

  // Interpolate to find list of return quaternions
  Eigen::MatrixXd qInterp = RoboticsMath::quatInterp(qRef, zeroToOne, xx);

  // Interpolate to find list of return positions
  Eigen::VectorXd sInterpSpline = sInterp.reverse(); // spline requires ascending order

  std::vector<double> sVec;
  sVec.resize(sRef.size());
  Eigen::VectorXd::Map(&sVec[0], sRef.size()) = sRef.reverse();

  Eigen::VectorXd x = poseDataRef.row(0).reverse(); // interp x
  std::vector<double> xVec;
  xVec.resize(x.size());
  Eigen::VectorXd::Map(&xVec[0], x.size()) = x;
  tk::spline Sx;
  Sx.set_points(sVec, xVec);
  Eigen::VectorXd xInterp(nPtsTotal);
  xInterp.fill(0);
  for (int i = 0; i < nPtsTotal; i++)
  {
    xInterp(i) = Sx(sInterpSpline(i));
  }
  xInterp = xInterp.reverse().eval();

  Eigen::VectorXd y = poseDataRef.row(1).reverse(); // interp y
  std::vector<double> yVec;
  yVec.resize(y.size());
  Eigen::VectorXd::Map(&yVec[0], y.size()) = y;
  tk::spline Sy;
  Sy.set_points(sVec, yVec);
  Eigen::VectorXd yInterp(nPtsTotal);
  yInterp.fill(0);
  for (int i = 0; i < nPtsTotal; i++)
  {
    yInterp(i) = Sy(sInterpSpline(i));
  }
  yInterp = yInterp.reverse().eval();

  Eigen::VectorXd z = poseDataRef.row(2).reverse(); // interp z
  std::vector<double> zVec;
  zVec.resize(z.size());
  Eigen::VectorXd::Map(&zVec[0], z.size()) = z;
  tk::spline Sz;
  Sz.set_points(sVec, zVec);
  Eigen::VectorXd zInterp(nPtsTotal);
  for (int i = 0; i < nPtsTotal; i++)
  {
    zInterp(i) = Sz(sInterpSpline(i));
  }
  zInterp = zInterp.reverse().eval();

  Eigen::MatrixXd pInterp(3, nPtsTotal);
  pInterp.fill(0);
  pInterp.row(0) = xInterp.transpose();
  pInterp.row(1) = yInterp.transpose();
  pInterp.row(2) = zInterp.transpose();

  medlab::InterpRet interpResults;
  interpResults.s = sInterp;
  interpResults.p = pInterp;
  interpResults.q = qInterp;

  currInterpolatedBackbone_ = interpResults;

  return interpResults;

}

medlab::WeightingRet CTR3Robot::computeStabilityWeightingMatrix(RoboticsMath::Vector6d qVec, double S, double sThreshold, double alphaS)
{
  RoboticsMath::Matrix6d W = (exp(1 / (S - sThreshold)) - 1) * RoboticsMath::Matrix6d::Identity();
  RoboticsMath::Vector6d vS1 = RoboticsMath::Vector6d::Zero();
  RoboticsMath::Vector6d dSdq = RoboticsMath::Vector6d::Zero();

  double rotationalStep = 0.05*M_PI / 180.0;
  double translationalStep = 1E-5;
  double step;

  RoboticsMath::Vector6d qFD;
  qFD.block<3,1>(0,0) = qVec.block<3,1>(0,0);
  qFD.block<3,1>(3,0) = qVec.block<3,1>(3,0);

  medlab::CTR3KinematicsInputVector qKinematicsUpper;
  medlab::CTR3KinematicsInputVector qKinematicsLower;

  for (int i=0; i < 6; i++)
  {
    RoboticsMath::Vector6d direction = RoboticsMath::Vector6d::Zero();
    direction(i) = 0;

    if (i < 3)
    {
      step = rotationalStep;
    }
    else
    {
      step = translationalStep;
    }

    RoboticsMath::Vector6d qFDUpper = qFD + step*direction;
    RoboticsMath::Vector6d qFDLower = qFD - step*direction;

    qKinematicsUpper.PsiL = qFDUpper.block<3,1>(0,0);
    qKinematicsUpper.Beta = qFDUpper.block<3,1>(3,0);
    qKinematicsUpper.Ftip << 0.0, 0.0, 0.0;
    qKinematicsUpper.Ttip << 0.0, 0.0, 0.0;

    qKinematicsLower.PsiL = qFDLower.block<3,1>(0,0);
    qKinematicsLower.Beta = qFDLower.block<3,1>(3,0);
    qKinematicsLower.Ftip << 0.0, 0.0, 0.0;
    qKinematicsLower.Ttip << 0.0, 0.0, 0.0;

    auto retUpper = CTR::Kinematics_with_dense_output(cannula_, qKinematicsUpper, medlab::OType());
    auto retLower = CTR::Kinematics_with_dense_output(cannula_, qKinematicsLower, medlab::OType());

    double SUpper = CTR::GetStability(retUpper.y_final);
    double SLower = CTR::GetStability(retLower.y_final);

    dSdq(i) = (SUpper - SLower) / (2*step);

  }

  vS1 = alphaS*dSdq;

  WStability = W;
  vS = vS1;

  medlab::WeightingRet output;
  output.W = W;
  output.dh = vS;
  return output;
}
