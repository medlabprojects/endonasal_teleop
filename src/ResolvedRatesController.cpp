#include "ResolvedRatesController.h"

ResolvedRatesController::ResolvedRatesController(medlab::Cannula3 cannula, medlab::CTR3RobotParams params):
  robot_(cannula,params)
{
  // we might want to specify what control scheme to use, which weighting matrices
  // --> might also make sense to overload/have a default for the control scheme
  lambdaTracking_ = 1.0E8;
  lambdaDamping_ = 0.5;
  lambdaJL_ = 1.0E3;
}

ResolvedRatesController::~ResolvedRatesController()
{

}

void ResolvedRatesController::init()
{
  // Construct w/ CTR3Robot with params and base frame

  // Set Input Device
  // Resolved Rates Prep --> extract ptip, qtip, Jtip, calc WJointLims

  RoboticsMath::Vector6d qHome;
  qHome << 0.0, 0.0, 0.0, -160.9E-3, -127.2E-3, -86.4E-3;
  robot_.init(qHome);

  // Compute initial Weighting matrices
  SetTrackingGain(lambdaTracking_);
  SetDampingGain(lambdaDamping_);
  dhPrev_ = Eigen::Vector3d::Zero();

  // Compute WJointLims_
  //Eigen::Vector3d betas = qHome.bottomRows(3);
  medlab::CTR3RobotParams params = robot_.GetCurRobotParams();
  Eigen::Vector3d L;
  L << params.L1, params.L2, params.L3;
  RoboticsMath::Vector6d x = transformBetaToX(qHome,L);
  Eigen::Vector3d xBeta;
  xBeta = x.bottomRows(3);
  medlab::WeightingRet JLWeightingRet = computeJLWeightingMatrix(xBeta,dhPrev_, L);
  WJointLims_ = JLWeightingRet.W;
  dhPrev_ = JLWeightingRet.dh;


}

RoboticsMath::Vector6d ResolvedRatesController::step(RoboticsMath::Vector6d desTwist)  // TODO:  input is commanded twist  --- output would be joint values (alpha, beta)
{
  currentLimitFlags_.clear();

  // TODO: also have a converged flag or something to let us know that the robot is where we commanded and not trying its best
  // --> save this as a member variable and have a getter
  RoboticsMath::Vector6d desQ;
//  desQ = [desPsi1 desPsi2 desPsi3 desBeta1 desBeta2 desBeta3]

  return desQ;
}

bool ResolvedRatesController::SetTrackingGain(double LambdaTracking)
{
  WTracking_(0, 0) = LambdaTracking;
  WTracking_(1, 1) = LambdaTracking;
  WTracking_(2, 2) = LambdaTracking;
  WTracking_(3,3) = 0.0;
  WTracking_(4,4) = 0.0;
//  WTracking_(3, 3) = 0.1*LambdaTracking*(180.0 / M_PI / 2.0)*(180.0 / M_PI / 2.0);
//  WTracking_(4, 4) = 0.1*LambdaTracking*(180.0 / M_PI / 2.0)*(180.0 / M_PI / 2.0);
  WTracking_(5, 5) = LambdaTracking*(180.0 / M_PI / 2.0)*(180.0 / M_PI / 2.0)*1E1; // Increase to track roll
  return true;
}

bool ResolvedRatesController::SetDampingGain(double LambdaDamping)
{
  double thetadeg = 2.0; // degrees to damp as much as 1mm
  WDamping_(0, 0) = LambdaDamping*(180.0 / M_PI / thetadeg)*(180.0 / M_PI / thetadeg);
  WDamping_(1, 1) = LambdaDamping*(180.0 / M_PI / thetadeg)*(180.0 / M_PI / thetadeg);
  WDamping_(2, 2) = LambdaDamping*(180.0 / M_PI / thetadeg)*(180.0 / M_PI / thetadeg);
  WDamping_(3, 3) = LambdaDamping*5.0E8;
  WDamping_(4, 4) = LambdaDamping*5.0E8;
  WDamping_(5, 5) = LambdaDamping*5.0E8;
  return true;
}

//bool ResolvedRatesController::SetInputDeviceTransform(Eigen::Matrix4d TRegistration)
//{
//	return true;
//}

CTR3Robot ResolvedRatesController::GetRobot()
{
  return robot_;
}

RoboticsMath::Vector6d ResolvedRatesController::saturateJointVelocities(RoboticsMath::Vector6d delta_qx, int node_freq)
{
  double max_rot_speed = 0.8; // rad/sec
  double max_trans_speed = 5.0; // mm/sec

  RoboticsMath::Vector6d commanded_speed = delta_qx*node_freq;
  RoboticsMath::Vector6d saturated_speed = commanded_speed;
  RoboticsMath::Vector6d delta_qx_sat;

  // saturate tip rotations
  for (int i = 0; i<3; i++)
  {
    if (fabs(commanded_speed(i)) > max_rot_speed)
    {
      saturated_speed(i) = (commanded_speed(i) / fabs(commanded_speed(i)))*max_rot_speed;

      if(i==0){
        currentLimitFlags_.push_back(ResolvedRatesController::LIMIT_FLAG::VELOCITY_T1_ROT); }
      else if (i == 1){
        currentLimitFlags_.push_back(ResolvedRatesController::LIMIT_FLAG::VELOCITY_T2_ROT); }
      else if (i == 2){
        currentLimitFlags_.push_back(ResolvedRatesController::LIMIT_FLAG::VELOCITY_T3_ROT); }

      //std::cout << "Tube rotation speed saturated for tube" << i << std::endl << std::endl;
    }
    delta_qx_sat(i) = saturated_speed(i) / node_freq;
  }

  // saturate translations
  for (int i = 3; i<6; i++)
  {
    if (fabs(commanded_speed(i)) > max_trans_speed)
    {
      saturated_speed(i) = (commanded_speed(i) / fabs(commanded_speed(i)))*max_trans_speed;

      if(i==0){
        currentLimitFlags_.push_back(ResolvedRatesController::LIMIT_FLAG::VELOCITY_T1_TRANS); }
      else if (i == 1){
        currentLimitFlags_.push_back(ResolvedRatesController::LIMIT_FLAG::VELOCITY_T2_TRANS); }
      else if (i == 2){
        currentLimitFlags_.push_back(ResolvedRatesController::LIMIT_FLAG::VELOCITY_T3_TRANS); }
      //std::cout << "Tube translation speed saturated for tube" << i << std::endl << std::endl;
    }
    delta_qx_sat(i) = saturated_speed(i) / node_freq;
  }

  return delta_qx_sat;
}

RoboticsMath::Vector6d ResolvedRatesController::transformBetaToX(RoboticsMath::Vector6d qbeta, Eigen::Vector3d L)
{
  RoboticsMath::Vector6d qx;
  qx << qbeta(0), qbeta(1), qbeta(2), 0, 0, 0;
  qx(3) = L(0) - L(1) + qbeta(3) - qbeta(4);
  qx(4) = L(1) - L(2) + qbeta(4) - qbeta(5);
  qx(5) = L(2) + qbeta(5);
  return qx;
}

RoboticsMath::Vector6d ResolvedRatesController::transformXToBeta(RoboticsMath::Vector6d qx, Eigen::Vector3d L)
{
  RoboticsMath::Vector6d qbeta;
  qbeta << qx(0), qx(1), qx(2), 0, 0, 0;
  qbeta(3) = qx(3) + qx(4) + qx(5) - L(0);
  qbeta(4) = qx(4) + qx(5) - L(1);
  qbeta(5) = qx(5) - L(2);
  return qbeta;
}

double ResolvedRatesController::dhFunction(double xmin, double xmax, double x)
{
  double dh = fabs((xmax - xmin)*(xmax - xmin)*(2 * x - xmax - xmin) / (4 * (xmax - x)*(xmax - x)*(x - xmin)*(x - xmin)));
  return dh;
}

medlab::WeightingRet ResolvedRatesController::computeJLWeightingMatrix(Eigen::Vector3d x, Eigen::Vector3d dhPrev, Eigen::Vector3d L)
{
  RoboticsMath::Matrix6d W = RoboticsMath::Matrix6d::Identity();

  // No penalties on the rotational degrees of freedom (they don't have any joint limits)
  // Therefore leave the first three entries in W as 1.

  double eps = 2e-3;

  // x1:
  double x1min = eps;
  double x1max = L(0) - L(1) - eps;
  double x1 = x(0);
  double dh1 = dhFunction(x1min, x1max, x1);
  W(3, 3) = (dh1 >= dhPrev(0))*(1 + dh1) + (dh1 < dhPrev(0)) * 1;
  //W(3,3) = 1+dh1;
  W(3, 3) *= lambdaJL_;

  // x2:
  double x2min = eps;
  double x2max = L(1) - L(2) - eps;
  double x2 = x(1);
  double dh2 = dhFunction(x2min, x2max, x2);
  W(4, 4) = (dh2 >= dhPrev(1))*(1 + dh2) + (dh2 < dhPrev(1)) * 1;
  //W(4,4) = 1+dh2;
  W(4, 4) *= lambdaJL_;

  // x3:
  double x3min = eps;
  double x3max = L(2) - eps;
  double x3 = x(2);
  double dh3 = dhFunction(x3min, x3max, x3);
  W(5, 5) = (dh3 >= dhPrev(2))*(1 + dh3) + (dh3 < dhPrev(2)) * 1;
  //W(5,5) = 1+dh3;
  W(5, 5) *= lambdaJL_;

  Eigen::Vector3d dh;
  dh << dh1, dh2, dh3;

  medlab::WeightingRet output;
  output.W = W;
  output.dh = dh;
  return output;
}

Eigen::Vector3d ResolvedRatesController::limitBetaValsSimple(Eigen::Vector3d x_in, Eigen::Vector3d L)
{
  Eigen::Vector3d x = x_in;
  double epsilon = 0.5e-3;  // keep a 0.5 mm minimum margin

  // check tube 3 first:
  if (x(2) < epsilon)
  {
    x(2) = epsilon;
    currentLimitFlags_.push_back(ResolvedRatesController::LIMIT_FLAG::T3_FRONT);
    //std::cout << "Tube 3 translation saturated (front)" << std::endl;
  }
  else if (x(2) > L(2) - epsilon)
  {
    x(2) = L(2) - epsilon;
    currentLimitFlags_.push_back(ResolvedRatesController::LIMIT_FLAG::T3_BACK);
    //std::cout << "Tube 3 translation saturated (rear)" << std::endl;
  }

  // now check tube 2:
  if (x(1) < epsilon)
  {
    x(1) = epsilon;
    currentLimitFlags_.push_back(ResolvedRatesController::LIMIT_FLAG::T2_FRONT);
    //std::cout << "Tube 2 translation saturated (front)" << std::endl;
  }
  else if (x(1) > L(1) - L(2) - epsilon)
  {
    x(1) = L(1) - L(2) - epsilon;
    currentLimitFlags_.push_back(ResolvedRatesController::LIMIT_FLAG::T2_BACK);
    //std::cout << "Tube 2 translation saturated (rear)" << std::endl;
  }

  // and last check tube 1:
  if (x(0) < epsilon)
  {
    x(0) = epsilon;
    currentLimitFlags_.push_back(ResolvedRatesController::LIMIT_FLAG::T1_FRONT);
    //std::cout << "Tube 1 translation saturated (front)" << std::endl;
  }
  else if (x(0) > L(0) - L(1) - epsilon)
  {
    x(0) = L(0) - L(1) - epsilon;
    currentLimitFlags_.push_back(ResolvedRatesController::LIMIT_FLAG::T1_BACK);
    //std::cout << "Tube 1 translation saturated (rear)" << std::endl;
  }

  return x;
}

Eigen::Vector3d ResolvedRatesController::limitBetaValsBimanualAlgorithm(Eigen::Vector3d Beta_in, Eigen::Vector3d L_in)
{
  int nTubes = 3;

  // plate thicknesses (constant)
  // eventually we'll want to load them from elsewhere
  // these also need to be updated to correct values for the new robot
  // and this whole algorithm will need to be adapted
  Eigen::Matrix<double, 4, 1> tplus;
  tplus << 0.0, 10e-3, 10e-3, 10e-3;

  Eigen::Matrix<double, 4, 1> tminus;
  tminus << 0.0, 10e-3, 10e-3, 10e-3;

  Eigen::Matrix<double, 4, 1> extMargin;
  extMargin << 1e-3, 1e-3, 10e-3, 1e-3;
  // tube 2 is different because we have to make sure it doesn't knock off the end effector

  double tp = 25e-3;

  Eigen::Matrix<double, 4, 1> L;
  L << 3 * L_in(0), L_in(0), L_in(1), L_in(2);

  Eigen::Matrix<double, 4, 1> beta;
  beta << -2 * L_in(0), Beta_in(0), Beta_in(1), Beta_in(2);

  // starting with tube 1, correct any translational joint limit violations
  for (int i = 1; i <= nTubes; i++)
  {
    // if it's going to hit the carriage behind it, move it up:
    if (beta(i) - tminus(i) < beta(i - 1) + tplus(i - 1))
    {
      beta(i) = beta(i - 1) + tplus(i - 1) + tminus(i);
    }

    // if it's going to not leave space for the other carriages in front of it, move it back:
    double partial_sum = 0.0;
    for (int k = i + 1; k <= nTubes; k++)
    {
      partial_sum += tplus(k) + tminus(k);
    }
    if (beta(i) + tplus(i) > -tp - partial_sum)
    {
      beta(i) = -tp - partial_sum - tplus(i);
    }

    // if the tube is getting too close to the end of the next tube out, move it up:
    if (beta(i) + L(i) > beta(i - 1) + L(i - 1) - extMargin(i))
    {
      beta(i) = beta(i - 1) + L(i - 1) - L(i) - extMargin(i);
    }

    // if the tube is going to retract all the way behind the front plate, move it up:
    if (beta(i) + L(i) - 0.001*(nTubes - i + 1) < 0.0)
    {
      beta(i) = -L(i) + 0.001*(nTubes - i + 1);
    }
  }

  Eigen::Vector3d Beta;
  Beta << beta(1), beta(2), beta(3);
  return Beta;
}

RoboticsMath::Vector6d ResolvedRatesController::scaleInputDeviceVelocity(RoboticsMath::Vector6d desTwistDelta)
{
  RoboticsMath::Vector6d scaledDesTwistDelta = desTwistDelta;

  //double pStepMax = 2.0e-3; // 2 mm
  double vMax = 0.1; // [m/s]
  double pStepMax = vMax / rosLoopRate_; // [mm]

  double pErr = desTwistDelta.topRows<3>().norm();
  double gain = 1.0;
  if (pErr > pStepMax)
  {
    gain = pStepMax / pErr;
  }
  scaledDesTwistDelta.topRows<3>() *= gain;

  //double angleStepMax = 0.05; //0.05 radians
  double wMax = 2.5; // [rad/s]
  double angleStepMax = wMax / rosLoopRate_;
  double angleErr = desTwistDelta.bottomRows<3>().norm();
  gain = 1.0;
  if (angleErr > angleStepMax)
  {
    gain = angleStepMax / angleErr;
  }
  scaledDesTwistDelta.bottomRows<3>() *= gain;

  return scaledDesTwistDelta;
}
